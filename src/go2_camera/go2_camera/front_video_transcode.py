#!/usr/bin/env python3
"""
Front camera H264 relay.

Bypasses rclpy/rmw entirely — uses cyclonedds-python directly so the CDR
deserializer only reads the two fields that actually exist on the wire
(time_frame + video720p).  The rmw_cyclonedds layer crashes trying to
read video360p / video180p (which the Go2 does not send), dropping IDR
frames and preventing the decoder from ever starting.
"""
import sys
import threading
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence, uint8, uint64
from cyclonedds.core import WaitSet, ReadCondition, ViewState, InstanceState, SampleState

Gst.init(sys.argv)

# Only declare the two fields the Go2 actually sends.
# CycloneDDS X-Types allows a subscriber to define fewer fields than the
# publisher; extra fields are silently ignored rather than causing errors.
@dataclass
class Go2FrontVideoData(IdlStruct,
                        typename='unitree_go::msg::dds_::Go2FrontVideoData_'):
    time_frame: uint64
    video720p: sequence[uint8]


H264_START = b'\x00\x00\x00\x01'


class FrontVideoRelay:
    def __init__(self, gs_ip, gs_port, framerate, fec_percentage):
        self.pipeline = None
        self.appsrc = None
        self.bus = None
        self.loop = None
        self.gst_thread = None
        self.pts = 0
        self.frame_duration = Gst.SECOND // framerate
        self._nal_count = 0

        pipeline_str = (
            f'appsrc name=src is-live=true block=false format=time '
            f'caps=video/x-h264,stream-format=byte-stream,alignment=nal ! '
            f'h264parse ! '
            f'rtph264pay config-interval=1 ! '
            f'rtpulpfecenc percentage={fec_percentage} ! '
            f'udpsink host={gs_ip} port={gs_port} sync=false'
        )
        print(f'Pipeline: {pipeline_str}', flush=True)

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name('src')

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self._on_gst_message)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.gst_thread.start()
        print('GStreamer pipeline running', flush=True)

    def _on_gst_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f'GStreamer error: {err}', file=sys.stderr, flush=True)
            print(f'Debug: {debug}', file=sys.stderr, flush=True)
            self.loop.quit()
        elif t == Gst.MessageType.EOS:
            print('GStreamer EOS', flush=True)
            self.loop.quit()
        elif t == Gst.MessageType.WARNING:
            warn, _ = message.parse_warning()
            print(f'GStreamer warning: {warn}', flush=True)
        return True

    def push(self, raw: bytes):
        start = raw.find(H264_START)
        if start < 0:
            return
        data = raw[start:]
        if len(data) < 5:
            return

        self._nal_count += 1
        if self._nal_count <= 10 or self._nal_count % 150 == 0:
            nal_type = data[4] & 0x1F
            print(f'NAL {self._nal_count}: {len(data)}B type={nal_type} '
                  f'(0x{data[4]:02x})', flush=True)

        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.pts = self.pts
        buf.duration = self.frame_duration
        self.pts += self.frame_duration
        ret = self.appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            print(f'appsrc push-buffer: {ret}', file=sys.stderr, flush=True)

    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsrc = None
            self.bus = None
        if self.loop and self.loop.is_running():
            self.loop.quit()
        if self.gst_thread:
            self.gst_thread.join(timeout=2)
            self.gst_thread = None


def main():
    import os
    gs_ip = os.environ.get('GS_IP', '192.168.123.100')
    gs_port = int(os.environ.get('GS_PORT', '42074'))
    framerate = int(os.environ.get('FRAMERATE', '30'))
    fec_pct = int(os.environ.get('FEC_PCT', '30'))
    # ROS2 topic /frontvideostream → DDS topic rt/frontvideostream
    dds_topic_name = os.environ.get('VIDEO_TOPIC', 'rt/frontvideostream')

    relay = FrontVideoRelay(gs_ip, gs_port, framerate, fec_pct)

    qos = Qos(
        Policy.Reliability.Reliable(),
        Policy.Durability.Volatile,
        Policy.History.KeepLast(1),
    )

    dp = DomainParticipant(0)
    topic = Topic(dp, dds_topic_name, Go2FrontVideoData, qos=qos)
    reader = DataReader(dp, topic, qos=qos)

    print(f'Subscribed to {dds_topic_name}', flush=True)

    condition = ReadCondition(
        reader,
        ViewState.Any | InstanceState.Any | SampleState.NotRead,
    )
    waitset = WaitSet(dp)
    waitset.attach(condition)

    try:
        while True:
            waitset.wait(1.0)
            for sample in reader.take(condition=condition):
                relay.push(bytes(sample.video720p))
    except KeyboardInterrupt:
        print('Shutting down.', flush=True)
    finally:
        relay.stop()


if __name__ == '__main__':
    main()
