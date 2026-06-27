import sys
import threading
import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from unitree_go.msg import Go2FrontVideoData

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(sys.argv)

H264_START = b'\x00\x00\x00\x01'


class FrontVideoTranscodeNode(Node):
    def __init__(self):
        super().__init__('front_video_transcode')

        self.declare_parameter('gs_ip', '192.168.123.100')
        self.declare_parameter('gs_port', 42074)
        self.declare_parameter('fec_percentage', 30)

        self.pipeline = None
        self.appsrc = None
        self.bus = None
        self.loop = None
        self.gst_thread = None
        self._nal_count = 0
        self._sps = None
        self._pps = None

        self._start_pipeline()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # raw=True: callback receives CDR bytes instead of a deserialized object.
        # This bypasses rmw_cyclonedds trying to read video360p/video180p fields
        # that the Go2 does not actually send, which was silently dropping IDR frames.
        self.create_subscription(
            Go2FrontVideoData,
            '/frontvideostream',
            self._video_callback,
            qos,
            raw=True,
        )
        self.get_logger().info('Subscribed to /frontvideostream (raw CDR mode)')

    def _start_pipeline(self):
        gs_ip = self.get_parameter('gs_ip').value
        gs_port = self.get_parameter('gs_port').value
        fec_percentage = self.get_parameter('fec_percentage').value

        pipeline_str = (
            f'appsrc name=src is-live=true do-timestamp=true block=false format=time '
            f'caps=video/x-h264,stream-format=byte-stream,alignment=nal ! '
            f'h264parse ! '
            f'rtph264pay config-interval=1 ! '
            f'rtpulpfecenc percentage={fec_percentage} ! '
            f'udpsink host={gs_ip} port={gs_port} sync=false'
        )
        self.get_logger().info(f'Launching pipeline:\n{pipeline_str}')

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name('src')

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self._on_gst_message)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Unable to set pipeline to PLAYING')

        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.gst_thread.start()
        self.get_logger().info('Pipeline running.')

    def _video_callback(self, raw_msg):
        # raw_msg is the raw CDR-serialized bytes of the DDS message.
        # Scan for the H264 Annex-B start code — everything from there onward
        # is one H264 NAL unit.
        data = bytes(raw_msg)
        start = data.find(H264_START)
        if start < 0:
            return
        data = data[start:]
        if len(data) < 5:
            return

        self._nal_count += 1
        if self._nal_count <= 10 or self._nal_count % 150 == 0:
            # Find all NAL types in this buffer
            pos = 0
            nal_types = []
            while True:
                idx = data.find(H264_START, pos)
                if idx < 0 or idx + 5 > len(data):
                    break
                nal_types.append(data[idx + 4] & 0x1F)
                pos = idx + 4
            self.get_logger().info(
                f'buf {self._nal_count}: {len(data)}B NAL types={nal_types}'
            )

        # Cache any SPS/PPS the Go2 emits (only at camera boot).
        # Re-inject them before every IDR if we ever get one.
        nals = self._split_nals(data)
        for nal in nals:
            nal_type = nal[4] & 0x1F
            if nal_type == 7:
                if self._sps != nal:
                    self._sps = nal
                    self.get_logger().info(f'Captured SPS ({len(nal)}B)')
            elif nal_type == 8:
                if self._pps != nal:
                    self._pps = nal
                    self.get_logger().info(f'Captured PPS ({len(nal)}B)')
            elif nal_type == 5 and self._sps and self._pps:
                # IDR arrived — prepend SPS+PPS so decoder (re)initialises cleanly
                data = self._sps + self._pps + data

        self._push(data)

    def _split_nals(self, data):
        nals = []
        pos = 0
        while True:
            idx = data.find(H264_START, pos)
            if idx < 0:
                break
            next_idx = data.find(H264_START, idx + 4)
            nals.append(data[idx: next_idx] if next_idx >= 0 else data[idx:])
            pos = idx + 4
        return nals

    def _push(self, data):
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        ret = self.appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warn(f'appsrc push-buffer: {ret}')

    def _on_gst_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f'GStreamer error: {err}')
            self.get_logger().error(f'Debug: {debug}')
            self.loop.quit()
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn('GStreamer EOS.')
            self.loop.quit()
        elif t == Gst.MessageType.WARNING:
            warn, _ = message.parse_warning()
            self.get_logger().warn(f'GStreamer warning: {warn}')
        return True

    def stop_pipeline(self):
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


def main(args=None):
    rclpy.init(args=args)
    node = FrontVideoTranscodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.stop_pipeline()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
