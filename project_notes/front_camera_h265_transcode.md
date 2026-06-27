# Front Camera Video Pipeline

## Goal

Receive the Go2's front camera H264 stream from the `frontvideofeed` DDS topic
and forward it as RTP H264 over UDP to the groundstation.

**Note:** H265 transcoding was the original goal but the Jetson has no hardware video
encoder available (`nvv4l2*` GStreamer plugins not installed — only `nvjpegenc` is present).
Software H265 encode (x265) would hammer the CPU. H264 passthrough is zero overhead.

## What We Know About `frontvideofeed`

- Published on the Go2's bare CycloneDDS network (192.168.123.x, interface `enP8p1s0`)
- Publisher: bare DDS app (Go2 firmware, `_CREATED_BY_BARE_DDS_APP_`)
- Message type: **`unitree_go/msg/Go2FrontVideoData`** (confirmed)
  ```
  uint64 time_frame
  uint8[] video720p   ← 1280×720 H264, use this
  uint8[] video360p   ← 640×360 H264
  uint8[] video180p   ← 320×180 H264
  ```
- QoS: RELIABLE, KEEP_LAST depth=1, VOLATILE, INFINITE lifespan/deadline
- Each message contains one complete H264 access unit (frame) at all three resolutions
- Subscribe using `alignment=au` in GStreamer appsrc caps (one AU per push)

## Recommended Architecture

```
Go2 firmware (CycloneDDS, 192.168.123.x)
  → /frontvideofeed topic (H264 NAL units per message)
  → [Jetson Orin] front_video_transcode.py
      ROS2 subscriber (rmw_cyclonedds_cpp) extracts H264 bytes
      → GStreamer appsrc (video/x-h264)
      → h264parse                      ← normalize Annex-B/AVCC, align AUs
      → nvv4l2decoder                  ← Jetson NVDEC hardware H264 decode
      → nvvidconv                      ← stays in NVMM, zero-copy
      → video/x-raw(memory:NVMM),NV12
      → nvv4l2h265enc                  ← Jetson NVENC hardware H265 encode
      → h265parse → rtph265pay
      → rtpulpfecenc percentage=30     ← FEC for WiFi packet loss
      → udpsink host=<gs_ip> port=42074
  → [Groundstation] receive with gst-launch-1.0 (see below)
```

## Why This Approach

**Hardware codec path (nvv4l2decoder → nvv4l2h265enc):**
- Runs entirely on Jetson's dedicated video hardware (NVDEC + NVENC), no CPU load
- Zero-copy between decode and encode via NVMM shared memory
- CPU-based alternatives (avdec_h264 → x265enc) would saturate the Orin Nano

**Direct UDP RTP for video:**
- Existing camera pipelines (camera_capture.py, camera_ros2_conversion.py) all use RTP/UDP
- GStreamer's rtpulpfecenc adds forward error correction for WiFi resilience

**H265 over H264 for transport:**
- ~40% bitrate reduction at same quality — meaningful for a bandwidth-constrained link
- Jetson hardware encodes H265 at essentially the same power cost as H264

## GStreamer Pipeline String

```python
pipeline_str = (
    f'appsrc name=src is-live=true block=false format=time '
    f'caps=video/x-h264,stream-format=byte-stream,alignment=au,'
    f'width=1280,height=720,framerate=30/1 ! '
    f'h264parse ! '
    f'nvv4l2decoder ! '
    f'nvvidconv ! '
    f'video/x-raw(memory:NVMM),format=NV12 ! '
    f'nvv4l2h265enc preset-level=1 bitrate={bitrate} ! '
    f'h265parse ! '
    f'rtph265pay config-interval=1 ! '
    f'rtpulpfecenc percentage=30 ! '
    f'udpsink host={gs_ip} port=42074 sync=false'
)
```

Tuning notes:
- `bitrate=2000000` (2 Mbps) is a good starting point for 720p H265
- `preset-level=1` is low-latency on Jetson; `preset-level=4` is max quality
- If the Go2 sends AVCC-format H264 (no 0x000001 start codes), change caps to
  `stream-format=avc,alignment=au` — h264parse will still handle it

## Node Structure

New file: `src/go2_camera/go2_camera/front_video_transcode.py`

Pattern closely follows `camera_ros2_conversion.py` but:
- Subscribes to `/frontvideofeed` (`unitree_go/msg/Go2FrontVideoData`, RELIABLE, depth=1)
- Extracts `bytes(msg.video720p)` — no BGR conversion, no cv_bridge needed
- Caps are `video/x-h264,stream-format=byte-stream,alignment=au,width=1280,height=720`
- QoS must match Go2: `ReliabilityPolicy.RELIABLE`, `DurabilityPolicy.VOLATILE`, depth=1

Key callback (replaces `image_callback`):
```python
def video_callback(self, msg):
    if self.appsrc is None:
        return
    data = bytes(msg.video720p)
    if not data:
        return
    buf = Gst.Buffer.new_allocate(None, len(data), None)
    buf.fill(0, data)
    buf.pts = self.pts
    buf.duration = self.frame_duration
    self.pts += self.frame_duration
    self.appsrc.emit('push-buffer', buf)
```

Run under `rmw_cyclonedds_cpp` with `CYCLONEDDS_URI` pointing to the cyclonedds.xml
that binds to `enP8p1s0`. All systemd services are disabled — run directly, not as a service.

## Port Assignments (existing convention)

| Port  | Stream                                |
|-------|---------------------------------------|
| 42067 | Infrared camera H265 (camera_capture) |
| 42073 | ROS2 image → H265 (ros2_conversion)  |
| 42074 | Front camera H265 (new — this node)  |

## Groundstation Receiver

```bash
# CPU decode (works on any machine with GStreamer + libav):
gst-launch-1.0 udpsrc port=42074 \
  caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96" \
  ! rtpulpfecdec pt=122 \
  ! rtph265depay \
  ! h265parse \
  ! avdec_h265 \
  ! videoconvert \
  ! autovideosink

# Or display with VLC (easier for testing):
# Point VLC at a .sdp file with:
#   m=video 42074 RTP/AVP 96
#   a=rtpmap:96 H265/90000
```

## Open Questions Before Implementing

1. **No relay needed** — the transcode node runs under `rmw_cyclonedds_cpp` directly on
   `enP8p1s0`, same as `go2_relay.py`. It subscribes to `frontvideofeed` independently.
   No need to add it to `go2_relay.py` (that relay is for Zenoh bridge, not this pipeline).
2. **Frame boundary** — confirmed complete AU per message (three resolutions per message
   implies the Go2 encodes all three before sending). Use `alignment=au`.
3. **Groundstation IP** — make a ROS2 parameter, default to the known groundstation IP.
4. **Network path** — confirm groundstation can receive UDP from Jetson on port 42074
   (no firewall/NAT blocking). Transport is direct UDP only.

## Implementation Steps

1. Confirm message type (step above)
2. Create `front_video_transcode.py` node based on `camera_ros2_conversion.py` pattern
3. Register in `setup.py` entry_points
4. Test pipeline with `gst-launch-1.0` first (inject a test H264 file via udpsrc to
   verify the decode/encode/send chain before adding the ROS2 subscription)
5. Add systemd service under `orin_systemd/` mirroring `go2-relay.service` pattern
6. Add groundstation receive command to `groundstation/README.md`
