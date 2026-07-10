#!/usr/bin/env python3
"""
Standalone base station node for ZED-F9P over a single USB connection.

Opens /dev/tty_rtkgps, configures survey-in and RTCM3 output on the USB port
via raw UBX messages, then reads the mixed UBX+RTCM byte stream and publishes
RTCM3 correction frames to /rtcm.

Survey-in progress is logged to the ROS console.  No NavSatFix or other GPS
data is published — the base station's only job is producing RTK corrections.
"""

import struct
import threading

import rclpy
from rclpy.node import Node
import serial
from rtcm_msgs.msg import Message


# ── UBX frame builder ─────────────────────────────────────────────────────────

def _ubx(cls: int, mid: int, payload: bytes = b'') -> bytes:
    msg = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for b in msg:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + msg + bytes([ck_a, ck_b])


def _cfg_tmode3_svin(min_dur: int, acc_lim_m: float) -> bytes:
    """UBX-CFG-TMODE3: survey-in mode (40-byte payload)."""
    payload = struct.pack('<HHiiibbbbIII',
        0,                       # version
        0x0001,                  # flags: mode = SURVEY_IN
        0, 0, 0,                 # ecefX/Y/Z (not used in survey-in)
        0, 0, 0, 0,              # ecefXHP/YHP/ZHP + reserved1
        0,                       # fixedPosAcc (not used)
        min_dur,                 # svinMinDur [s]
        int(acc_lim_m * 10000), # svinAccLimit [0.1 mm]
    ) + b'\x00' * 8             # reserved2
    return _ubx(0x06, 0x71, payload)


def _cfg_prt_usb_rtcm() -> bytes:
    """UBX-CFG-PRT port 3 (USB): enable RTCM3 output alongside UBX (20-byte payload)."""
    payload = struct.pack('<BBHIIHHHxx',
        3,      # portID = USB
        0,      # reserved0
        0,      # txReady (disabled)
        0,      # mode (not applicable for USB)
        0,      # baudRate (not applicable for USB)
        0x0007, # inProtoMask:  UBX | NMEA | RTCM3
        0x0021, # outProtoMask: UBX | RTCM3
        0,      # flags
    )
    return _ubx(0x06, 0x00, payload)


def _cfg_msg_usb(cls: int, mid: int, rate: int) -> bytes:
    """UBX-CFG-MSG (8-byte): set output rate on USB port (index 3)."""
    # rate order: DDC, UART1, UART2, USB, SPI, reserved
    return _ubx(0x06, 0x01, bytes([cls, mid, 0, 0, 0, rate, 0, 0]))


# RTCM3 messages to output (class 0xF5):
#   0x05 = 1005  Stationary RTK reference station ARP
#   0x4D = 1077  GPS MSM7
#   0x57 = 1087  GLONASS MSM7
#   0xE6 = 1230  GLONASS code-phase biases
_RTCM_IDS = [0x05, 0x4D, 0x57, 0xE6]


# ── Node ──────────────────────────────────────────────────────────────────────

class GpsBaseNode(Node):
    def __init__(self):
        super().__init__('gps_base_node')
        self.declare_parameter('port',         '/dev/tty_rtkgps')
        self.declare_parameter('baudrate',     115200)
        self.declare_parameter('svin_min_dur', 180)
        self.declare_parameter('svin_acc_lim', 2.0)

        port    = self.get_parameter('port').get_parameter_value().string_value
        baud    = self.get_parameter('baudrate').get_parameter_value().integer_value
        min_dur = self.get_parameter('svin_min_dur').get_parameter_value().integer_value
        acc_lim = self.get_parameter('svin_acc_lim').get_parameter_value().double_value

        self._pub = self.create_publisher(Message, '/rtcm', 10)
        self._serial = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Opened {port} at {baud} baud')

        self._send_config(min_dur, acc_lim)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _send_config(self, min_dur: int, acc_lim: float) -> None:
        cmds = [
            _cfg_tmode3_svin(min_dur, acc_lim),
            _cfg_prt_usb_rtcm(),
        ]
        for rtcm_id in _RTCM_IDS:
            cmds.append(_cfg_msg_usb(0xF5, rtcm_id, 1))
        cmds.append(_cfg_msg_usb(0x01, 0x3B, 1))  # NAV-SVIN at 1 Hz for status logging

        for cmd in cmds:
            self._serial.write(cmd)

        self.get_logger().info(
            f'Config sent: survey-in min={min_dur}s acc<={acc_lim}m, RTCM on USB'
        )

    def _read_loop(self) -> None:
        buf = bytearray()
        while rclpy.ok():
            try:
                chunk = self._serial.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break
            if chunk:
                buf.extend(chunk)
                buf = self._process(buf)

    def _process(self, buf: bytearray) -> bytearray:
        i = 0
        while i < len(buf):
            b = buf[i]

            # RTCM3 frame: 0xD3 + 2-byte length + body + 3-byte CRC
            if b == 0xD3:
                if i + 3 > len(buf):
                    break
                length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
                end = i + 3 + length + 3
                if end > len(buf):
                    break
                msg = Message()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.message = bytes(buf[i:end])
                self._pub.publish(msg)
                i = end
                continue

            # UBX frame: 0xB5 0x62 + class + id + 2-byte length + payload + 2-byte checksum
            if b == 0xB5 and i + 1 < len(buf) and buf[i + 1] == 0x62:
                if i + 6 > len(buf):
                    break
                pay_len = struct.unpack_from('<H', buf, i + 4)[0]
                end = i + 6 + pay_len + 2
                if end > len(buf):
                    break
                self._handle_ubx(buf[i + 2], buf[i + 3],
                                  bytes(buf[i + 6: i + 6 + pay_len]))
                i = end
                continue

            i += 1

        return buf[i:]

    def _handle_ubx(self, cls: int, mid: int, payload: bytes) -> None:
        # NAV-SVIN (0x01 0x3B): log survey-in progress
        # Layout: version(1) reserved(3) iTOW(4) dur(4) meanX/Y/Z(12)
        #         meanXHP/YHP/ZHP(3) reserved(1) meanAcc(4) obs(4) valid(1) active(1) reserved(2)
        if cls == 0x01 and mid == 0x3B and len(payload) >= 40:
            dur      = struct.unpack_from('<I', payload, 8)[0]
            mean_acc = struct.unpack_from('<I', payload, 28)[0] * 0.0001  # → metres
            obs      = struct.unpack_from('<I', payload, 32)[0]
            valid    = bool(payload[36])
            active   = bool(payload[37])
            if active:
                self.get_logger().info(
                    f'Survey-in: {dur}s elapsed, acc={mean_acc:.3f}m, obs={obs}'
                )
            elif valid:
                self.get_logger().info('Survey-in complete — RTCM corrections active')

    def destroy_node(self) -> None:
        self._serial.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GpsBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
