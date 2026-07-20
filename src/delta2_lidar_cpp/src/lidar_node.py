#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time


class Delta2LidarDriver(Node):
    def __init__(self):
        super().__init__('delta2_lidar_driver_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.001
        )

        self.get_logger().info(f'Đã mở serial {self.port} @ {self.baudrate}')

        self.buffer = bytearray()

        self.N = 720
        self.ranges = [float('inf')] * self.N
        self.prev_ranges = [float('inf')] * self.N
        self.updated = set()

        self.current_rpm = 414.0
        self.last_publish_time = time.time()

        self.alpha = 0.35
        self.range_min = 0.15
        self.range_max = 8.0

        self.create_timer(0.003, self.read_serial)

    def be16(self, data):
        return (data[0] << 8) | data[1]

    def sbe16(self, data):
        v = self.be16(data)
        return v - 65536 if v & 0x8000 else v

    def checksum16(self, data):
        return sum(data) & 0xFFFF

    def read_serial(self):
        n = self.ser.in_waiting
        if n > 0:
            self.buffer.extend(self.ser.read(n))

        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-512:]

        while True:
            idx = self.buffer.find(b'\xAA')
            if idx < 0:
                self.buffer.clear()
                return

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < 3:
                return

            frame_len = self.be16(self.buffer[1:3])
            total_len = frame_len + 2

            if total_len < 20 or total_len > 128:
                del self.buffer[0]
                continue

            if len(self.buffer) < total_len:
                return

            frame = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            self.parse_frame(frame)

    def parse_frame(self, frame):
        if len(frame) < 12:
            return

        if frame[0] != 0xAA:
            return

        if frame[3] != 0x01 or frame[4] != 0x61 or frame[5] != 0xAD:
            return

        crc_recv = self.be16(frame[-2:])
        crc_calc = self.checksum16(frame[:-2])
        if crc_recv != crc_calc:
            return

        payload_len = self.be16(frame[6:8])
        payload = frame[8:8 + payload_len]

        if len(payload) < 5:
            return

        rpm_raw = payload[0]
        self.current_rpm = max(60.0, rpm_raw * 3.0)

        offset_angle = self.sbe16(payload[1:3]) * 0.01
        start_angle = self.be16(payload[3:5]) * 0.01

        sample_count = (payload_len - 5) // 3
        if sample_count <= 0:
            return

        sector_angle = 22.5
        angle_step = sector_angle / sample_count

        for i in range(sample_count):
            base = 5 + i * 3
            if base + 2 >= len(payload):
                break

            quality = payload[base]
            raw_distance = self.be16(payload[base + 1:base + 3])

            if raw_distance == 0 or quality == 0:
                continue

            distance_m = (raw_distance * 0.25) / 1000.0

            if not (self.range_min <= distance_m <= self.range_max):
                continue

            angle_deg = (start_angle + i * angle_step + offset_angle) % 360.0
            idx = int(angle_deg * 2.0) % self.N

            old = self.prev_ranges[idx]
            if math.isinf(old):
                filtered = distance_m
            else:
                filtered = self.alpha * distance_m + (1.0 - self.alpha) * old

            self.ranges[idx] = filtered
            self.prev_ranges[idx] = filtered
            self.updated.add(idx)

        now = time.time()

        # Publish khi đã có gần đủ vòng, hoặc quá 0.15s thì vẫn publish
        if len(self.updated) >= 500 or (now - self.last_publish_time) >= 0.15:
            self.publish_scan()
            self.updated.clear()
            self.last_publish_time = now

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / self.N

        msg.scan_time = 60.0 / self.current_rpm
        msg.time_increment = msg.scan_time / self.N

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        reordered = self.ranges[self.N // 2:] + self.ranges[:self.N // 2]
        msg.ranges = reordered

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Delta2LidarDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()