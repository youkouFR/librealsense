#!/usr/bin/env python3
"""
Test script: publishes synthetic RGB image + small diagnostic point cloud.
- RGB: simple 320x240 checkerboard pattern
- PointCloud2: ~20 points at known positions with distinct colors
               (allows easy debugging of 3D viewer rendering)

Same topic names as the real bridge so ros2-ai-viewer picks them up.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


class TestPublisher(Node):
    def __init__(self):
        super().__init__("test_publisher")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("pc_topic", "/camera/depth/color/points")
        self.declare_parameter("fps", 2)

        rgb_topic = self.get_parameter("rgb_topic").value
        pc_topic = self.get_parameter("pc_topic").value
        fps = self.get_parameter("fps").value

        self._rgb_pub = self.create_publisher(Image, rgb_topic, 10)
        self._pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)
        self._timer = self.create_timer(1.0 / fps, self._publish)

        self.get_logger().info(f"Test publisher ready: RGB→{rgb_topic}, PC→{pc_topic}")

    def _make_checkerboard(self, w=320, h=240, cell=40):
        """Simple checkerboard RGB image."""
        img = np.zeros((h, w, 3), dtype=np.uint8)
        for y in range(h):
            for x in range(w):
                if ((x // cell) + (y // cell)) % 2 == 0:
                    img[y, x] = [80, 80, 80]  # dark grey
                else:
                    img[y, x] = [200, 200, 200]  # light grey
        return img

    def _pack_rgb_float(self, r: int, g: int, b: int) -> float:
        """Pack 3 uint8 channels into a single float32 (ROS2 PointCloud2 convention)."""
        packed = (r << 16) | (g << 8) | b
        return struct.unpack("f", struct.pack("I", packed))[0]

    def _publish(self):
        # --- RGB Image ---
        img = self._make_checkerboard()
        img_msg = Image()
        img_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_frame")
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = img.shape[1] * 3
        img_msg.data = img.tobytes()
        self._rgb_pub.publish(img_msg)

        # --- PointCloud2 (synthetic, small, known coords) ---
        # RealSense uses: +X right, +Y down, +Z forward
        # After conversion to OpenGL:  X→X,  Y→-Y,  Z→-Z
        # So OpenGL coords ~= (X, -Y, -Z)
        # We design the cloud so that in OpenGL it spans e.g. (-1..1, -1..1, -3..-1)

        points = []

        # 1. Origin marker — red (should appear at center of bounding box)
        points.append((0.0, 0.0, 1.5, 255, 0, 0))   # will be GL: (0, 0, -1.5)

        # 2. Axes: +X direction — green
        for x in np.linspace(0.1, 1.0, 5):
            points.append((x, 0.0, 1.5, 0, 255, 0))

        # 3. Axes: +Y direction (down in RS, up in GL) — blue
        for y in np.linspace(0.1, 1.0, 5):
            points.append((0.0, y, 1.5, 0, 0, 255))

        # 4. Axes: +Z direction (forward) — white
        for z in np.linspace(0.5, 3.0, 6):
            points.append((0.0, 0.0, z, 255, 255, 255))

        # 5. Corner points — yellow
        points.append((-1.0, -1.0, 1.5, 255, 255, 0))
        points.append((1.0, -1.0, 1.5, 255, 255, 0))
        points.append((-1.0, 1.0, 1.5, 255, 255, 0))
        points.append((1.0, 1.0, 1.5, 255, 255, 0))

        n = len(points)
        dtype = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)])
        arr = np.zeros(n, dtype=dtype)
        for i, (x, y, z, r, g, b) in enumerate(points):
            arr[i]["x"] = x
            arr[i]["y"] = y
            arr[i]["z"] = z
            arr[i]["rgb"] = self._pack_rgb_float(r, g, b)

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = PointCloud2()
        pc_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_frame")
        pc_msg.height = 1
        pc_msg.width = n
        pc_msg.is_bigendian = False
        pc_msg.point_step = dtype.itemsize
        pc_msg.row_step = dtype.itemsize * n
        pc_msg.is_dense = True
        pc_msg.data = arr.tobytes()
        pc_msg.fields = fields
        self._pc_pub.publish(pc_msg)

        self.get_logger().info(f"Published test frame | PC: {n} points (origin at RS z=1.5)")


def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
