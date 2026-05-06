#!/usr/bin/env python3
"""
RealSense ROS2 Bridge: Publishes RGB and PointCloud2 topics from a physical RealSense camera.
- RGB topic: /camera/color/image_raw (sensor_msgs/msg/Image)
- PointCloud topic: /camera/depth/color/points (sensor_msgs/msg/PointCloud2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import cv2


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__("realsense_publisher")

        # Declare parameters
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("pc_topic", "/camera/depth/color/points")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        rgb_topic = self.get_parameter("rgb_topic").value
        pc_topic = self.get_parameter("pc_topic").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value

        self._rgb_pub = self.create_publisher(Image, rgb_topic, 10)
        self._pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)

        self.get_logger().info(f"Publishing RGB to: {rgb_topic}")
        self.get_logger().info(f"Publishing PointCloud2 to: {pc_topic}")

        # Initialize RealSense pipeline
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        profile = self._pipeline.start(config)

        # Get depth scale for converting depth pixels to meters
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f"Depth scale: {self._depth_scale}")

        # Create alignment and pointcloud processing blocks
        self._align = rs.align(rs.stream.color)
        self._pc = rs.pointcloud()

        # Camera intrinsics (will be updated per frame)
        self._intrinsics = None

        # Timer for publishing
        self._timer = self.create_timer(1.0 / fps, self._publish_frame)

        self.get_logger().info("RealSense pipeline started.")

    def _publish_frame(self):
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError as e:
            self.get_logger().warn(f"Failed to get frames: {e}")
            return

        # Align depth to color
        aligned_frames = self._align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Get color intrinsics
        if self._intrinsics is None:
            self._intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()

        # --- Publish RGB Image ---
        color_image = np.asanyarray(color_frame.get_data())
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_color_optical_frame"
        img_msg.height = rgb_image.shape[0]
        img_msg.width = rgb_image.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = rgb_image.shape[1] * 3
        img_msg.data = rgb_image.tobytes()

        self._rgb_pub.publish(img_msg)

        # --- Publish PointCloud2 ---
        # Map depth to 3D points
        self._pc.map_to(color_frame)
        points = self._pc.calculate(depth_frame)
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        # Filter out invalid points (z == 0)
        valid_mask = vertices[:, 2] > 0
        valid_points = vertices[valid_mask]

        if len(valid_points) > 0:
            # Create RGB colors for each valid point
            # Map 3D point back to 2D pixel coordinates
            uvs = np.zeros((len(valid_points), 2), dtype=np.int32)
            for i, pt in enumerate(valid_points):
                px, py = rs.rs2_project_point_to_pixel(self._intrinsics, pt.tolist())
                uvs[i, 0] = int(np.clip(px, 0, rgb_image.shape[1] - 1))
                uvs[i, 1] = int(np.clip(py, 0, rgb_image.shape[0] - 1))

            # Get RGB colors for each point
            colors = rgb_image[uvs[:, 1], uvs[:, 0]].astype(np.float32) / 255.0

            # Create structured array for PointCloud2
            # Fields: x, y, z, rgb
            dtype = np.dtype([
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rgb", np.float32),
            ])
            cloud_array = np.zeros(len(valid_points), dtype=dtype)
            cloud_array["x"] = valid_points[:, 0]
            cloud_array["y"] = valid_points[:, 1]
            cloud_array["z"] = valid_points[:, 2]

            # Pack RGB into a single float (ROS2 convention: rgb as float32)
            colors_uint8 = (colors * 255).astype(np.uint8)
            cloud_array["rgb"] = (
                (colors_uint8[:, 0].astype(np.uint32) << 16)
                | (colors_uint8[:, 1].astype(np.uint32) << 8)
                | (colors_uint8[:, 2].astype(np.uint32))
            ).view(np.float32)

            # Create PointCloud2 message
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            pc_msg = PointCloud2()
            pc_msg.header = Header()
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            pc_msg.header.frame_id = "camera_color_optical_frame"
            pc_msg.height = 1
            pc_msg.width = len(valid_points)
            pc_msg.is_bigendian = False
            pc_msg.point_step = dtype.itemsize
            pc_msg.row_step = dtype.itemsize * len(valid_points)
            pc_msg.is_dense = True
            pc_msg.data = cloud_array.tobytes()
            pc_msg.fields = fields

            self._pc_pub.publish(pc_msg)

    def destroy_node(self):
        self._pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
