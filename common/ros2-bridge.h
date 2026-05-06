// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <memory>
#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <condition_variable>

#ifdef HAS_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

namespace rs2
{
    struct point_cloud_data
    {
        std::vector<float> vertices; // x, y, z
        std::vector<float> colors;   // r, g, b (0.0-1.0)
        size_t count = 0;
    };

    struct synced_frame_data
    {
        cv::Mat rgb_image;
        rs2::points pointcloud;
        point_cloud_data pc_data;
        double timestamp = 0.0;
        bool has_rgb = false;
        bool has_pointcloud = false;
    };

    class ros2_bridge
    {
    public:
        ros2_bridge();
        ~ros2_bridge();

        bool initialize(const std::string& rgb_topic, const std::string& pc_topic);
        void start();
        void stop();

        bool poll_for_frame(synced_frame_data& data);

        bool is_connected() const { return _connected; }
        std::string get_error_message() const { return _error_message; }

    private:
        void spin_thread();

#ifdef HAS_ROS2
        void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg);

        std::shared_ptr<rclcpp::Node> _node;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> _rgb_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> _pc_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _rgb_sub_single;

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
#endif

        std::thread _spin_thread;
        std::atomic<bool> _running{false};
        std::atomic<bool> _connected{false};

        std::mutex _data_mutex;
        std::queue<synced_frame_data> _frame_queue;
        std::condition_variable _data_cv;

        std::string _error_message;

        static constexpr size_t MAX_QUEUE_SIZE = 5;

#ifdef HAS_ROS2
        rs2::points convert_pc2_to_rs2_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg);
        void parse_pc2_to_data(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg, point_cloud_data& out_data);
        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);
#endif
    };
}
