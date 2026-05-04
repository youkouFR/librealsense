// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include "ros2-bridge.h"
#include <rsutils/string/from.h>

namespace rs2
{
    ros2_bridge::ros2_bridge() = default;

    ros2_bridge::~ros2_bridge()
    {
        stop();
    }

    bool ros2_bridge::initialize(const std::string& rgb_topic, const std::string& pc_topic)
    {
#ifdef HAS_ROS2
        try
        {
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }

            _node = rclcpp::Node::make_shared("ros2_ai_viewer_node");

            _rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                _node, rgb_topic, rclcpp::QoS(rclcpp::KeepLast(5)).get_rmw_qos_profile());

            _pc_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                _node, pc_topic, rclcpp::QoS(rclcpp::KeepLast(5)).get_rmw_qos_profile());

            _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *_rgb_sub, *_pc_sub);
            _sync->registerCallback(std::bind(&ros2_bridge::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

            _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            _executor->add_node(_node);

            _connected = true;
            _error_message.clear();

            RCLCPP_INFO(_node->get_logger(), "ROS2 bridge initialized: RGB=%s, PC=%s",
                       rgb_topic.c_str(), pc_topic.c_str());
            return true;
        }
        catch (const std::exception& e)
        {
            _error_message = rsutils::string::from() << "Failed to initialize ROS2: " << e.what();
            _connected = false;
            return false;
        }
#else
        _error_message = "ROS2 support not compiled. Build with ROS2 installed.";
        return false;
#endif
    }

    void ros2_bridge::start()
    {
        if (!_connected || _running) return;

        _running = true;
        _spin_thread = std::thread(&ros2_bridge::spin_thread, this);
    }

    void ros2_bridge::stop()
    {
        if (!_running) return;

        _running = false;
        if (_spin_thread.joinable())
        {
            _spin_thread.join();
        }

#ifdef HAS_ROS2
        if (_executor)
        {
            _executor->cancel();
        }
#endif
    }

    bool ros2_bridge::poll_for_frame(synced_frame_data& data)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        if (_frame_queue.empty())
        {
            return false;
        }

        data = std::move(_frame_queue.front());
        _frame_queue.pop();
        return true;
    }

    void ros2_bridge::spin_thread()
    {
#ifdef HAS_ROS2
        while (_running && rclcpp::ok())
        {
            _executor->spin_some(std::chrono::milliseconds(10));
        }
#endif
    }

#ifdef HAS_ROS2
    void ros2_bridge::sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        try
        {
            synced_frame_data data;

            // Convert RGB image
            auto cv_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
            data.rgb_image = cv_ptr->image;
            data.has_rgb = true;

            // Convert PointCloud2 to rs2::points
            data.pointcloud = convert_pc2_to_rs2_points(pc_msg);
            data.has_pointcloud = true;

            // Timestamp
            data.timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;

            // Push to queue (drop oldest if full)
            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                if (_frame_queue.size() >= MAX_QUEUE_SIZE)
                {
                    _frame_queue.pop();
                }
                _frame_queue.push(std::move(data));
            }
            _data_cv.notify_one();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(_node->get_logger(), "Error in sync callback: %s", e.what());
        }
    }

    rs2::points ros2_bridge::convert_pc2_to_rs2_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        uint32_t width = pc_msg->width;
        uint32_t height = pc_msg->height;

        // Parse PointCloud2 fields to find x,y,z,r,g,b offsets
        int x_offset = -1, y_offset = -1, z_offset = -1;
        int r_offset = -1, g_offset = -1, b_offset = -1;

        for (const auto& field : pc_msg->fields)
        {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "r" || field.name == "red") r_offset = field.offset;
            else if (field.name == "g" || field.name == "green") g_offset = field.offset;
            else if (field.name == "b" || field.name == "blue") b_offset = field.offset;
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0)
        {
            RCLCPP_WARN(_node->get_logger(), "PointCloud2 missing x/y/z fields");
            return rs2::points();
        }

        // Note: Full pointcloud conversion to rs2::points requires creating a proper frame
        // For now, we store the raw data and handle rendering separately
        // A complete implementation would need to create a custom frame type
        return rs2::points();
    }
#endif
}
