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

            // Use sensor_data QoS (best_effort) to match typical camera publishers
            auto qos = rclcpp::SensorDataQoS();
            qos.keep_last(10);

            _rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                _node, rgb_topic, qos.get_rmw_qos_profile());

            _pc_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                _node, pc_topic, qos.get_rmw_qos_profile());

            // ApproximateTime sync with larger queue and relaxed time tolerance
            _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), *_rgb_sub, *_pc_sub);
            _sync->registerCallback(std::bind(&ros2_bridge::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
            
            // Also subscribe to RGB alone as fallback
            _rgb_sub_single = _node->create_subscription<sensor_msgs::msg::Image>(
                rgb_topic, qos, std::bind(&ros2_bridge::rgb_callback, this, std::placeholders::_1));

            _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            _executor->add_node(_node);

            _connected = true;
            _error_message.clear();

            RCLCPP_INFO(_node->get_logger(), "ROS2 bridge initialized: RGB=%s, PC=%s",
                       rgb_topic.c_str(), pc_topic.c_str());
            RCLCPP_INFO(_node->get_logger(), "QoS: best_effort reliability");
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
            RCLCPP_INFO(_node->get_logger(), "Sync callback triggered! RGB: %dx%d, PC: %d points",
                       rgb_msg->width, rgb_msg->height, pc_msg->width);
            synced_frame_data data;

            // Convert RGB image
            auto cv_ptr = cv_bridge::toCvCopy(rgb_msg, "bgr8");
            data.rgb_image = cv_ptr->image;
            data.has_rgb = true;

            RCLCPP_INFO(_node->get_logger(), "Image converted: %dx%d, channels=%d",
                       data.rgb_image.cols, data.rgb_image.rows, data.rgb_image.channels());

            // Convert PointCloud2 to rs2::points
            data.pointcloud = convert_pc2_to_rs2_points(pc_msg);
            parse_pc2_to_data(pc_msg, data.pc_data);
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
            RCLCPP_INFO(_node->get_logger(), "Frame queued, queue size: %zu", _frame_queue.size());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(_node->get_logger(), "Error in sync callback: %s", e.what());
        }
    }

    void ros2_bridge::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            RCLCPP_INFO(_node->get_logger(), "RGB-only callback triggered! Image: %dx%d",
                       msg->width, msg->height);
            synced_frame_data data;

            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            data.rgb_image = cv_ptr->image;
            data.has_rgb = true;
            data.has_pointcloud = false;
            data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                if (_frame_queue.size() >= MAX_QUEUE_SIZE)
                {
                    _frame_queue.pop();
                }
                _frame_queue.push(std::move(data));
            }
            _data_cv.notify_one();
            RCLCPP_INFO(_node->get_logger(), "RGB frame queued, queue size: %zu", _frame_queue.size());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(_node->get_logger(), "Error in RGB callback: %s", e.what());
        }
    }

    rs2::points ros2_bridge::convert_pc2_to_rs2_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        uint32_t width = pc_msg->width;
        uint32_t height = pc_msg->height;

        // Parse PointCloud2 fields to find x,y,z,r,g,b offsets
        int x_offset = -1, y_offset = -1, z_offset = -1;
        int r_offset = -1, g_offset = -1, b_offset = -1;
        bool has_rgb = false;

        for (const auto& field : pc_msg->fields)
        {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "rgb") r_offset = field.offset; // packed rgb
            else if (field.name == "r" || field.name == "red") r_offset = field.offset;
            else if (field.name == "g" || field.name == "green") g_offset = field.offset;
            else if (field.name == "b" || field.name == "blue") b_offset = field.offset;
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0)
        {
            RCLCPP_WARN(_node->get_logger(), "PointCloud2 missing x/y/z fields");
            return rs2::points();
        }

        if (r_offset >= 0) has_rgb = true;

        const uint8_t* data = pc_msg->data.data();
        uint32_t point_step = pc_msg->point_step;
        size_t num_points = pc_msg->width * pc_msg->height;

        // Store in synced_frame_data's pc_data
        // Note: We can't easily populate rs2::points without a custom frame, 
        // so we populate the raw data structure instead.
        // This function is kept for compatibility but the actual data is stored in pc_data.
        
        // For now, return empty rs2::points as we use raw data for rendering
        return rs2::points();
    }

    void ros2_bridge::parse_pc2_to_data(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg, point_cloud_data& out_data)
    {
        int x_offset = -1, y_offset = -1, z_offset = -1;
        int r_offset = -1, g_offset = -1, b_offset = -1;
        bool has_packed_rgb = false;

        for (const auto& field : pc_msg->fields)
        {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "rgb") { r_offset = field.offset; has_packed_rgb = true; }
            else if (field.name == "r" || field.name == "red") r_offset = field.offset;
            else if (field.name == "g" || field.name == "green") g_offset = field.offset;
            else if (field.name == "b" || field.name == "blue") b_offset = field.offset;
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0) return;

        const uint8_t* data = pc_msg->data.data();
        uint32_t point_step = pc_msg->point_step;
        size_t num_points = pc_msg->width * pc_msg->height;

        out_data.vertices.reserve(num_points * 3);
        if (r_offset >= 0) out_data.colors.reserve(num_points * 3);

        for (size_t i = 0; i < num_points; ++i)
        {
            const uint8_t* ptr = data + i * point_step;
            float x = *reinterpret_cast<const float*>(ptr + x_offset);
            float y = *reinterpret_cast<const float*>(ptr + y_offset);
            float z = *reinterpret_cast<const float*>(ptr + z_offset);

            // Filter invalid points
            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || z <= 0.0f) continue;

            // Coordinate conversion:
            // RealSense: +X Right, +Y Down, +Z Forward (Left-Handed)
            // OpenGL:    +X Right, +Y Up,   -Z Forward (Right-Handed)
            // Transform: X -> X, Y -> -Y, Z -> -Z
            out_data.vertices.push_back(x);
            out_data.vertices.push_back(-y);
            out_data.vertices.push_back(-z);

            if (r_offset >= 0)
            {
                if (has_packed_rgb)
                {
                    // Packed RGB float
                    uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(ptr + r_offset);
                    float r = ((rgb_val >> 16) & 0xFF) / 255.0f;
                    float g = ((rgb_val >> 8) & 0xFF) / 255.0f;
                    float b = (rgb_val & 0xFF) / 255.0f;
                    out_data.colors.push_back(r);
                    out_data.colors.push_back(g);
                    out_data.colors.push_back(b);
                }
                else if (g_offset >= 0 && b_offset >= 0)
                {
                    // Separate channels
                    float r = *reinterpret_cast<const uint8_t*>(ptr + r_offset) / 255.0f;
                    float g = *reinterpret_cast<const uint8_t*>(ptr + g_offset) / 255.0f;
                    float b = *reinterpret_cast<const uint8_t*>(ptr + b_offset) / 255.0f;
                    out_data.colors.push_back(r);
                    out_data.colors.push_back(g);
                    out_data.colors.push_back(b);
                }
            }
        }
        out_data.count = out_data.vertices.size() / 3;
    }
#endif
}
