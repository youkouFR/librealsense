// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

namespace rs2
{
    struct detection_instance
    {
        int class_id = -1;
        std::string class_name;
        float confidence = 0.0f;

        // 2D bounding box
        cv::Rect bbox;

        // Segmentation mask (binary, same size as input or cropped to bbox)
        cv::Mat mask;

        // Pose keypoints (x, y, confidence) - COCO format: 17 keypoints for human pose
        struct Keypoint {
            float x, y, confidence;
        };
        std::vector<Keypoint> keypoints;

        // 3D position (from pointcloud)
        cv::Vec3f position_3d = {0, 0, 0};
        bool has_3d_position = false;
    };

    struct detection_result
    {
        std::vector<detection_instance> instances;
        double inference_time_ms = 0.0;
        cv::Size input_size;
    };

    // COCO 80-class names
    static const std::vector<std::string> COCO_CLASS_NAMES = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };

    // COCO Pose keypoints connections (for skeleton drawing)
    static const std::vector<std::pair<int, int>> COCO_POSE_CONNECTIONS = {
        {0, 1}, {0, 2}, {1, 3}, {2, 4},  // Head
        {5, 6}, {5, 7}, {7, 9}, {6, 8}, {8, 10},  // Arms
        {5, 11}, {6, 12}, {11, 12},  // Torso
        {11, 13}, {12, 14}, {13, 15}, {14, 16}  // Legs
    };

    class ai_inference
    {
    public:
        ai_inference();
        ~ai_inference();

        // Load YOLOv8 model (detect, seg, or pose)
        bool load_model(const std::string& model_path, float conf_threshold = 0.5f);

        // Run inference on RGB image
        detection_result infer(const cv::Mat& rgb_image);

        // Run inference with depth for 3D position estimation
        detection_result infer_with_depth(const cv::Mat& rgb_image, const cv::Mat& depth_image);

        bool is_loaded() const { return _model_loaded; }
        std::string get_error_message() const { return _error_message; }

        // Model type detection
        enum model_type
        {
            DETECTION,
            SEGMENTATION,
            POSE,
            UNKNOWN
        };
        model_type get_model_type() const { return _model_type; }

        void set_confidence_threshold(float threshold) { _conf_threshold = threshold; }
        void set_nms_threshold(float threshold) { _nms_threshold = threshold; }

    private:
        detection_result run_yolov8_detect(const cv::Mat& image);
        detection_result run_yolov8_seg(const cv::Mat& image);
        detection_result run_yolov8_pose(const cv::Mat& image);

        void estimate_3d_positions(detection_result& result, const cv::Mat& depth_image);

#ifdef HAS_ONNXRUNTIME
        struct onnx_impl;
        std::unique_ptr<onnx_impl> _impl;
#endif

        bool _model_loaded = false;
        model_type _model_type = UNKNOWN;
        float _conf_threshold = 0.5f;
        float _nms_threshold = 0.45f;
        std::string _error_message;
        std::string _model_path;

        // YOLOv8 parameters
        static constexpr int INPUT_WIDTH = 640;
        static constexpr int INPUT_HEIGHT = 640;
    };
}
