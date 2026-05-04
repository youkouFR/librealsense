// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include "ai-inference.h"
#include <rsutils/string/from.h>
#include <chrono>
#include <algorithm>
#include <numeric>

namespace rs2
{
#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>

    struct ai_inference::onnx_impl
    {
        Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolov8"};
        Ort::SessionOptions session_options;
        std::unique_ptr<Ort::Session> session;
        Ort::AllocatorWithDefaultOptions allocator;

        std::vector<const char*> input_names;
        std::vector<const char*> output_names;
        std::vector<std::vector<int64_t>> output_shapes;

        onnx_impl()
        {
            session_options.SetIntraOpNumThreads(4);
            session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        }
    };
#endif

    ai_inference::ai_inference() = default;

    ai_inference::~ai_inference() = default;

    bool ai_inference::load_model(const std::string& model_path, float conf_threshold)
    {
        _model_path = model_path;
        _conf_threshold = conf_threshold;

#ifdef HAS_ONNXRUNTIME
        try
        {
            _impl = std::make_unique<onnx_impl>();

            // Detect model type from filename
            if (model_path.find("seg") != std::string::npos)
            {
                _model_type = SEGMENTATION;
            }
            else if (model_path.find("pose") != std::string::npos)
            {
                _model_type = POSE;
            }
            else
            {
                _model_type = DETECTION;
            }

#ifdef _WIN32
            std::wstring wpath(model_path.begin(), model_path.end());
            _impl->session = std::make_unique<Ort::Session>(_impl->env, wpath.c_str(), _impl->session_options);
#else
            _impl->session = std::make_unique<Ort::Session>(_impl->env, model_path.c_str(), _impl->session_options);
#endif

            // Get input/output info
            size_t num_input_nodes = _impl->session->GetInputCount();
            size_t num_output_nodes = _impl->session->GetOutputCount();

            _impl->input_names.resize(num_input_nodes);
            _impl->output_names.resize(num_output_nodes);

            for (size_t i = 0; i < num_input_nodes; i++)
            {
                Ort::AllocatedStringPtr input_name = _impl->session->GetInputNameAllocated(i, _impl->allocator);
                _impl->input_names[i] = input_name.get();
            }

            for (size_t i = 0; i < num_output_nodes; i++)
            {
                Ort::AllocatedStringPtr output_name = _impl->session->GetOutputNameAllocated(i, _impl->allocator);
                _impl->output_names[i] = output_name.get();

                Ort::TypeInfo type_info = _impl->session->GetOutputTypeInfo(i);
                auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
                _impl->output_shapes.push_back(tensor_info.GetShape());
            }

            _model_loaded = true;
            _error_message.clear();
            return true;
        }
        catch (const std::exception& e)
        {
            _error_message = rsutils::string::from() << "Failed to load model: " << e.what();
            _model_loaded = false;
            return false;
        }
#else
        _error_message = "ONNX Runtime not available. Build with ONNXRUNTIME_ROOT set.";
        return false;
#endif
    }

    detection_result ai_inference::infer(const cv::Mat& rgb_image)
    {
        detection_result result;
        result.input_size = rgb_image.size();

        if (!_model_loaded || rgb_image.empty())
        {
            return result;
        }

        auto start = std::chrono::high_resolution_clock::now();

#ifdef HAS_ONNXRUNTIME
        switch (_model_type)
        {
            case DETECTION:
                result = run_yolov8_detect(rgb_image);
                break;
            case SEGMENTATION:
                result = run_yolov8_seg(rgb_image);
                break;
            case POSE:
                result = run_yolov8_pose(rgb_image);
                break;
            default:
                break;
        }
#endif

        auto end = std::chrono::high_resolution_clock::now();
        result.inference_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        return result;
    }

    detection_result ai_inference::infer_with_depth(const cv::Mat& rgb_image, const cv::Mat& depth_image)
    {
        auto result = infer(rgb_image);

        if (!depth_image.empty() && !result.instances.empty())
        {
            estimate_3d_positions(result, depth_image);
        }

        return result;
    }

#ifdef HAS_ONNXRUNTIME
    detection_result ai_inference::run_yolov8_detect(const cv::Mat& image)
    {
        detection_result result;

        // Preprocess: resize and normalize
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32FC3, 1.0 / 255.0);

        // CHW format
        std::vector<cv::Mat> channels(3);
        cv::split(float_image, channels);
        std::vector<float> input_data;
        input_data.reserve(INPUT_WIDTH * INPUT_HEIGHT * 3);
        for (const auto& ch : channels)
        {
            std::vector<float> ch_data = ch.reshape(1, 1);
            input_data.insert(input_data.end(), ch_data.begin(), ch_data.end());
        }

        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, INPUT_HEIGHT, INPUT_WIDTH};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

        // Run inference
        std::vector<Ort::Value> output_tensors = _impl->session->Run(
            Ort::RunOptions{nullptr},
            _impl->input_names.data(), &input_tensor, 1,
            _impl->output_names.data(), _impl->output_names.size());

        // Parse output: [1, 84, 8400] for detection (4 bbox + 80 classes)
        float* output = output_tensors[0].GetTensorMutableData<float>();
        auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

        int num_classes = shape[1] - 4;
        int num_boxes = shape[2];

        struct detection_candidate
        {
            int class_id;
            float confidence;
            cv::Rect bbox;
        };

        std::vector<detection_candidate> candidates;

        for (int i = 0; i < num_boxes; ++i)
        {
            float x_center = output[0 * num_boxes + i];
            float y_center = output[1 * num_boxes + i];
            float w = output[2 * num_boxes + i];
            float h = output[3 * num_boxes + i];

            float max_score = 0;
            int max_class = 0;

            for (int c = 0; c < num_classes; ++c)
            {
                float score = output[(4 + c) * num_boxes + i];
                if (score > max_score)
                {
                    max_score = score;
                    max_class = c;
                }
            }

            if (max_score > _conf_threshold)
            {
                int x = static_cast<int>((x_center - w / 2) * image.cols / INPUT_WIDTH);
                int y = static_cast<int>((y_center - h / 2) * image.rows / INPUT_HEIGHT);
                int bw = static_cast<int>(w * image.cols / INPUT_WIDTH);
                int bh = static_cast<int>(h * image.rows / INPUT_HEIGHT);

                candidates.push_back({max_class, max_score, cv::Rect(x, y, bw, bh)});
            }
        }

        // NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(
            [&candidates](size_t i) { return candidates[i].bbox; },
            [&candidates](size_t i) { return candidates[i].confidence; },
            static_cast<int>(candidates.size()),
            _conf_threshold, _nms_threshold, indices);

        for (int idx : indices)
        {
            detection_instance instance;
            instance.class_id = candidates[idx].class_id;
            instance.class_name = (instance.class_id < COCO_CLASS_NAMES.size())
                ? COCO_CLASS_NAMES[instance.class_id] : "unknown";
            instance.confidence = candidates[idx].confidence;
            instance.bbox = candidates[idx].bbox;
            result.instances.push_back(instance);
        }

        return result;
    }

    detection_result ai_inference::run_yolov8_seg(const cv::Mat& image)
    {
        // Similar to detect, but also extracts mask prototypes
        // YOLOv8-seg output: [1, 116, 8400] where 116 = 4 bbox + 80 classes + 32 mask coeffs
        detection_result result;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(float_image, channels);
        std::vector<float> input_data;
        input_data.reserve(INPUT_WIDTH * INPUT_HEIGHT * 3);
        for (const auto& ch : channels)
        {
            std::vector<float> ch_data = ch.reshape(1, 1);
            input_data.insert(input_data.end(), ch_data.begin(), ch_data.end());
        }

        std::vector<int64_t> input_shape = {1, 3, INPUT_HEIGHT, INPUT_WIDTH};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

        std::vector<Ort::Value> output_tensors = _impl->session->Run(
            Ort::RunOptions{nullptr},
            _impl->input_names.data(), &input_tensor, 1,
            _impl->output_names.data(), _impl->output_names.size());

        // For segmentation, we have two outputs:
        // [0]: [1, 116, 8400] - boxes + classes + mask coefficients
        // [1]: [1, 32, 160, 160] - mask prototypes
        float* output = output_tensors[0].GetTensorMutableData<float>();
        auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

        int num_classes = 80;
        int num_mask_coeffs = 32;
        int num_boxes = shape[2];

        struct seg_candidate
        {
            int class_id;
            float confidence;
            cv::Rect bbox;
            std::vector<float> mask_coeffs;
        };

        std::vector<seg_candidate> candidates;

        for (int i = 0; i < num_boxes; ++i)
        {
            float x_center = output[0 * num_boxes + i];
            float y_center = output[1 * num_boxes + i];
            float w = output[2 * num_boxes + i];
            float h = output[3 * num_boxes + i];

            float max_score = 0;
            int max_class = 0;

            for (int c = 0; c < num_classes; ++c)
            {
                float score = output[(4 + c) * num_boxes + i];
                if (score > max_score)
                {
                    max_score = score;
                    max_class = c;
                }
            }

            if (max_score > _conf_threshold)
            {
                seg_candidate cand;
                cand.class_id = max_class;
                cand.confidence = max_score;
                int x = static_cast<int>((x_center - w / 2) * image.cols / INPUT_WIDTH);
                int y = static_cast<int>((y_center - h / 2) * image.rows / INPUT_HEIGHT);
                int bw = static_cast<int>(w * image.cols / INPUT_WIDTH);
                int bh = static_cast<int>(h * image.rows / INPUT_HEIGHT);
                cand.bbox = cv::Rect(x, y, bw, bh);

                // Extract mask coefficients
                int coeff_offset = 4 + num_classes;
                cand.mask_coeffs.resize(num_mask_coeffs);
                for (int m = 0; m < num_mask_coeffs; ++m)
                {
                    cand.mask_coeffs[m] = output[(coeff_offset + m) * num_boxes + i];
                }

                candidates.push_back(cand);
            }
        }

        // NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(
            [&candidates](size_t i) { return candidates[i].bbox; },
            [&candidates](size_t i) { return candidates[i].confidence; },
            static_cast<int>(candidates.size()),
            _conf_threshold, _nms_threshold, indices);

        // If we have mask prototypes, decode masks
        if (output_tensors.size() > 1)
        {
            float* mask_protos = output_tensors[1].GetTensorMutableData<float>();
            auto proto_shape = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();
            int proto_h = proto_shape[2];
            int proto_w = proto_shape[3];
            int num_proto = proto_shape[1];

            for (int idx : indices)
            {
                detection_instance instance;
                instance.class_id = candidates[idx].class_id;
                instance.class_name = (instance.class_id < COCO_CLASS_NAMES.size())
                    ? COCO_CLASS_NAMES[instance.class_id] : "unknown";
                instance.confidence = candidates[idx].confidence;
                instance.bbox = candidates[idx].bbox;

                // Decode mask: (32 coeffs) x (32 x 160x160 protos) -> 160x160 mask
                cv::Mat mask_coeff_mat(1, num_mask_coeffs, CV_32F, candidates[idx].mask_coeffs.data());
                cv::Mat proto_mat(num_proto, proto_h * proto_w, CV_32F, mask_protos);
                cv::Mat raw_mask = mask_coeff_mat * proto_mat;
                raw_mask = raw_mask.reshape(1, proto_h);

                // Crop and resize to bbox
                cv::Mat cropped_mask;
                cv::resize(raw_mask, cropped_mask, cv::Size(instance.bbox.width, instance.bbox.height));

                // Binarize
                instance.mask = cropped_mask > 0.0f;
                instance.mask.convertTo(instance.mask, CV_8U, 255);

                result.instances.push_back(instance);
            }
        }
        else
        {
            // Fallback: just detection without masks
            for (int idx : indices)
            {
                detection_instance instance;
                instance.class_id = candidates[idx].class_id;
                instance.class_name = (instance.class_id < COCO_CLASS_NAMES.size())
                    ? COCO_CLASS_NAMES[instance.class_id] : "unknown";
                instance.confidence = candidates[idx].confidence;
                instance.bbox = candidates[idx].bbox;
                result.instances.push_back(instance);
            }
        }

        return result;
    }

    detection_result ai_inference::run_yolov8_pose(const cv::Mat& image)
    {
        // YOLOv8-pose output: [1, 56, 8400] where 56 = 4 bbox + 1 (obj) + 51 (17 keypoints * 3)
        detection_result result;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(float_image, channels);
        std::vector<float> input_data;
        input_data.reserve(INPUT_WIDTH * INPUT_HEIGHT * 3);
        for (const auto& ch : channels)
        {
            std::vector<float> ch_data = ch.reshape(1, 1);
            input_data.insert(input_data.end(), ch_data.begin(), ch_data.end());
        }

        std::vector<int64_t> input_shape = {1, 3, INPUT_HEIGHT, INPUT_WIDTH};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

        std::vector<Ort::Value> output_tensors = _impl->session->Run(
            Ort::RunOptions{nullptr},
            _impl->input_names.data(), &input_tensor, 1,
            _impl->output_names.data(), _impl->output_names.size());

        float* output = output_tensors[0].GetTensorMutableData<float>();
        auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

        int num_boxes = shape[2];
        int num_keypoints = 17;

        struct pose_candidate
        {
            float confidence;
            cv::Rect bbox;
            std::vector<detection_instance::Keypoint> keypoints;
        };

        std::vector<pose_candidate> candidates;

        for (int i = 0; i < num_boxes; ++i)
        {
            float x_center = output[0 * num_boxes + i];
            float y_center = output[1 * num_boxes + i];
            float w = output[2 * num_boxes + i];
            float h = output[3 * num_boxes + i];

            // For pose, confidence is at index 4
            float confidence = output[4 * num_boxes + i];

            if (confidence > _conf_threshold)
            {
                pose_candidate cand;
                cand.confidence = confidence;
                int x = static_cast<int>((x_center - w / 2) * image.cols / INPUT_WIDTH);
                int y = static_cast<int>((y_center - h / 2) * image.rows / INPUT_HEIGHT);
                int bw = static_cast<int>(w * image.cols / INPUT_WIDTH);
                int bh = static_cast<int>(h * image.rows / INPUT_HEIGHT);
                cand.bbox = cv::Rect(x, y, bw, bh);

                // Extract keypoints (51 values: x, y, conf for each of 17 keypoints)
                int kp_offset = 5;
                cand.keypoints.resize(num_keypoints);
                for (int k = 0; k < num_keypoints; ++k)
                {
                    cand.keypoints[k].x = output[(kp_offset + k * 3 + 0) * num_boxes + i] * image.cols / INPUT_WIDTH;
                    cand.keypoints[k].y = output[(kp_offset + k * 3 + 1) * num_boxes + i] * image.rows / INPUT_HEIGHT;
                    cand.keypoints[k].confidence = output[(kp_offset + k * 3 + 2) * num_boxes + i];
                }

                candidates.push_back(cand);
            }
        }

        // NMS
        std::vector<int> indices;
        std::vector<float> confidences;
        std::vector<cv::Rect> bboxes;
        for (const auto& c : candidates)
        {
            confidences.push_back(c.confidence);
            bboxes.push_back(c.bbox);
        }
        cv::dnn::NMSBoxes(bboxes, confidences, _conf_threshold, _nms_threshold, indices);

        for (int idx : indices)
        {
            detection_instance instance;
            instance.class_id = 0;  // person
            instance.class_name = "person";
            instance.confidence = candidates[idx].confidence;
            instance.bbox = candidates[idx].bbox;
            instance.keypoints = candidates[idx].keypoints;
            result.instances.push_back(instance);
        }

        return result;
    }
#endif

    void ai_inference::estimate_3d_positions(detection_result& result, const cv::Mat& depth_image)
    {
        if (depth_image.empty() || result.instances.empty())
            return;

        for (auto& instance : result.instances)
        {
            // Use center of bbox to sample depth
            int cx = instance.bbox.x + instance.bbox.width / 2;
            int cy = instance.bbox.y + instance.bbox.height / 2;

            // Clamp to image bounds
            cx = std::max(0, std::min(cx, depth_image.cols - 1));
            cy = std::max(0, std::min(cy, depth_image.rows - 1));

            // Sample depth from a small region around center
            cv::Rect roi(cx - 5, cy - 5, 10, 10);
            roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);

            cv::Mat roi_depth = depth_image(roi);
            cv::Scalar mean_depth;
            cv::mean(roi_depth, mean_depth);

            // Assuming depth is in millimeters (common for RealSense)
            // Convert to meters
            instance.position_3d = cv::Vec3f(
                static_cast<float>(cx),
                static_cast<float>(cy),
                static_cast<float>(mean_depth[0] / 1000.0f)
            );
            instance.has_3d_position = true;
        }
    }
}
