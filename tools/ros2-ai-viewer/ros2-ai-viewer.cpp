// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "viewer.h"
#include "os.h"
#include "ux-window.h"
#include "ros2-bridge.h"
#include "ai-inference.h"
#include "detection-overlay.h"

#include <common/cli.h>

#include <thread>
#include <iostream>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>

using namespace rs2;

// Global state
struct app_state
{
    std::mutex data_mutex;
    cv::Mat current_rgb;
    rs2::points current_pointcloud;
    detection_result current_detections;
    double fps = 0.0;
    double ai_fps = 0.0;
    std::atomic<bool> running{true};
};

// Convert cv::Mat to OpenGL texture and upload
std::shared_ptr<texture_buffer> upload_cv_mat(const cv::Mat& rgb)
{
    auto tex = std::make_shared<texture_buffer>();

    if (rgb.empty())
        return tex;

    cv::Mat rgb_display;
    if (rgb.channels() == 3)
    {
        cv::cvtColor(rgb, rgb_display, cv::COLOR_BGR2RGB);
    }
    else
    {
        rgb_display = rgb;
    }

    tex->upload_image(rgb_display.cols, rgb_display.rows, rgb_display.data, GL_RGB);

    return tex;
}

// Draw RGB frame with detection overlays
void draw_rgb_with_detections(const rect& viewer_rect, const cv::Mat& rgb,
                               const detection_result& detections, detection_overlay& overlay,
                               ImFont* font)
{
    if (rgb.empty())
        return;

    auto tex = upload_cv_mat(rgb);
    if (!tex)
        return;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex->get_gl_handle());

    glBegin(GL_QUADS);
    glTexCoord2f(0, 1); glVertex2f(viewer_rect.x, viewer_rect.y);
    glTexCoord2f(1, 1); glVertex2f(viewer_rect.x + viewer_rect.w, viewer_rect.y);
    glTexCoord2f(1, 0); glVertex2f(viewer_rect.x + viewer_rect.w, viewer_rect.y + viewer_rect.h);
    glTexCoord2f(0, 0); glVertex2f(viewer_rect.x, viewer_rect.y + viewer_rect.h);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Draw detection overlays
    overlay.draw_2d_overlay(viewer_rect, detections, font);
}

// Draw info panel
void draw_info_panel(app_state& state, const ros2_bridge& bridge, const ai_inference& ai)
{
    ImGui::Begin("Info Panel", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("ROS2 Connection: %s", bridge.is_connected() ? "YES" : "NO");
    if (!bridge.is_connected())
    {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Error: %s", bridge.get_error_message().c_str());
    }

    ImGui::Separator();

    ImGui::Text("AI Model: %s", ai.is_loaded() ? "LOADED" : "NOT LOADED");
    if (ai.is_loaded())
    {
        const char* type_str = "Unknown";
        switch (ai.get_model_type())
        {
            case ai_inference::DETECTION: type_str = "Detection"; break;
            case ai_inference::SEGMENTATION: type_str = "Segmentation"; break;
            case ai_inference::POSE: type_str = "Pose"; break;
            default: break;
        }
        ImGui::Text("  Type: %s", type_str);
    }
    else if (!ai.get_error_message().empty())
    {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "  Error: %s", ai.get_error_message().c_str());
    }

    ImGui::Separator();

    ImGui::Text("FPS: %.1f", state.fps);
    ImGui::Text("AI Inference: %.1f ms", state.current_detections.inference_time_ms);
    ImGui::Text("Detections: %d", (int)state.current_detections.instances.size());

    ImGui::End();
}

int main(int argc, const char** argv) try
{
    rs2::cli cmd("ros2-ai-viewer");

    rs2::cli_no_context::value<std::string> rgb_topic_arg("rgb-topic", "string", "/camera/color/image_raw", "RGB image topic name");
    rs2::cli_no_context::value<std::string> pc_topic_arg("pc-topic", "string", "/camera/depth/color/points", "Point cloud topic name");
    rs2::cli_no_context::value<std::string> model_arg("model", "string", "", "YOLOv8 ONNX model path (optional)");
    rs2::cli_no_context::value<float> conf_arg("conf", "float", 0.5f, "Detection confidence threshold");

    cmd.add(rgb_topic_arg);
    cmd.add(pc_topic_arg);
    cmd.add(model_arg);
    cmd.add(conf_arg);

    auto settings = cmd.process(argc, argv);

    std::string rgb_topic = rgb_topic_arg.getValue();
    std::string pc_topic = pc_topic_arg.getValue();
    std::string model_path = model_arg.getValue();
    float conf_threshold = conf_arg.getValue();

    // Initialize ROS2 bridge
    ros2_bridge bridge;
    bool ros2_ok = bridge.initialize(rgb_topic, pc_topic);

    if (ros2_ok)
    {
        bridge.start();
        std::cout << "ROS2 bridge started. Topics: RGB=" << rgb_topic << ", PC=" << pc_topic << std::endl;
    }
    else
    {
        std::cerr << "ROS2 initialization failed: " << bridge.get_error_message() << std::endl;
        std::cerr << "Continuing without ROS2 support..." << std::endl;
    }

    // Initialize AI inference
    ai_inference ai;
    if (!model_path.empty())
    {
        if (ai.load_model(model_path, conf_threshold))
        {
            std::cout << "AI model loaded: " << model_path << std::endl;
        }
        else
        {
            std::cerr << "Failed to load AI model: " << ai.get_error_message() << std::endl;
        }
    }

    // Detection overlay renderer
    detection_overlay overlay;

    // Application state
    app_state state;

    // Create viewer window
    context ctx;
    ux_window window("ROS2 AI Viewer", ctx);

    // FPS tracking
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    // Main loop
    while (window && state.running)
    {
        // Poll for new frames from ROS2
        synced_frame_data ros_data;
        if (bridge.poll_for_frame(ros_data))
        {
            std::lock_guard<std::mutex> lock(state.data_mutex);
            state.current_rgb = ros_data.rgb_image.clone();

            // Run AI inference if model is loaded
            if (ai.is_loaded() && !state.current_rgb.empty())
            {
                state.current_detections = ai.infer(state.current_rgb);
            }

            // FPS calculation
            frame_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            if (elapsed >= 1.0)
            {
                state.fps = frame_count / elapsed;
                frame_count = 0;
                last_fps_time = now;
            }
        }

        // Get viewer dimensions
        auto output_height = 100.0f;
        rect viewer_rect = {340.f, 50.f, window.width() - 340.f, window.height() - 50.f - output_height};

        // Top bar
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({window.width(), 50.f});
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoSavedSettings;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 10));
        ImGui::Begin("Top Bar", nullptr, flags);

        ImGui::PushFont(window.get_large_font());
        ImGui::Text("ROS2 AI Viewer");
        ImGui::PopFont();

        ImGui::SameLine(window.width() - 200);
        ImGui::Text("FPS: %.1f", state.fps);

        ImGui::End();
        ImGui::PopStyleVar();

        // Left panel - controls
        ImGui::SetNextWindowPos({0, 50});
        ImGui::SetNextWindowSize({340.f, window.height() - 50.f - output_height});
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 1.0f));
        ImGui::Begin("Control Panel", nullptr, flags | ImGuiWindowFlags_AlwaysVerticalScrollbar);

        // ROS2 settings
        ImGui::Text("ROS2 Topics");
        ImGui::Text("  RGB: %s", rgb_topic.c_str());
        ImGui::Text("  PC:  %s", pc_topic.c_str());
        ImGui::Separator();

        // AI settings
        ImGui::Text("AI Model");
        if (ImGui::Button("Reload Model"))
        {
            if (!model_path.empty())
            {
                ai.load_model(model_path, conf_threshold);
            }
        }
        ImGui::SliderFloat("Confidence Threshold", &conf_threshold, 0.1f, 0.9f);
        ai.set_confidence_threshold(conf_threshold);
        ImGui::Separator();

        // Detection display settings
        overlay.draw_settings();

        ImGui::End();
        ImGui::PopStyleColor();

        // Main viewer area
        ImGui::SetNextWindowPos({340.f, 50.f});
        ImGui::SetNextWindowSize({window.width() - 340.f, window.height() - 50.f - output_height});
        ImGui::Begin("Viewer", nullptr, flags);

        {
            std::lock_guard<std::mutex> lock(state.data_mutex);

            if (!state.current_rgb.empty())
            {
                draw_rgb_with_detections(viewer_rect, state.current_rgb,
                                         state.current_detections, overlay, window.get_font());
            }
            else
            {
                // No data overlay
                ImGui::SetCursorPos({viewer_rect.w / 2 - 100, viewer_rect.h / 2});
                ImGui::Text("Waiting for ROS2 data...");
            }
        }

        ImGui::End();

        // Info panel (top-right)
        draw_info_panel(state, bridge, ai);

        // Detection legend
        {
            std::lock_guard<std::mutex> lock(state.data_mutex);
            overlay.draw_legend(state.current_detections);
        }

        // Output panel (bottom)
        ImGui::SetNextWindowPos({340.f, window.height() - output_height});
        ImGui::SetNextWindowSize({window.width() - 340.f, output_height});
        ImGui::Begin("Output", nullptr, flags);
        ImGui::Text("Log output panel - coming soon");
        ImGui::End();
    }

    // Cleanup
    bridge.stop();

    return EXIT_SUCCESS;
}
catch (const error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
