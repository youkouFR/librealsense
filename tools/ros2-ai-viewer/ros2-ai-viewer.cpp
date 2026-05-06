// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "rendering.h"
#include "model-views.h"
#include "os.h"
#include "ros2-bridge.h"
#include "ai-inference.h"
#include "detection-overlay.h"

#include <common/cli.h>

#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include "glad/glad.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <thread>
#include <iostream>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace rs2;

// Global state
struct app_state
{
    std::mutex data_mutex;
    cv::Mat current_rgb;
    std::shared_ptr<texture_buffer> current_tex;
    point_cloud_data current_pc_data;
    detection_result current_detections;
    double fps = 0.0;
    std::atomic<bool> running{true};

    // 3D View state
    float cam_rot_x = 0.0f;
    float cam_rot_y = 0.0f;
    float cam_dist = 2.0f;
    bool show_3d = false;
};

// 3D Point Cloud Renderer using immediate-mode GL (matches realsense-viewer fallback)
struct pc_renderer
{
    std::vector<float> vertices;
    std::vector<float> colors;

    void update(const point_cloud_data& data)
    {
        vertices = data.vertices;
        colors = data.colors;
        if (colors.empty() && !vertices.empty())
        {
            colors.resize(vertices.size(), 0.8f);
        }
    }

    void render()
    {
        if (vertices.empty() || vertices.size() / 3 == 0) return;
        size_t n = vertices.size() / 3;
        bool has_color = !colors.empty() && colors.size() >= n * 3;

        glBegin(GL_POINTS);
        for (size_t i = 0; i < n; ++i)
        {
            if (has_color)
                glColor3f(colors[i*3], colors[i*3+1], colors[i*3+2]);
            else
                glColor3f(0.8f, 0.8f, 0.8f);
            glVertex3f(vertices[i*3], vertices[i*3+1], vertices[i*3+2]);
        }
        glEnd();
    }

    void cleanup() {}
};

// Convert cv::Mat to OpenGL texture and upload
std::shared_ptr<texture_buffer> upload_cv_mat(const cv::Mat& rgb)
{
    auto tex = std::make_shared<texture_buffer>();
    if (rgb.empty()) return tex;

    cv::Mat rgb_display;
    if (rgb.channels() == 3)
        cv::cvtColor(rgb, rgb_display, cv::COLOR_BGR2RGB);
    else
        rgb_display = rgb;

    tex->upload_image(rgb_display.cols, rgb_display.rows, rgb_display.data, GL_RGB);
    return tex;
}

// Draw info panel
void draw_info_panel(app_state& state, const ros2_bridge& bridge, const ai_inference& ai)
{
    ImGui::Begin("Info Panel", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("ROS2 Connection: %s", bridge.is_connected() ? "YES" : "NO");
    if (!bridge.is_connected())
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Error: %s", bridge.get_error_message().c_str());
    ImGui::Separator();
    ImGui::Text("AI Model: %s", ai.is_loaded() ? "LOADED" : "NOT LOADED");
    if (ai.is_loaded())
    {
        const char* type_str = "Unknown";
        switch (ai.get_model_type()) {
            case ai_inference::DETECTION: type_str = "Detection"; break;
            case ai_inference::SEGMENTATION: type_str = "Segmentation"; break;
            case ai_inference::POSE: type_str = "Pose"; break;
            default: break;
        }
        ImGui::Text("  Type: %s", type_str);
    }
    else if (!ai.get_error_message().empty())
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "  Error: %s", ai.get_error_message().c_str());
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
    cmd.add(rgb_topic_arg); cmd.add(pc_topic_arg); cmd.add(model_arg); cmd.add(conf_arg);

    auto settings = cmd.process(argc, argv);
    std::string rgb_topic = rgb_topic_arg.getValue();
    std::string pc_topic = pc_topic_arg.getValue();
    std::string model_path = model_arg.getValue();
    float conf_threshold = conf_arg.getValue();

    // Initialize ROS2 bridge
    ros2_bridge bridge;
    bool ros2_ok = bridge.initialize(rgb_topic, pc_topic);
    if (ros2_ok) {
        bridge.start();
        std::cout << "ROS2 bridge started. Topics: RGB=" << rgb_topic << ", PC=" << pc_topic << std::endl;
    } else {
        std::cerr << "ROS2 initialization failed: " << bridge.get_error_message() << std::endl;
    }

    // Initialize AI inference
    ai_inference ai;
    if (!model_path.empty() && !ai.load_model(model_path, conf_threshold))
        std::cerr << "Failed to load AI model: " << ai.get_error_message() << std::endl;

    detection_overlay overlay;
    app_state state;
    pc_renderer pc_render;

    // Initialize GLFW and create window
    if (!glfwInit()) { std::cerr << "Failed to initialize GLFW" << std::endl; return EXIT_FAILURE; }

    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    GLFWwindow* win = glfwCreateWindow(1280, 720, "ROS2 AI Viewer", nullptr, nullptr);
    if (!win) { glfwTerminate(); return EXIT_FAILURE; }

    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        { std::cerr << "Failed to initialize GLAD" << std::endl; glfwTerminate(); return EXIT_FAILURE; }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    ImFont* large_font = ImGui::GetIO().Fonts->AddFontDefault();
    large_font->Scale = 1.5f;

    // FPS tracking
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    // Main loop
    while (!glfwWindowShouldClose(win) && state.running)
    {
        glfwPollEvents();

        // Poll for new frames from ROS2
        synced_frame_data ros_data;
        if (bridge.poll_for_frame(ros_data))
        {
            std::lock_guard<std::mutex> lock(state.data_mutex);
            state.current_rgb = ros_data.rgb_image.clone();
            if (!state.current_rgb.empty())
                state.current_tex = upload_cv_mat(state.current_rgb);

            if (ros_data.pc_data.count > 0) {
                state.current_pc_data = ros_data.pc_data;
                pc_render.update(state.current_pc_data);
            }

            if (ai.is_loaded() && !state.current_rgb.empty())
                state.current_detections = ai.infer(state.current_rgb);

            frame_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            if (elapsed >= 1.0) {
                state.fps = frame_count / elapsed;
                frame_count = 0;
                last_fps_time = now;
            }
        }

        // Get window dimensions
        int win_w, win_h;
        glfwGetFramebufferSize(win, &win_w, &win_h);
        float width = (float)win_w;
        float height = (float)win_h;

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings;
        float output_height = 100.0f;

        // Top bar
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({width, 50.f});
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 10));
        ImGui::Begin("Top Bar", nullptr, flags);
        ImGui::PushFont(large_font);
        ImGui::Text("ROS2 AI Viewer");
        ImGui::PopFont();
        ImGui::SameLine(width - 200);
        ImGui::Text("FPS: %.1f", state.fps);
        ImGui::End();
        ImGui::PopStyleVar();

        // Left panel - controls
        ImGui::SetNextWindowPos({0, 50});
        ImGui::SetNextWindowSize({340.f, height - 50.f - output_height});
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 1.0f));
        ImGui::Begin("Control Panel", nullptr, flags | ImGuiWindowFlags_AlwaysVerticalScrollbar);

        ImGui::Text("View Mode");
        if (ImGui::Button(state.show_3d ? "Switch to 2D" : "Switch to 3D"))
            state.show_3d = !state.show_3d;
        ImGui::SameLine();
        ImGui::Text(state.show_3d ? "3D PointCloud" : "2D RGB");
        ImGui::Separator();

        if (state.show_3d) {
            ImGui::Text("3D Camera Control");
            ImGui::SliderFloat("Distance", &state.cam_dist, 0.1f, 20.0f);
            ImGui::SliderAngle("Rotation X", &state.cam_rot_x, -180.0f, 180.0f);
            ImGui::SliderAngle("Rotation Y", &state.cam_rot_y, -180.0f, 180.0f);
            ImGui::Separator();
        }

        ImGui::Text("ROS2 Topics");
        ImGui::Text("  RGB: %s", rgb_topic.c_str());
        ImGui::Text("  PC:  %s", pc_topic.c_str());
        ImGui::Separator();

        ImGui::Text("AI Model");
        if (ImGui::Button("Reload Model"))
            if (!model_path.empty()) ai.load_model(model_path, conf_threshold);
        ImGui::SliderFloat("Confidence Threshold", &conf_threshold, 0.1f, 0.9f);
        ai.set_confidence_threshold(conf_threshold);
        ImGui::Separator();
        overlay.draw_settings();
        ImGui::End();
        ImGui::PopStyleColor();

        // Main viewer area
        ImGui::SetNextWindowPos({340.f, 50.f});
        ImGui::SetNextWindowSize({width - 340.f, height - 50.f - output_height});
        ImGui::Begin("Viewer", nullptr, flags);

        {
            std::lock_guard<std::mutex> lock(state.data_mutex);

            if (state.show_3d && state.current_pc_data.count > 0)
            {
                // === 3D Point Cloud View using ImDrawList (CPU projection) ===
                // This completely avoids OpenGL state conflicts with ImGui's backend
                ImVec2 avail = ImGui::GetContentRegionAvail();
                ImVec2 win_pos = ImGui::GetWindowPos();
                ImVec2 content_min = ImGui::GetCursorScreenPos(); // absolute

                auto& pc = state.current_pc_data;

                // Compute bounding box for auto-centering
                float min_x=1e9, max_x=-1e9, min_y=1e9, max_y=-1e9, min_z=1e9, max_z=-1e9;
                size_t step = std::max(size_t(1), pc.count / 200);
                for (size_t i = 0; i < pc.count; i += step) {
                    float x = pc.vertices[i*3], y = pc.vertices[i*3+1], z = pc.vertices[i*3+2];
                    if (x < min_x) min_x = x; if (x > max_x) max_x = x;
                    if (y < min_y) min_y = y; if (y > max_y) max_y = y;
                    if (z < min_z) min_z = z; if (z > max_z) max_z = z;
                }
                float cx = (min_x+max_x)/2, cy = (min_y+max_y)/2, cz = (min_z+max_z)/2;
                float extent = std::max({max_x-min_x, max_y-min_y, max_z-min_z});
                if (extent < 0.1f) extent = 1.0f;
                state.cam_dist = extent * 3.0f + 2.0f;

                // CPU projection matrices
                float fov = 60.0f * 3.14159f / 180.0f;
                float aspect = avail.x / std::max(avail.y, 1.0f);
                float f = 1.0f / tanf(fov / 2.0f);
                float near_plane = 0.01f, far_plane = extent * 10.0f + 100.0f;

                // Camera position (orbit)
                float rad_x = state.cam_rot_x * 3.14159f / 180.0f;
                float rad_y = state.cam_rot_y * 3.14159f / 180.0f;
                float ex = state.cam_dist * sinf(rad_y) * cosf(rad_x) + cx;
                float ey = state.cam_dist * sinf(rad_x) + cy;
                float ez = state.cam_dist * cosf(rad_y) * cosf(rad_x) + cz;

                // Forward vector: from eye to center
                float fx = cx - ex, fy = cy - ey, fz = cz - ez;
                float fl = sqrtf(fx*fx + fy*fy + fz*fz);
                fx /= fl; fy /= fl; fz /= fl;

                // Right vector: cross(forward, world_up=(0,1,0)) = (-fz, 0, fx)
                float sx = -fz;
                float sy = 0.0f;
                float sz = fx;
                float sl = sqrtf(sx*sx + sy*sy + sz*sz);
                if (sl > 0.001f) { sx /= sl; sy /= sl; sz /= sl; }
                else { sx = 1.0f; sy = 0.0f; sz = 0.0f; } // fallback: forward parallel to world up

                // Up vector: cross(right, forward)
                float ux = sy * fz - sz * fy;
                float uy = sz * fx - sx * fz;
                float uz = sx * fy - sy * fx;

                // Transform a point to screen space
                auto project = [&](float px, float py, float pz, float& sx_out, float& sy_out) -> bool {
                    // View transform
                    float dx = px - ex, dy = py - ey, dz = pz - ez;
                    float vx = sx*dx + sy*dy + sz*dz;  // right
                    float vy = ux*dx + uy*dy + uz*dz;  // up
                    float vz = fx*dx + fy*dy + fz*dz;  // forward (positive = in front of camera)

                    // Clipping (vz is the positive depth from camera)
                    if (vz <= near_plane || vz > far_plane) return false;

                    // Perspective divide (vz = positive depth)
                    float x_clip = vx * f / (aspect * vz);
                    float y_clip = vy * f / vz;

                    // NDC to screen
                    sx_out = (x_clip + 1.0f) * 0.5f * avail.x + content_min.x;
                    sy_out = (1.0f - y_clip) * 0.5f * avail.y + content_min.y;
                    return true;
                };

                ImDrawList* dl = ImGui::GetWindowDrawList();

                // Draw background
                dl->AddRectFilled(content_min, ImVec2(content_min.x + avail.x, content_min.y + avail.y),
                    IM_COL32(15, 15, 20, 255));

                // Draw all points (subsampled for performance)
                size_t pc_n = pc.vertices.size() / 3;
                size_t draw_step = std::max(size_t(1), pc_n / 30000);
                for (size_t i = 0; i < pc_n; i += draw_step)
                {
                    float px = pc.vertices[i*3], py = pc.vertices[i*3+1], pz = pc.vertices[i*3+2];
                    float sx, sy;
                    if (project(px, py, pz, sx, sy))
                    {
                        // Skip points outside the visible area
                        if (sx < content_min.x - 10 || sx > content_min.x + avail.x + 10 ||
                            sy < content_min.y - 10 || sy > content_min.y + avail.y + 10)
                            continue;

                        ImU32 col;
                        if (pc.colors.size() >= (i+1)*3)
                            col = IM_COL32((int)(pc.colors[i*3]*255), (int)(pc.colors[i*3+1]*255), (int)(pc.colors[i*3+2]*255), 255);
                        else
                            col = IM_COL32(200, 200, 200, 255);

                        dl->AddCircleFilled(ImVec2(sx, sy), 2.0f, col);
                    }
                }

                // ImGui overlay
                ImGui::SetCursorScreenPos(content_min);
                ImGui::Text("Pts: %zu | Center: (%.2f,%.2f,%.2f) | Dist: %.1f | BBox: %.1f",
                    pc.count, cx, cy, cz, state.cam_dist, extent);
            }
            else if (!state.show_3d)
            {
                // 2D RGB View
                if (state.current_tex && state.current_tex->get_gl_handle() && !state.current_rgb.empty())
                {
                    ImVec2 avail = ImGui::GetContentRegionAvail();
                    float img_w = (float)state.current_rgb.cols;
                    float img_h = (float)state.current_rgb.rows;
                    float scale = std::min(avail.x / img_w, avail.y / img_h);
                    ImVec2 draw_size(img_w * scale, img_h * scale);
                    ImVec2 offset((avail.x - draw_size.x) / 2, (avail.y - draw_size.y) / 2);
                    ImGui::SetCursorPos(offset);
                    ImGui::Image((void*)(intptr_t)state.current_tex->get_gl_handle(), draw_size);
                }
                else
                {
                    ImVec2 avail = ImGui::GetContentRegionAvail();
                    ImGui::SetCursorPos({avail.x / 2 - 50, avail.y / 2});
                    ImGui::Text("Waiting for ROS2 data...");
                }
            }
            else
            {
                ImVec2 avail = ImGui::GetContentRegionAvail();
                ImGui::SetCursorPos({avail.x / 2 - 50, avail.y / 2});
                ImGui::Text("Waiting for point cloud data...");
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
        ImGui::SetNextWindowPos({340.f, height - output_height});
        ImGui::SetNextWindowSize({width - 340.f, output_height});
        ImGui::Begin("Output", nullptr, flags);
        ImGui::Text("Log output panel - coming soon");
        ImGui::End();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(win, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    // Cleanup
    pc_render.cleanup();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
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
