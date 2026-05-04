// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include "detection-overlay.h"
#include <sstream>
#include <iomanip>

namespace rs2
{
    // COCO class colors (vibrant palette)
    const std::vector<ImVec4> detection_overlay::CLASS_COLORS = {
        ImVec4(0.96f, 0.24f, 0.38f, 1.0f),  // person - red
        ImVec4(0.20f, 0.63f, 0.96f, 1.0f),  // bicycle - blue
        ImVec4(0.20f, 0.80f, 0.20f, 1.0f),  // car - green
        ImVec4(0.96f, 0.63f, 0.20f, 1.0f),  // motorcycle - orange
        ImVec4(0.60f, 0.20f, 0.96f, 1.0f),  // airplane - purple
        ImVec4(0.20f, 0.96f, 0.96f, 1.0f),  // bus - cyan
        ImVec4(0.96f, 0.20f, 0.80f, 1.0f),  // train - pink
        ImVec4(0.80f, 0.80f, 0.20f, 1.0f),  // truck - yellow
        ImVec4(0.40f, 0.40f, 0.96f, 1.0f),  // boat - dark blue
        ImVec4(0.96f, 0.40f, 0.20f, 1.0f),  // traffic light - dark orange
        ImVec4(0.20f, 0.96f, 0.40f, 1.0f),  // fire hydrant - lime
        ImVec4(0.60f, 0.96f, 0.20f, 1.0f),  // stop sign - yellow-green
        ImVec4(0.96f, 0.20f, 0.60f, 1.0f),  // parking meter - magenta
        ImVec4(0.20f, 0.40f, 0.96f, 1.0f),  // bench - royal blue
        ImVec4(0.96f, 0.80f, 0.20f, 1.0f),  // bird - gold
        ImVec4(0.40f, 0.96f, 0.60f, 1.0f),  // cat - mint
        ImVec4(0.60f, 0.20f, 0.40f, 1.0f),  // dog - maroon
        ImVec4(0.96f, 0.40f, 0.60f, 1.0f),  // horse - coral
        ImVec4(0.20f, 0.60f, 0.96f, 1.0f),  // sheep - sky blue
        ImVec4(0.80f, 0.96f, 0.20f, 1.0f),  // cow - chartreuse
        ImVec4(0.40f, 0.20f, 0.96f, 1.0f),  // elephant - violet
        ImVec4(0.96f, 0.60f, 0.40f, 1.0f),  // bear - peach
        ImVec4(0.20f, 0.80f, 0.60f, 1.0f),  // zebra - teal
        ImVec4(0.60f, 0.40f, 0.96f, 1.0f),  // giraffe - lavender
        ImVec4(0.96f, 0.20f, 0.40f, 1.0f),  // backpack - crimson
        ImVec4(0.40f, 0.96f, 0.40f, 1.0f),  // umbrella - lime green
        ImVec4(0.96f, 0.96f, 0.20f, 1.0f),  // handbag - lemon
        ImVec4(0.20f, 0.20f, 0.96f, 1.0f),  // tie - navy
        ImVec4(0.80f, 0.20f, 0.96f, 1.0f),  // suitcase - plum
        ImVec4(0.96f, 0.40f, 0.80f, 1.0f),  // frisbee - hot pink
        ImVec4(0.40f, 0.80f, 0.96f, 1.0f),  // skis - light blue
        ImVec4(0.96f, 0.80f, 0.40f, 1.0f),  // snowboard - amber
        ImVec4(0.80f, 0.40f, 0.20f, 1.0f),  // sports ball - brown
        ImVec4(0.20f, 0.96f, 0.80f, 1.0f),  // kite - aqua
        ImVec4(0.60f, 0.96f, 0.60f, 1.0f),  // baseball bat - pale green
        ImVec4(0.96f, 0.20f, 0.20f, 1.0f),  // baseball glove - firebrick
        ImVec4(0.40f, 0.60f, 0.20f, 1.0f),  // skateboard - olive
        ImVec4(0.20f, 0.40f, 0.60f, 1.0f),  // surfboard - steel blue
        ImVec4(0.60f, 0.20f, 0.80f, 1.0f),  // tennis racket - orchid
        ImVec4(0.80f, 0.60f, 0.96f, 1.0f),  // bottle - thistle
        ImVec4(0.96f, 0.60f, 0.80f, 1.0f),  // wine glass - pink
        ImVec4(0.60f, 0.96f, 0.80f, 1.0f),  // cup - pale turquoise
        ImVec4(0.80f, 0.96f, 0.60f, 1.0f),  // fork - pale goldenrod
        ImVec4(0.96f, 0.96f, 0.60f, 1.0f),  // knife - light yellow
        ImVec4(0.60f, 0.80f, 0.96f, 1.0f),  // spoon - light steel blue
        ImVec4(0.96f, 0.40f, 0.40f, 1.0f),  // bowl - salmon
        ImVec4(0.40f, 0.96f, 0.20f, 1.0f),  // banana - spring green
        ImVec4(0.20f, 0.60f, 0.40f, 1.0f),  // apple - sea green
        ImVec4(0.96f, 0.80f, 0.60f, 1.0f),  // sandwich - burlywood
        ImVec4(0.60f, 0.40f, 0.20f, 1.0f),  // orange - saddle brown
        ImVec4(0.40f, 0.20f, 0.60f, 1.0f),  // broccoli - dark slate blue
        ImVec4(0.80f, 0.20f, 0.60f, 1.0f),  // carrot - medium violet red
        ImVec4(0.20f, 0.80f, 0.80f, 1.0f),  // hot dog - dark cyan
        ImVec4(0.96f, 0.20f, 0.96f, 1.0f),  // pizza - fuchsia
        ImVec4(0.60f, 0.60f, 0.20f, 1.0f),  // donut - dark khaki
        ImVec4(0.20f, 0.20f, 0.60f, 1.0f),  // cake - midnight blue
        ImVec4(0.80f, 0.40f, 0.80f, 1.0f),  // chair - medium orchid
        ImVec4(0.40f, 0.80f, 0.40f, 1.0f),  // couch - medium sea green
        ImVec4(0.80f, 0.80f, 0.40f, 1.0f),  // potted plant - dark khaki
        ImVec4(0.40f, 0.40f, 0.80f, 1.0f),  // bed - slate blue
        ImVec4(0.96f, 0.60f, 0.60f, 1.0f),  // dining table - light coral
        ImVec4(0.60f, 0.96f, 0.40f, 1.0f),  // toilet - green yellow
        ImVec4(0.40f, 0.60f, 0.96f, 1.0f),  // tv - cornflower blue
        ImVec4(0.96f, 0.40f, 0.96f, 1.0f),  // laptop - violet
        ImVec4(0.60f, 0.20f, 0.60f, 1.0f),  // mouse - dark orchid
        ImVec4(0.20f, 0.96f, 0.60f, 1.0f),  // remote - medium spring green
        ImVec4(0.96f, 0.96f, 0.40f, 1.0f),  // keyboard - yellow
        ImVec4(0.40f, 0.96f, 0.96f, 1.0f),  // cell phone - dark turquoise
        ImVec4(0.96f, 0.20f, 0.60f, 1.0f),  // microwave - deep pink
        ImVec4(0.60f, 0.40f, 0.40f, 1.0f),  // oven - rosy brown
        ImVec4(0.40f, 0.60f, 0.60f, 1.0f),  // toaster - cadet blue
        ImVec4(0.60f, 0.60f, 0.40f, 1.0f),  // sink - dark sea green
        ImVec4(0.96f, 0.80f, 0.80f, 1.0f),  // refrigerator - misty rose
        ImVec4(0.80f, 0.96f, 0.80f, 1.0f),  // book - honeydew
        ImVec4(0.80f, 0.80f, 0.96f, 1.0f),  // clock - lavender
        ImVec4(0.96f, 0.40f, 0.20f, 1.0f),  // vase - orangered
        ImVec4(0.20f, 0.40f, 0.20f, 1.0f),  // scissors - dark green
        ImVec4(0.96f, 0.60f, 0.96f, 1.0f),  // teddy bear - plum
        ImVec4(0.40f, 0.20f, 0.40f, 1.0f),  // hair drier - dark magenta
        ImVec4(0.20f, 0.60f, 0.80f, 1.0f),  // toothbrush - light sea green
    };

    detection_overlay::detection_overlay() = default;

    ImVec4 detection_overlay::get_class_color(int class_id)
    {
        if (class_id >= 0 && class_id < CLASS_COLORS.size())
        {
            return CLASS_COLORS[class_id];
        }
        return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);  // white fallback
    }

    void detection_overlay::draw_2d_overlay(const rect& stream_rect, const detection_result& result, ImFont* font)
    {
        if (result.instances.empty())
            return;

        // Calculate scale factors
        float scale_x = stream_rect.w / result.input_size.width;
        float scale_y = stream_rect.h / result.input_size.height;

        for (const auto& instance : result.instances)
        {
            ImVec4 color = get_class_color(instance.class_id);

            // Draw mask
            if (_show_mask && !instance.mask.empty())
            {
                draw_mask(stream_rect, instance);
            }

            // Draw bounding box
            if (_show_bbox)
            {
                draw_bbox(stream_rect, instance);
            }

            // Draw keypoints (pose)
            if (_show_keypoints && !instance.keypoints.empty())
            {
                draw_keypoints(stream_rect, instance);
            }

            // Draw label
            if (_show_confidence)
            {
                draw_label(stream_rect, instance);
            }
        }
    }

    void detection_overlay::draw_bbox(const rect& stream_rect, const detection_instance& instance)
    {
        ImVec4 color = get_class_color(instance.class_id);

        float x = stream_rect.x + instance.bbox.x * (stream_rect.w / (float)instance.bbox.width);
        float y = stream_rect.y + instance.bbox.y * (stream_rect.h / (float)instance.bbox.height);

        // Simpler scaling
        float scale_x = stream_rect.w;
        float scale_y = stream_rect.h;

        float x1 = stream_rect.x + (instance.bbox.x * scale_x);
        float y1 = stream_rect.y + (instance.bbox.y * scale_y);
        float x2 = stream_rect.x + ((instance.bbox.x + instance.bbox.width) * scale_x);
        float y2 = stream_rect.y + ((instance.bbox.y + instance.bbox.height) * scale_y);

        // Draw box
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRect(
            ImVec2(x1, y1), ImVec2(x2, y2),
            ImColor(color.x, color.y, color.z, 1.0f),
            0.0f, 0, _bbox_line_width);
    }

    void detection_overlay::draw_mask(const rect& stream_rect, const detection_instance& instance)
    {
        if (instance.mask.empty())
            return;

        ImVec4 color = get_class_color(instance.class_id);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        // Draw mask as colored overlay within bbox
        // This is a simplified version - for production, you'd want to render the actual mask polygon
        float x1 = stream_rect.x + instance.bbox.x;
        float y1 = stream_rect.y + instance.bbox.y;
        float x2 = stream_rect.x + instance.bbox.x + instance.bbox.width;
        float y2 = stream_rect.y + instance.bbox.y + instance.bbox.height;

        // Draw semi-transparent colored rectangle as mask placeholder
        draw_list->AddRectFilled(
            ImVec2(x1, y1), ImVec2(x2, y2),
            ImColor(color.x, color.y, color.z, _mask_alpha * 0.5f));
    }

    void detection_overlay::draw_keypoints(const rect& stream_rect, const detection_instance& instance)
    {
        if (instance.keypoints.empty())
            return;

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec4 color = get_class_color(instance.class_id);

        // Draw connections first
        for (const auto& conn : COCO_POSE_CONNECTIONS)
        {
            if (conn.first < instance.keypoints.size() && conn.second < instance.keypoints.size())
            {
                const auto& kp1 = instance.keypoints[conn.first];
                const auto& kp2 = instance.keypoints[conn.second];

                if (kp1.confidence > 0.3f && kp2.confidence > 0.3f)
                {
                    float x1 = stream_rect.x + kp1.x;
                    float y1 = stream_rect.y + kp1.y;
                    float x2 = stream_rect.x + kp2.x;
                    float y2 = stream_rect.y + kp2.y;

                    draw_list->AddLine(
                        ImVec2(x1, y1), ImVec2(x2, y2),
                        ImColor(color.x, color.y, color.z, 0.8f), 2.0f);
                }
            }
        }

        // Draw keypoints
        for (const auto& kp : instance.keypoints)
        {
            if (kp.confidence > 0.3f)
            {
                float x = stream_rect.x + kp.x;
                float y = stream_rect.y + kp.y;

                draw_list->AddCircleFilled(ImVec2(x, y), 4.0f, ImColor(1.0f, 1.0f, 1.0f, 1.0f));
                draw_list->AddCircle(ImVec2(x, y), 4.0f, ImColor(color.x, color.y, color.z, 1.0f), 0, 2.0f);
            }
        }
    }

    void detection_overlay::draw_label(const rect& stream_rect, const detection_instance& instance)
    {
        std::ostringstream ss;
        ss << instance.class_name << " " << std::fixed << std::setprecision(2) << instance.confidence;
        std::string label = ss.str();

        ImVec4 color = get_class_color(instance.class_id);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        float x = stream_rect.x + instance.bbox.x;
        float y = stream_rect.y + instance.bbox.y - 20;

        // Background for text
        ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
        draw_list->AddRectFilled(
            ImVec2(x, y), ImVec2(x + text_size.x + 8, y + text_size.y + 4),
            ImColor(color.x, color.y, color.z, 0.8f));

        // Text
        draw_list->AddText(ImVec2(x + 4, y + 2), ImColor(1.0f, 1.0f, 1.0f, 1.0f), label.c_str());
    }

    void detection_overlay::draw_3d_overlay(const detection_result& result)
    {
        // Draw 3D pose axes for detected objects with 3D positions
        if (!_show_3d_pose)
            return;

        for (const auto& instance : result.instances)
        {
            if (!instance.has_3d_position)
                continue;

            // Draw 3D axis at object position
            float axis_size = 0.1f;
            texture_buffer::draw_axes(axis_size, 2.0f);
        }
    }

    void detection_overlay::draw_legend(const detection_result& result)
    {
        if (result.instances.empty())
            return;

        ImGui::Begin("Detection Legend", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Detections: %d", (int)result.instances.size());
        ImGui::Text("Inference: %.1f ms", result.inference_time_ms);
        ImGui::Separator();

        // Show unique classes detected
        std::vector<int> unique_classes;
        for (const auto& inst : result.instances)
        {
            if (std::find(unique_classes.begin(), unique_classes.end(), inst.class_id) == unique_classes.end())
            {
                unique_classes.push_back(inst.class_id);
            }
        }

        for (int class_id : unique_classes)
        {
            ImVec4 color = get_class_color(class_id);
            std::string name = (class_id < COCO_CLASS_NAMES.size()) ? COCO_CLASS_NAMES[class_id] : "unknown";

            // Count instances
            int count = 0;
            for (const auto& inst : result.instances)
            {
                if (inst.class_id == class_id) count++;
            }

            ImGui::ColorButton(("##color_" + name).c_str(), color);
            ImGui::SameLine();
            ImGui::Text("%s: %d", name.c_str(), count);
        }

        ImGui::End();
    }

    void detection_overlay::draw_settings()
    {
        ImGui::Separator();
        ImGui::Text("Detection Display");
        ImGui::Checkbox("Show Bounding Boxes", &_show_bbox);
        ImGui::Checkbox("Show Masks", &_show_mask);
        ImGui::Checkbox("Show Confidence", &_show_confidence);
        ImGui::Checkbox("Show Keypoints", &_show_keypoints);
        ImGui::Checkbox("Show 3D Pose", &_show_3d_pose);
        ImGui::SliderFloat("Mask Alpha", &_mask_alpha, 0.0f, 1.0f);
        ImGui::SliderFloat("Box Line Width", &_bbox_line_width, 1.0f, 5.0f);
    }
}
