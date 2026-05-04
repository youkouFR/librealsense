// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#pragma once

#include "ai-inference.h"
#include "rendering.h"
#include <imgui.h>
#include <vector>

namespace rs2
{
    class detection_overlay
    {
    public:
        detection_overlay();

        // Draw 2D detection overlay on top of RGB stream
        void draw_2d_overlay(const rect& stream_rect, const detection_result& result, ImFont* font);

        // Draw 3D pose axes in pointcloud view
        void draw_3d_overlay(const detection_result& result);

        // Draw legend panel
        void draw_legend(const detection_result& result);

        // Settings UI
        void draw_settings();

        // Configuration
        void set_show_bbox(bool show) { _show_bbox = show; }
        void set_show_mask(bool show) { _show_mask = show; }
        void set_show_confidence(bool show) { _show_confidence = show; }
        void set_show_keypoints(bool show) { _show_keypoints = show; }
        void set_show_3d_pose(bool show) { _show_3d_pose = show; }
        void set_mask_alpha(float alpha) { _mask_alpha = alpha; }
        void set_bbox_line_width(float width) { _bbox_line_width = width; }

        bool show_bbox() const { return _show_bbox; }
        bool show_mask() const { return _show_mask; }
        bool show_confidence() const { return _show_confidence; }
        bool show_keypoints() const { return _show_keypoints; }
        bool show_3d_pose() const { return _show_3d_pose; }
        float mask_alpha() const { return _mask_alpha; }
        float bbox_line_width() const { return _bbox_line_width; }

    private:
        ImVec4 get_class_color(int class_id);
        void draw_bbox(const rect& stream_rect, const detection_instance& instance);
        void draw_mask(const rect& stream_rect, const detection_instance& instance);
        void draw_keypoints(const rect& stream_rect, const detection_instance& instance);
        void draw_label(const rect& stream_rect, const detection_instance& instance);

        // Class colors (COCO palette)
        static const std::vector<ImVec4> CLASS_COLORS;

        // Display options
        bool _show_bbox = true;
        bool _show_mask = true;
        bool _show_confidence = true;
        bool _show_keypoints = true;
        bool _show_3d_pose = true;
        float _mask_alpha = 0.5f;
        float _bbox_line_width = 2.0f;
    };
}
