#include "ViewportPanel.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <cmath>
#include <algorithm>
#include <cstdio>

EditorCamera ViewportPanel::camera;

// --- Math Helpers ---

// Robust Weak-Perspective Projection (No complex 4x4 matrices)
static ImVec2 ProjectPoint(Vec3 p, const EditorCamera& cam, ImVec2 canvasSize, bool isGizmo = false) {
    float cx = std::cos(cam.yaw);
    float sx = std::sin(cam.yaw);
    float cy = std::cos(cam.pitch);
    float sy = std::sin(cam.pitch);

    // 1. Camera transform
    float x = p.x;
    float y = p.y;
    float z = p.z;

    // If not a gizmo, offset by a target to orbit around it
    if (!isGizmo) {
        x -= cam.target.x;
        y -= cam.target.y;
        z -= cam.target.z;
    }

    // 2. Rotate Yaw (Y-axis)
    float tx = x * cx - z * sx;
    float tz = x * sx + z * cx;
    x = tx; z = tz;

    // 3. Rotate Pitch (X-axis)
    float ty = y * cy - z * sy;
    tz = y * sy + z * cy;
    y = ty; z = tz;

    // 4. Translate Camera Distance
    // For gizmo, we use a fixed distance so it doesn't zoom away
    float dist = isGizmo ? 5.0f : cam.distance;
    z -= dist;

    // 5. Perspective Projection
    // Simple fov calculation
    float fov = (canvasSize.y) * 1.5f; 
    
    // Clipping: If behind camera, return off-screen
    if (z >= -0.1f) return ImVec2(-10000.0f, -10000.0f);
    
    float scale = fov / -z;
    
    return ImVec2(
        (x * scale) + (canvasSize.x * 0.5f), 
        -(y * scale) + (canvasSize.y * 0.5f) 
    );
}

void ViewportPanel::draw(bool* p_open)
{
    // Remove padding so the viewport fills the entire window
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    
    if (ImGui::Begin("Viewport", p_open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 winPos = ImGui::GetCursorScreenPos();
        ImVec2 winSize = ImGui::GetContentRegionAvail();

        // 1. Input Handling
        if (ImGui::IsWindowHovered()) {
            // Zoom
            float wheel = ImGui::GetIO().MouseWheel;
            if (wheel != 0.0f) {
                camera.distance -= wheel * (camera.distance * 0.1f);
                if (camera.distance < 0.1f) camera.distance = 0.1f;
            }

            // Orbit (Right Click)
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                ImVec2 delta = ImGui::GetIO().MouseDelta;
                camera.yaw -= delta.x * 0.005f;
                camera.pitch += delta.y * 0.005f;
                // Clamp Pitch
                if (camera.pitch > 1.5f) camera.pitch = 1.5f;
                if (camera.pitch < -1.5f) camera.pitch = -1.5f;
            }

            // Pan (Middle Click)
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
                ImVec2 delta = ImGui::GetIO().MouseDelta;
                float panSpeed = camera.distance * 0.002f; // Scale pan speed with zoom
                float cx = cos(camera.yaw);
                float sx = sin(camera.yaw);
                
                // Pan relative to camera view
                camera.target.x -= (delta.x * cx) * panSpeed; // X
                camera.target.z -= (delta.x * sx) * panSpeed; // Z
                camera.target.y += delta.y * panSpeed;        // Y (Up/Down)
            }
        }

        // 2. Draw Background
        dl->AddRectFilled(winPos, ImVec2(winPos.x + winSize.x, winPos.y + winSize.y), IM_COL32(20, 20, 22, 255));

        // 3. Simple 3D Grid (Fixed Step)
        dl->PushClipRect(winPos, ImVec2(winPos.x + winSize.x, winPos.y + winSize.y), true);
        
        const int gridSize = 20; // Size of grid
        const float step = 1.0f; // 1 meter spacing
        ImU32 colGrid = IM_COL32(255, 255, 255, 30);
        ImU32 colAxisX = IM_COL32(200, 50, 50, 200);
        ImU32 colAxisZ = IM_COL32(50, 50, 200, 200);

        for (int i = -gridSize; i <= gridSize; ++i) {
            // Z-Lines (Running along X axis)
            Vec3 p1 = { (float)-gridSize * step, 0, (float)i * step };
            Vec3 p2 = { (float)gridSize * step, 0, (float)i * step };
            
            ImVec2 s = ProjectPoint(p1, camera, winSize);
            ImVec2 e = ProjectPoint(p2, camera, winSize);

            // Simple culling
            if (s.x > -5000 && e.x > -5000) {
                ImU32 col = (i == 0) ? colAxisX : colGrid;
                float thick = (i == 0) ? 2.0f : 1.0f;
                dl->AddLine(ImVec2(winPos.x + s.x, winPos.y + s.y), ImVec2(winPos.x + e.x, winPos.y + e.y), col, thick);
            }

            // X-Lines (Running along Z axis)
            p1 = { (float)i * step, 0, (float)-gridSize * step };
            p2 = { (float)i * step, 0, (float)gridSize * step };
            
            s = ProjectPoint(p1, camera, winSize);
            e = ProjectPoint(p2, camera, winSize);

            if (s.x > -5000 && e.x > -5000) {
                ImU32 col = (i == 0) ? colAxisZ : colGrid;
                float thick = (i == 0) ? 2.0f : 1.0f;
                dl->AddLine(ImVec2(winPos.x + s.x, winPos.y + s.y), ImVec2(winPos.x + e.x, winPos.y + e.y), col, thick);
            }
        }
        dl->PopClipRect();

        // 4. Toolbar (Restored Visibility)
        ImGui::SetCursorPos(ImVec2(20, 20));
        
        // Style adjustments just for toolbar
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.4f, 0.0f, 1.0f)); // Orange active

        ImGui::BeginGroup();
            if (ImGui::Button("SELECT", ImVec2(60, 30))) { /* Select */ }
            ImGui::SameLine(0, 5); // 5px spacing
            if (ImGui::Button("MOVE", ImVec2(60, 30))) { /* Move */ }
            ImGui::SameLine(0, 5);
            if (ImGui::Button("ROTATE", ImVec2(60, 30))) { /* Rotate */ }
            ImGui::SameLine(0, 5);
            if (ImGui::Button("SCALE", ImVec2(60, 30))) { /* Scale */ }
        ImGui::EndGroup();

        ImGui::PopStyleColor(3);
        ImGui::PopStyleVar();

        // 5. Gizmo (Bottom Right)
        ImVec2 gizmoPos = ImVec2(winPos.x + 50, winPos.y + winSize.y - 50);
        Vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1} };
        ImU32 colors[] = { IM_COL32(220,50,50,255), IM_COL32(50,220,50,255), IM_COL32(50,50,220,255) };
        char labels[] = {'X', 'Y', 'Z'};

        // Sort axes by depth (Z) to draw correct overlap
        struct AxisSort { int id; float z; };
        AxisSort depth[3];
        for(int i=0; i<3; i++) {
            ImVec2 p = ProjectPoint(axes[i], camera, {200, 200}, true); // Use small canvas for gizmo math
            depth[i] = {i, p.y}; // Rough depth est from Y projection 
        }
        
        // Simple bubble sort back-to-front
        if(depth[0].z < depth[1].z) std::swap(depth[0], depth[1]);
        if(depth[0].z < depth[2].z) std::swap(depth[0], depth[2]);
        if(depth[1].z < depth[2].z) std::swap(depth[1], depth[2]);

        // Draw Origin
        dl->AddCircleFilled(gizmoPos, 3.0f, IM_COL32(255,255,255,255));

        for(int i=2; i>=0; i--) {
            int idx = depth[i].id;
            ImVec2 end = ProjectPoint(axes[idx], camera, {100, 100}, true); // Projection for gizmo
            // Re-center gizmo projection locally
            end.x = (end.x - 50.0f) + gizmoPos.x;
            end.y = (end.y - 50.0f) + gizmoPos.y;
            
            dl->AddLine(gizmoPos, end, colors[idx], 3.0f);
            dl->AddText(ImVec2(end.x + 2, end.y + 2), colors[idx], &labels[idx], &labels[idx]+1);
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
}