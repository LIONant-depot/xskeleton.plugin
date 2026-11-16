#ifndef XSKELETON_VIEWPORT_H
#define XSKELETON_VIEWPORT_H
#pragma once

// One 3D panel that draws skeletons EXACTLY the way the skeleton editor does (its `runtime`: the wedge bone gizmo with its outline and its translucent fill, the root sphere, the depth tint, the shadow the fills cast on the grid), for any editor that needs
// to show one or several: the animation editor shows one posed by a clip, the retarget editor shows a source and a destination side by side.
// The panel owns the scene (camera, grid, line pipeline); the editor owns the data and hands in, each frame, what to draw:
//     m_Viewport.Render(std::array{ viewport_item{ .m_pSkeleton = &Skel, .m_Bones = PoseBones } });
// An item is a skeleton plus its bone worlds (the editor evaluated them: rest, a clip's pose, a retargeted pose), an offset to place it in the world,
// an optional base colour (to tell two skeletons apart), a selection and a hovered bone. Nothing here knows clips, retargeting or descriptors.
// Picking and the selection UI stay with the editors (the skeleton editor has intents, masks and LOD colouring on top of this).
#include "xskeleton_editor_scene.h"
#include "xskeleton_editor_runtime.h"
#include "xskeleton_editor_view.h"
#include "source/Tools/Editor/xeditor_camera_toolbar.h"

#include <set>
#include <span>
#include <vector>

namespace xskeleton_editor
{
    struct viewport_item
    {
        const xskeleton::skeleton*          m_pSkeleton   = nullptr;
        std::span<const bone_world>         m_Bones;                            // one per bone of m_pSkeleton, in the space the editor evaluated
        xmath::fvec3                        m_Offset      = {};                 // added to every bone position: places a second skeleton beside the first
        std::uint32_t                       m_NormalColor = 0;                  // 0: the style's own; else the base colour of this skeleton's bones
        const std::set<int>*                m_pSelected   = nullptr;            // selected bones (null: none)
        int                                 m_Hovered     = -1;
        render_color_mode                   m_ColorMode   = render_color_mode::NORMAL_RENDER;
        int                                 m_MaskGroup   = -1;
        bool                                m_bLeadsToSelected = true;          // false: a selected bone lights only its own wedge, not the one leading to it
        bool                                m_bOutlineOnly = false;             // true: outlines without the translucent fills (and so without shadows)
    };

    struct skeleton_viewport
    {
        bool                    m_bHoverRect = false, m_bHoverWin = false; ImVec2 m_DbgMin = {}, m_DbgSize = {};
        bool                    m_bGizmoHot = false, m_bGizmoDrag = false;      // set by an editor that runs a transform gizmo over the view, before Render
        runtime                 m_Scene;                    // the skeleton editor's own scene (outline, fill, shadow); public: the editors wire the camera commands and toolbar to it, and set the subject's centre and radius
        const bool*             m_pOpen = nullptr;          // when set and false, the render callback draws nothing (the panel's window is closed)
        std::vector<e19::draw_vert> m_ExtraLines;           // world-space line pairs an editor adds (a ragdoll's shapes over the bones): drawn with the bones, then cleared

        bool Init(xgpu::device& Device) noexcept { return m_Scene.Init(Device); }

        // The mouse is over the panel (valid after Render)
        bool Over(void) const noexcept { return m_bOver; }

        // The bone of an item under a screen point: what is drawn is what picks (the skeleton editor's own PickWedge). T is the distance along the ray: of two items, the smaller wins.
        bool PickBone(const viewport_item& It, const ImVec2& Mouse, int& iBone, float& T) noexcept
        {
            iBone = -1; T = 0;
            if (!It.m_pSkeleton || It.m_Bones.size() != It.m_pSkeleton->getBones().size()) return false;
            std::vector<bone_world> Moved(It.m_Bones.begin(), It.m_Bones.end());
            for (auto& B : Moved) B.m_Position = B.m_Position + It.m_Offset;
            const auto Dir = m_Scene.m_View.RayFromScreen(Mouse.x, Mouse.y);
            PickWedge(*It.m_pSkeleton, Moved, m_Scene.m_View.getPosition(), Dir, m_Scene.m_Radius, iBone, T);
            return iBone >= 0;
        }

        // A world position to the screen of this panel (for what an editor draws over it: links between bones, labels). False behind the camera. Valid after Render.
        ImVec2 RectMin()  const noexcept { return m_Min; }       // where the view drew last (a gizmo over it needs the rectangle)
        ImVec2 RectSize() const noexcept { return m_Size; }
        bool Project(const xmath::fvec3& P, ImVec2& Out) const noexcept
        {
            const xmath::fvec4 Clip = m_Scene.m_View.getW2C() * xmath::fvec4(P, 1.0f);
            if (Clip.m_W <= 1.0e-4f) return false;
            Out = ImVec2(m_Min.x + (Clip.m_X / Clip.m_W * 0.5f + 0.5f) * m_Size.x, m_Min.y + (Clip.m_Y / Clip.m_W * 0.5f + 0.5f) * m_Size.y);      // (the projection already flips Y for Vulkan: the skeleton editor's labels do the same)
            return true;
        }
        void Release() noexcept                  { m_Scene.Release(); }

        // The panel body. Call it where the viewport goes in the window; it takes the whole free area. Returns the screen rect it used (for overlays).
        void Render(std::span<const viewport_item> Items) noexcept
        {
            const ImVec2 Avail = ImGui::GetContentRegionAvail();
            const ImVec2 Min   = ImGui::GetCursorScreenPos();
            m_Min = Min; m_Size = Avail;
            m_DbgMin = Min; m_DbgSize = Avail; m_bHoverRect = ImGui::IsMouseHoveringRect(Min, ImVec2(Min.x + Avail.x, Min.y + Avail.y)); m_bHoverWin = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);      // (also while a button is held: a drag onto a bone still sees the bone)
            m_bOver = m_bHoverRect && m_bHoverWin;
            ImGui::GetWindowDrawList()->AddRectFilled(Min, ImVec2(Min.x + Avail.x, Min.y + Avail.y), IM_COL32(115, 115, 115, 255));        // the depth tint fades toward this
            if (m_bGizmoHot) ImGui::Dummy(Avail);          // a transform gizmo is hovered or dragged: it needs the hover (the keys and the camera still work; a drag keeps them out)
            else ImGui::InvisibleButton("##SkeletonViewport", Avail, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            if (!m_bGizmoDrag)
            {
                xeditor::PreviewGestures();
                m_Scene.HandleInput();
            }
            m_Scene.UpdateView(Min, Avail.x, Avail.y);

            m_Lines.clear(); m_Fills.clear(); m_Chunks.clear();
            const std::set<int> NoSelection; const std::vector<bool> NoTwist;
            xmath::fvec3 BMin( 1e30f,  1e30f,  1e30f), BMax(-1e30f, -1e30f, -1e30f);     // where the bones are this frame: the shadow's light is fitted to it
            for (const auto& It : Items)
            {
                if (!It.m_pSkeleton || It.m_Bones.size() != It.m_pSkeleton->getBones().size()) continue;
                m_Moved.assign(It.m_Bones.begin(), It.m_Bones.end());
                for (auto& B : m_Moved)
                {
                    B.m_Position = B.m_Position + It.m_Offset;
                    BMin = xmath::fvec3(std::min(BMin.m_X, B.m_Position.m_X), std::min(BMin.m_Y, B.m_Position.m_Y), std::min(BMin.m_Z, B.m_Position.m_Z));
                    BMax = xmath::fvec3(std::max(BMax.m_X, B.m_Position.m_X), std::max(BMax.m_Y, B.m_Position.m_Y), std::max(BMax.m_Z, B.m_Position.m_Z));
                }
                auto Style = m_Scene.Style();
                Style.m_bLeadsToSelected = It.m_bLeadsToSelected;
                if (It.m_NormalColor) Style.m_NormalColor = It.m_NormalColor;
                const auto& Sel = It.m_pSelected ? *It.m_pSelected : NoSelection;
                BuildWedgeGeometry(*It.m_pSkeleton, m_Moved, NoTwist, Style, Sel, m_Scene.m_Radius, It.m_ColorMode, It.m_MaskGroup, It.m_Hovered, m_Part, It.m_bOutlineOnly);
                m_Lines.insert(m_Lines.end(), m_Part.begin(), m_Part.end());
                if (!It.m_bOutlineOnly)
                {
                    BuildWedgeFillGeometry(*It.m_pSkeleton, m_Moved, NoTwist, Style, Sel, m_Scene.m_Radius, It.m_ColorMode, It.m_MaskGroup, It.m_Hovered, m_Part, &m_PartChunks);
                    for (auto C : m_PartChunks) { C.m_First += m_Fills.size(); m_Chunks.push_back(C); }
                    m_Fills.insert(m_Fills.end(), m_Part.begin(), m_Part.end());
                }
            }
            if (Items.size() > 1)       // several skeletons: their objects merged far to near, so one skeleton's bone in front of another's blends over it
            {
                std::stable_sort(m_Chunks.begin(), m_Chunks.end(), [](const fill_chunk& A, const fill_chunk& B) { return A.m_CameraDist > B.m_CameraDist; });
                m_Sorted.clear(); m_Sorted.reserve(m_Fills.size());
                for (const auto& C : m_Chunks) m_Sorted.insert(m_Sorted.end(), m_Fills.begin() + C.m_First, m_Fills.begin() + C.m_First + C.m_Count);
                m_Fills.swap(m_Sorted);
            }
            m_Lines.insert(m_Lines.end(), m_ExtraLines.begin(), m_ExtraLines.end()); m_ExtraLines.clear();
            m_Scene.SetLines(m_Lines);
            m_Scene.SetFill(m_Fills);

            // The light's view of the fills is rendered before the frame's UI (it opens a render pass of its own on the window)
            auto* pHost   = xeditor::host::current();
            auto* pWindow = pHost ? pHost->find<xgpu::window>() : nullptr;
            if (pWindow)
            {
                if (BMin.m_X <= BMax.m_X)
                {
                    BMin.m_Y = std::min(BMin.m_Y, 0.0f);        // down to the ground the shadow falls on
                    const xmath::fvec3 C = (BMin + BMax) * 0.5f;
                    const float R = std::max((BMax - BMin).Length() * 0.5f * 1.1f, 0.05f);
                    m_Scene.RenderShadow(*pWindow, &C, R);
                }
                else m_Scene.RenderShadow(*pWindow);
            }

            xgpu::tools::imgui::AddCustomRenderCallback([this](xgpu::cmd_buffer& CmdBuffer, const ImVec2&, const ImVec2&)
            {
                if (!m_pOpen || *m_pOpen) m_Scene.DrawBones(CmdBuffer);
            });
            m_Scene.DrawOverlay();
        }

    private:
        ImVec2                          m_Min = {}, m_Size = {};
        bool                            m_bOver = false;
        std::vector<e19::draw_vert>     m_Lines, m_Fills, m_Part, m_Sorted;
        std::vector<fill_chunk>         m_Chunks, m_PartChunks;
        std::vector<bone_world>         m_Moved;
    };
}

#endif
