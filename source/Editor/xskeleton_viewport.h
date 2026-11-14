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
        bool                                m_bOutlineOnly = false;             // true: outlines without the translucent fills (and so without shadows)
    };

    struct skeleton_viewport
    {
        runtime                 m_Scene;                    // the skeleton editor's own scene (outline, fill, shadow); public: the editors wire the camera commands and toolbar to it, and set the subject's centre and radius
        const bool*             m_pOpen = nullptr;          // when set and false, the render callback draws nothing (the panel's window is closed)

        bool Init(xgpu::device& Device) noexcept { return m_Scene.Init(Device); }
        void Release() noexcept                  { m_Scene.Release(); }

        // The panel body. Call it where the viewport goes in the window; it takes the whole free area. Returns the screen rect it used (for overlays).
        void Render(std::span<const viewport_item> Items) noexcept
        {
            const ImVec2 Avail = ImGui::GetContentRegionAvail();
            const ImVec2 Min   = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(Min, ImVec2(Min.x + Avail.x, Min.y + Avail.y), IM_COL32(115, 115, 115, 255));        // the depth tint fades toward this
            ImGui::InvisibleButton("##SkeletonViewport", Avail, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            xeditor::PreviewGestures();
            m_Scene.HandleInput();
            m_Scene.UpdateView(Min, Avail.x, Avail.y);

            m_Lines.clear(); m_Fills.clear();
            const std::set<int> NoSelection; const std::vector<bool> NoTwist;
            for (const auto& It : Items)
            {
                if (!It.m_pSkeleton || It.m_Bones.size() != It.m_pSkeleton->getBones().size()) continue;
                m_Moved.assign(It.m_Bones.begin(), It.m_Bones.end());
                for (auto& B : m_Moved) B.m_Position = B.m_Position + It.m_Offset;
                auto Style = m_Scene.Style();
                if (It.m_NormalColor) Style.m_NormalColor = It.m_NormalColor;
                const auto& Sel = It.m_pSelected ? *It.m_pSelected : NoSelection;
                BuildWedgeGeometry(*It.m_pSkeleton, m_Moved, NoTwist, Style, Sel, m_Scene.m_Radius, It.m_ColorMode, It.m_MaskGroup, It.m_Hovered, m_Part, It.m_bOutlineOnly);
                m_Lines.insert(m_Lines.end(), m_Part.begin(), m_Part.end());
                if (!It.m_bOutlineOnly)
                {
                    BuildWedgeFillGeometry(*It.m_pSkeleton, m_Moved, NoTwist, Style, Sel, m_Scene.m_Radius, It.m_ColorMode, It.m_MaskGroup, It.m_Hovered, m_Part);
                    m_Fills.insert(m_Fills.end(), m_Part.begin(), m_Part.end());
                }
            }
            m_Scene.SetLines(m_Lines);
            m_Scene.SetFill(m_Fills);

            // The light's view of the fills is rendered before the frame's UI (it opens a render pass of its own on the window)
            auto* pHost   = xeditor::host::current();
            auto* pWindow = pHost ? pHost->find<xgpu::window>() : nullptr;
            if (pWindow) m_Scene.RenderShadow(*pWindow);

            xgpu::tools::imgui::AddCustomRenderCallback([this](xgpu::cmd_buffer& CmdBuffer, const ImVec2&, const ImVec2&)
            {
                if (!m_pOpen || *m_pOpen) m_Scene.DrawBones(CmdBuffer);
            });
            m_Scene.DrawOverlay();
        }

    private:
        std::vector<e19::draw_vert>     m_Lines, m_Fills, m_Part;
        std::vector<bone_world>         m_Moved;
    };
}

#endif
