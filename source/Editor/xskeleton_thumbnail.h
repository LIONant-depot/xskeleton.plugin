#ifndef XSKELETON_THUMBNAIL_H
#define XSKELETON_THUMBNAIL_H
#pragma once

// Skeleton's thumbnail renderer: the SAME wedge-gizmo runtime the interactive editor preview panel
// already uses (xskeleton_editor::runtime, xskeleton_editor_runtime.h - the octahedral bone shapes,
// outline, translucent fill and the shadow they cast, all reused as-is via BuildWedgeGeometry/
// BuildWedgeFillGeometry from xskeleton_editor_view.h, not reimplemented), just pointed at a fixed
// front-biased angle with an exact-fit camera distance instead of the panel's own orbit camera - see
// xgeom_static_thumbnail.h (the sibling this mirrors) and xeditor_thumbnail_camera_fit.h for why.
//
// Posed with the skeleton's own FROZEN ("zero") pose - forward kinematics through each bone's local
// rest transform, the same computation session::ComputePoses does for its own FROZEN half
// (xskeleton_editor.h), ported here rather than shared: it is a ~15 line loop over the compiled
// skeleton alone, no descriptor/log access a thumbnail renderer doesn't have. No selection, no hover,
// no per-mask-layer coloring and no twist-bone dimming (all of that needs the display-name table built
// from the compiler's log, which again a thumbnail has no access to) - every bone renders in
// NORMAL_RENDER's plain white/grey.
//
// Owns its own small offscreen colour+depth render target and render pass (built once in Init(), reused
// every call) and its own deferred GPU readback - see xeditor_thumbnail.h's contract comment for why.
#include "source/Tools/Editor/xeditor_thumbnail.h"
#include "source/Tools/Editor/xeditor_thumbnail_camera_fit.h"
#include "source/Tools/Editor/xeditor_resource_editor.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_runtime.h"

#include <limits>
#include <list>
#include <unordered_map>

namespace xskeleton_editor
{
    class thumbnail_renderer final : public xeditor::thumbnail_renderer
    {
    public:
        static constexpr float s_CellPixels = 128.0f;   // matches xeditor_thumbnail_cache::s_CellPixels

        bool Init(xgpu::device& Device) noexcept override
        {
            if (m_bReady) return true;
            if (!m_Runtime.Init(Device)) return false;

            const int Cell = static_cast<int>(s_CellPixels);
            if (!Ok(Device.Create(m_ColorTarget, { .m_Format = xgpu::texture::format::R8G8B8A8_UNORM, .m_Width = Cell, .m_Height = Cell, .m_isGamma = false }))) return false;
            if (!Ok(Device.Create(m_DepthTarget, { .m_Format = xgpu::texture::format::DEPTH_U16,      .m_Width = Cell, .m_Height = Cell, .m_isGamma = false }))) return false;
            {
                auto Attachments = std::array<xgpu::renderpass::attachment, 2>{ { m_ColorTarget, m_DepthTarget } };
                if (!Ok(Device.Create(m_Pass, { .m_Attachments = Attachments }))) return false;
            }

            m_bReady = true;
            return true;
        }

        // Polled once per tick (see xeditor_thumbnail.h's contract) until it returns true with OutBitmap
        // filled in. Serialized to one guid at a time, same as xgeom_static_thumbnail.h's identical
        // single-in-flight-target reasoning.
        bool Render(xgpu::device& Device, xgpu::window& Window, xresource::full_guid Guid, xbitmap& OutBitmap) noexcept override
        {
            if (!m_bReady) return false;

            if (m_bBusy)
            {
                if (m_BusyGuid != Guid) return false;
                if (!m_bReadbackDone) return false;
                m_bBusy = false;
                if (m_ReadbackWidth != static_cast<int>(s_CellPixels) || m_ReadbackHeight != static_cast<int>(s_CellPixels)) return false;

                OutBitmap.CreateBitmap(static_cast<int>(s_CellPixels), static_cast<int>(s_CellPixels));
                auto Dst = OutBitmap.getMip<xcolori>(0);
                std::memcpy(Dst.data(), m_ReadbackPixels.data(), Dst.size() * sizeof(xcolori));
                return true;
            }

            auto* pSkeleton = Reference(Guid);
            if (!pSkeleton) return false;

            xmath::fvec3 Min, Max;
            ComputeFrozenPose(*pSkeleton, m_World, Min, Max);

            m_Runtime.m_Center   = (Min + Max) * 0.5f;
            m_Runtime.m_Radius   = std::max(0.01f, (Max - Min).Length() * 0.5f);
            m_Runtime.m_bReframe = false;   // exact-fit distance below replaces the panel's own bounding-sphere auto-fit

            m_Runtime.m_View.setFov(20_xdeg);
            m_Runtime.m_View.setAspect(1.0f);
            m_Runtime.m_View.setViewport({ 0, 0, static_cast<int>(s_CellPixels), static_cast<int>(s_CellPixels) });   // ComputeTightFitDistance needs a real viewport BEFORE it runs (it calls View.LookAt/getW2C itself) - UpdateView below sets it again, redundant but harmless
            // Front-biased, not a full 3/4 corner view - same convention as xgeom_static_thumbnail.h's
            // own angle (see that file's comment for the full reasoning).
            m_Runtime.m_Angles.m_Pitch = -12_xdeg;
            m_Runtime.m_Angles.m_Yaw   =  15_xdeg;
            m_Runtime.m_Target         = m_Runtime.m_Center;
            m_Runtime.m_Distance       = xeditor::ComputeTightFitDistance(m_Runtime.m_View, m_Runtime.m_Angles, m_Runtime.m_Target, Min, Max);
            m_Runtime.UpdateView(ImVec2(0, 0), s_CellPixels, s_CellPixels);   // viewport/aspect + LookAt(Distance, Angles, Target) - m_bReframe is false, so no auto-fit override

            const wedge_style           Style{};   // defaults: NORMAL_RENDER's plain white/grey, no camera-distance dimming a thumbnail needs
            const std::vector<bool>     NoTwist(m_World.size(), false);
            const std::set<int>         NoSelection;
            BuildWedgeGeometry    (*pSkeleton, m_World, NoTwist, Style, NoSelection, m_Runtime.m_Radius, render_color_mode::NORMAL_RENDER, -1, -1, m_Outline);
            BuildWedgeFillGeometry(*pSkeleton, m_World, NoTwist, Style, NoSelection, m_Runtime.m_Radius, render_color_mode::NORMAL_RENDER, -1, -1, m_Fill);
            m_Runtime.SetLines(m_Outline);
            m_Runtime.SetFill(m_Fill);
            m_Runtime.RenderShadow(Window);

            {
                // cmd_buffer's own destructor ends the render pass - see xgeom_static_thumbnail.h's
                // identical comment on why this is scoped to end before the readback below.
                auto CmdBuffer = Window.StartRenderPass(m_Pass);
                m_Runtime.DrawBones(CmdBuffer);
            }

            (void)Window.ReadbackTexture(m_ColorTarget, m_ReadbackPixels, m_ReadbackWidth, m_ReadbackHeight, m_bReadbackDone);
            m_bBusy    = true;
            m_BusyGuid = Guid;
            return false;
        }

    private:
        static bool Ok(xgpu::device::error* pErr) noexcept
        {
            if (!pErr) return true;
            std::printf("Skeleton thumbnail: %s\n", std::string(xgpu::getErrorMsg(pErr)).c_str());
            return false;
        }

        // Ported from session::ComputePoses's FROZEN half (xskeleton_editor.h) - forward kinematics
        // through each bone's local rest transform - dropped down to just what a thumbnail needs: no
        // BIND pose, and a real min/max bbox (for the exact-fit camera) instead of session's own
        // average-position/max-radius framing (that one is built for an orbit camera that can end up
        // looking from any angle; a thumbnail's angle is fixed, so the tighter corner-projected bound
        // ComputeTightFitDistance gives is worth a real bbox instead).
        static void ComputeFrozenPose(const xskeleton::skeleton& Skeleton, std::vector<bone_world>& World, xmath::fvec3& Min, xmath::fvec3& Max) noexcept
        {
            const auto Bones = Skeleton.getBones();
            const auto Rests = Skeleton.getBoneRests();
            World.resize(Bones.size());
            std::vector<xmath::fmat4> Mats(Bones.size());

            Min = xmath::fvec3(std::numeric_limits<float>::max());
            Max = xmath::fvec3(std::numeric_limits<float>::lowest());
            for (std::size_t i = 0; i < Bones.size(); ++i)
            {
                const xmath::fmat4 Local = Rests[i].m_RestPose.toMatrix();
                Mats[i] = Bones[i].m_iParent < 0 ? Local : Mats[Bones[i].m_iParent] * Local;
                auto& W = World[i];
                W.m_Position = Mats[i].ExtractPosition(); W.m_Right = Mats[i].Right(); W.m_Up = Mats[i].Up();
                Min.m_X = std::min(Min.m_X, W.m_Position.m_X); Max.m_X = std::max(Max.m_X, W.m_Position.m_X);
                Min.m_Y = std::min(Min.m_Y, W.m_Position.m_Y); Max.m_Y = std::max(Max.m_Y, W.m_Position.m_Y);
                Min.m_Z = std::min(Min.m_Z, W.m_Position.m_Z); Max.m_Z = std::max(Max.m_Z, W.m_Position.m_Z);
            }
            if (Bones.empty()) { Min = Max = xmath::fvec3(0); }
        }

        // Keeps the skeleton loaded for a little while after its last thumbnail request - mirrors
        // xgeom_static_thumbnail.h's own Reference()/LRU exactly, for the same reason (a render
        // recorded now is only consumed, and read back, by the GPU several frames later). No secondary
        // ref to chain here (unlike GeomSkin's thumbnail) - a skeleton owns no GPU buffers and
        // references nothing else itself.
        xskeleton::skeleton* Reference(const xresource::full_guid& Guid) noexcept
        {
            if (auto It = m_Refs.find(Guid); It != m_Refs.end())
            {
                m_Order.remove(Guid);
                m_Order.push_front(Guid);
                return xresource::g_Mgr.getResource(It->second);
            }

            xrsc::skeleton Ref;
            Ref.m_Instance = Guid.m_Instance;
            auto& Held = m_Refs.emplace(Guid, Ref).first->second;
            m_Order.push_front(Guid);
            while (m_Refs.size() > m_Capacity)
            {
                const auto Oldest = m_Order.back();
                if (auto It = m_Refs.find(Oldest); It != m_Refs.end()) { xresource::g_Mgr.ReleaseRef(It->second); m_Refs.erase(It); }
                m_Order.pop_back();
            }
            return xresource::g_Mgr.getResource(Held);
        }

        runtime                                                  m_Runtime;
        bool                                                      m_bReady = false;
        std::vector<bone_world>                                  m_World;
        std::vector<e19::draw_vert>                              m_Outline, m_Fill;

        xgpu::texture                                            m_ColorTarget;
        xgpu::texture                                            m_DepthTarget;
        xgpu::renderpass                                         m_Pass;
        bool                                                     m_bBusy          = false;
        xresource::full_guid                                     m_BusyGuid       {};
        std::vector<std::uint32_t>                               m_ReadbackPixels;
        int                                                      m_ReadbackWidth  = 0;
        int                                                      m_ReadbackHeight = 0;
        bool                                                     m_bReadbackDone  = false;

        std::size_t                                              m_Capacity = 20;
        std::unordered_map<xresource::full_guid, xrsc::skeleton> m_Refs;
        std::list<xresource::full_guid>                          m_Order;
    };
    // Registered next to g_Registration in xskeleton_editor.h (this header only defines the class).
}

#endif // XSKELETON_THUMBNAIL_H
