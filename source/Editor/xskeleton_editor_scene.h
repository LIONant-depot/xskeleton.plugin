#ifndef XSKELETON_EDITOR_SCENE_H
#define XSKELETON_EDITOR_SCENE_H
#pragma once

// The 3D scene the skeleton, animation and skin editors share: an orbit+fly camera and the ground grid
// (both now xeditor_tools::camera/xeditor_tools::grid - shared with every other 3D editor, not just
// these three), and the bones as line geometry that is rebuilt every frame (the pose and the camera
// change what it looks like) - that part stays here, since bone-wedge lines are skeleton-specific, not
// a generic editor-viewport concern. The editor builds the vertices with xskeleton_editor_view.h and
// hands them over; Draw runs from the panel's render callback, inside the window's own pass.
//
// scene inherits BOTH xeditor_tools classes (not composition) so every existing external call site
// (m_View, m_Angles, m_Distance, m_ShadowMap, m_White, HandleInput(), UpdateView(), ...) keeps working
// unchanged across xskeleton_editor.h, xgeom_skin_editor_preview.h, and both the Skeleton and GeomSkin
// thumbnail renderers - none of them needed to change for this refactor. Init/Release/DrawGrid are
// scene's OWN methods (same names as before), explicitly forwarding to the inherited grid's Init/
// Release/Draw after also handling scene's own line-drawing setup - this is deliberate name-hiding, not
// an override: the base grid::Init/Release exist as building blocks scene calls into, not entry points
// external code should reach past scene to call directly.
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_view.h"
#include "plugins/xskeleton.plugin/source/xskeleton_xgpu_rsc_loader.h"
#include "plugins/xskeleton.plugin/source/xskeleton_xgpu_rsc_loader.cpp"       // the resource loader: compiled once, in the host's translation unit
#include "source/Tools/Editor/xeditor_resource_editor.h"
#include "source/Tools/Editor/xeditor_camera.h"
#include "source/tools/xgpu_imgui_breach.h"
#include "source/tools/xgpu_view.h"
#include "source/tools/xgpu_xcore_bitmap_helpers.h"
#include "source/tools/editors/xgpu_editor_anim_pose.h"
#include "dependencies/xeditor_tools/src/xeditor_tools_camera.h"
#include "dependencies/xeditor_tools/src/xeditor_tools_grid.h"
#include "dependencies/imgui/imgui.h"

#include <cstring>

namespace xskeleton_editor
{
    // The bone lines' own shader - the grid's own shader now lives in xeditor_tools alongside the grid itself.
    inline constexpr std::uint32_t g_LineVertShader[] =
    {
        #include "draw_vert.h"
    };
    inline constexpr std::uint32_t g_LineFragShader[] =
    {
        #include "draw_frag.h"
    };

    struct line_push_constants
    {
        xmath::fmat4    m_L2C;
    };

    struct scene : xeditor_tools::camera, xeditor_tools::grid
    {
        // What is drawn
        std::vector<e19::draw_vert> m_Verts;
        std::size_t                 m_nDrawn = 0;

        xgpu::vertex_descriptor     m_LineVD;
        xgpu::buffer                m_LineIndices, m_LineVerts;
        xgpu::pipeline              m_LinePipeline;
        xgpu::pipeline_instance     m_LineInstance;

        // world space to the light's clip space; zero: no shadow, the grid is always lit - set by
        // whichever derived "runtime" class's own RenderShadow() fits the light to the subject and
        // casts it, every frame before Draw() runs.
        xmath::fmat4                m_ShadowL2C = xmath::fmat4::fromZero();

        bool                        m_bReady = false;

        // The camera's numbers, for the camera commands
        xeditor::camera_access Camera() noexcept { return { &m_Angles, &m_Distance, &m_Target, [this] { m_bReframe = true; } }; }

        static bool Ok(xgpu::device::error* pErr) noexcept
        {
            if (!pErr) return true;
            std::printf("Skeleton scene: %s\n", std::string(xgpu::getErrorMsg(pErr)).c_str());
            return false;
        }

        bool Init(xgpu::device& Device, bool bShadow = false) noexcept
        {
            if (m_bReady) return true;
            m_View.setFov(60_xdeg);
            m_View.setNearZ(0.01f);        // a zero near/far (the default) clips the 100-unit grid
            m_View.setFarZ(10000.0f);

            if (!xeditor_tools::grid::Init(Device, bShadow)) return false;

            auto Attributes = std::array
            { xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_X),     .m_Format = xgpu::vertex_descriptor::format::FLOAT_3D }
            , xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_U),     .m_Format = xgpu::vertex_descriptor::format::FLOAT_2D }
            , xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_Color), .m_Format = xgpu::vertex_descriptor::format::UINT8_4D_NORMALIZED }
            };
            if (!Ok(Device.Create(m_LineVD, xgpu::vertex_descriptor::setup{ .m_Topology = xgpu::vertex_descriptor::topology::LINE_LIST, .m_VertexSize = sizeof(e19::draw_vert), .m_Attributes = Attributes }))) return false;

            {
                xgpu::shader Vert, Frag;
                auto Shader = [&](xgpu::shader& Out, xgpu::shader::type::bit Type, const std::uint32_t* pCode, std::size_t nWords)
                {
                    return Ok(Device.Create(Out, { .m_Type = Type, .m_Sharer = xgpu::shader::setup::raw_data{ std::span{ (std::int32_t*)pCode, nWords } } }));
                };
                if (!Shader(Vert, xgpu::shader::type::bit::VERTEX,   g_LineVertShader, std::size(g_LineVertShader))) return false;
                if (!Shader(Frag, xgpu::shader::type::bit::FRAGMENT, g_LineFragShader, std::size(g_LineFragShader))) return false;

                auto Samplers = std::array{ xgpu::pipeline::sampler{} };
                auto Shaders  = std::array<const xgpu::shader*, 2>{ &Frag, &Vert };
                if (!Ok(Device.Create(m_LinePipeline, xgpu::pipeline::setup{ .m_VertexDescriptor = m_LineVD, .m_Shaders = Shaders, .m_PushConstantsSize = sizeof(line_push_constants), .m_Samplers = Samplers }))) return false;
                auto Bindings = std::array{ xgpu::pipeline_instance::sampler_binding{ m_White } };
                if (!Ok(Device.Create(m_LineInstance, { .m_PipeLine = m_LinePipeline, .m_SamplersBindings = Bindings }))) return false;
            }

            // The index buffer is an identity ramp: only the vertices change from frame to frame
            if (!Ok(Device.Create(m_LineIndices, { .m_Type = xgpu::buffer::type::INDEX, .m_EntryByteSize = sizeof(std::uint32_t), .m_EntryCount = g_MaxWedgeVertices }))) return false;
            (void)m_LineIndices.MemoryMap(0, g_MaxWedgeVertices, [&](void* pData)
            {
                auto* pIndex = static_cast<std::uint32_t*>(pData);
                for (int i = 0; i < g_MaxWedgeVertices; ++i) pIndex[i] = static_cast<std::uint32_t>(i);
            });
            if (!Ok(Device.Create(m_LineVerts, { .m_Type = xgpu::buffer::type::VERTEX, .m_Usage = xgpu::buffer::setup::usage::CPU_WRITE_GPU_READ, .m_EntryByteSize = sizeof(e19::draw_vert), .m_EntryCount = g_MaxWedgeVertices }))) return false;

            m_bReady = true;
            return true;
        }

        // The line pipeline goes back to the device (a dropped one crashes when the device empties its
        // queue); buffers drop by themselves. The grid's own GPU objects go back via its own Release().
        void Release() noexcept
        {
            xeditor::DestroyGpu(m_pDevice, m_LineInstance, m_LinePipeline);
            xeditor_tools::grid::Release();
        }

        // How the depth tint of the bones sees this camera
        wedge_style Style() const noexcept
        {
            wedge_style Style;
            Style.m_CameraPos = m_View.getPosition();
            Style.m_NearDepth = std::max(0.01f, m_Distance * 0.25f);
            Style.m_FarDepth  = m_Distance * 1.6f + m_Radius;
            return Style;
        }

        void SetLines(const std::vector<e19::draw_vert>& Verts) noexcept
        {
            m_nDrawn = std::min<std::size_t>(Verts.size(), g_MaxWedgeVertices);
            if (m_nDrawn == 0) return;
            (void)m_LineVerts.MemoryMap(0, static_cast<int>(m_nDrawn), [&](void* pData) { std::memcpy(pData, Verts.data(), m_nDrawn * sizeof(e19::draw_vert)); });
        }

        // The ground grid, fixed at the world origin so it always shows where zero is. From the panel's render callback.
        void DrawGrid(xgpu::cmd_buffer& CmdBuffer) noexcept
        {
            if (!m_bReady) return;
            xeditor_tools::grid::Draw(CmdBuffer, m_View.getW2C(), m_View.getPosition(), m_ShadowL2C);
        }

        // The bone lines
        void DrawLines(xgpu::cmd_buffer& CmdBuffer) noexcept
        {
            if (!m_bReady) return;
            const xmath::fmat4 W2C = m_View.getW2C();
            if (m_nDrawn)
            {
                CmdBuffer.setPipelineInstance(m_LineInstance);
                CmdBuffer.setBuffer(m_LineIndices);
                CmdBuffer.setBuffer(m_LineVerts);
                CmdBuffer.setPushConstants(line_push_constants{ .m_L2C = W2C });
                CmdBuffer.Draw(static_cast<int>(m_nDrawn));
            }
        }

        void Draw(xgpu::cmd_buffer& CmdBuffer) noexcept { DrawGrid(CmdBuffer); DrawLines(CmdBuffer); }
    };
}

#endif // XSKELETON_EDITOR_SCENE_H
