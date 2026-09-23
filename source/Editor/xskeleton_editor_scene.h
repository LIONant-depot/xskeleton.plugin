#ifndef XSKELETON_EDITOR_SCENE_H
#define XSKELETON_EDITOR_SCENE_H
#pragma once

// The 3D scene the skeleton, animation and skin editors share: an orbit camera (right drag turns, middle drag pans, the wheel zooms), the ground
// grid, and the bones as line geometry that is rebuilt every frame (the pose and the camera change what it looks like). The editor builds the
// vertices with xskeleton_editor_view.h and hands them over; Draw runs from the panel's render callback, inside the window's own pass.
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_view.h"
#include "plugins/xskeleton.plugin/source/xskeleton_xgpu_rsc_loader.h"
#include "plugins/xskeleton.plugin/source/xskeleton_xgpu_rsc_loader.cpp"       // the resource loader: compiled once, in the host's translation unit
#include "source/Tools/Editor/xeditor_resource_editor.h"
#include "source/Tools/Editor/xeditor_camera.h"
#include "source/tools/xgpu_imgui_breach.h"
#include "source/tools/xgpu_view.h"
#include "source/tools/xgpu_xcore_bitmap_helpers.h"
#include "source/tools/editors/xgpu_editor_viewport.h"
#include "source/tools/editors/xgpu_editor_anim_pose.h"
#include "dependencies/imgui/imgui.h"

#include <cstring>

namespace xskeleton_editor
{
    // The grid and the bone lines reuse shaders every editor already compiles (draw_vert/draw_frag, and E21's grid pair)
    inline constexpr std::uint32_t g_LineVertShader[] =
    {
        #include "draw_vert.h"
    };
    inline constexpr std::uint32_t g_LineFragShader[] =
    {
        #include "draw_frag.h"
    };
    inline constexpr std::uint32_t g_GridVertShader[] =
    {
        #include "E21_GridShader_vert.h"
    };
    inline constexpr std::uint32_t g_GridFragShader[] =
    {
        #include "E21_GridShader_frag.h"
    };

    struct line_push_constants
    {
        xmath::fmat4    m_L2C;
    };

    // The grid shader's uniform block is too large for push constants, so it is a real (dynamic) UBO. There is no shadow pass in these
    // viewers: the shadow matrix is zero, which the shader reads as "always lit".
    struct alignas(256) grid_uniform
    {
        xmath::fmat4    m_L2W;
        xmath::fmat4    m_W2C;
        xmath::fmat4    m_L2CTShadow;
        xmath::fvec3    m_WorldSpaceCameraPos = xmath::fvec3(0.0f, 10.0f, 0.0f);
        float           m_MajorGridDiv = 10.0f;
    };

    struct scene
    {
        xgpu::device*               m_pDevice = nullptr;
        bool                        m_bReady  = false;

        // Camera
        xgpu::tools::view           m_View;
        xmath::radian3              m_Angles;
        float                       m_Distance = -1;                   // -1 until the first frame framed the subject
        xmath::fvec3                m_Target   = xmath::fvec3(0, 0, 0);
        bool                        m_bReframe = true;
        float                       m_Radius   = 1.0f;                 // the subject: what the camera frames, and what sizes the near/far depth range
        xmath::fvec3                m_Center   = xmath::fvec3(0, 0, 0);

        // What is drawn
        std::vector<e19::draw_vert> m_Verts;
        std::size_t                 m_nDrawn = 0;

        xgpu::vertex_descriptor     m_GridVD, m_LineVD;
        xgpu::buffer                m_GridUBO, m_LineIndices, m_LineVerts;
        xgpu::pipeline              m_GridPipeline, m_LinePipeline;
        xgpu::pipeline_instance     m_GridInstance, m_LineInstance;
        xgpu::texture               m_White;
        e19::mesh_manager           m_Meshes;

        // Optional: a depth map the subject is drawn into from the light (the editor does that, before the frame's UI), which the grid receives
        xgpu::texture               m_ShadowMap;
        xgpu::renderpass            m_ShadowPass;
        xmath::fmat4                m_ShadowL2C = xmath::fmat4::fromZero();     // world space to the light's clip space; zero: no shadow, the grid is always lit
        bool                        m_bShadow   = false;

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
            m_pDevice = &Device;
            m_bShadow = bShadow;
            m_View.setFov(60_xdeg);
            m_View.setNearZ(0.01f);        // a zero near/far (the default) clips the 100-unit grid
            m_View.setFarZ(10000.0f);
            m_Meshes.Init(Device);      // only its ground plane is used

            if (auto* pErr = xgpu::tools::bitmap::Create(m_White, Device, xbitmap::getDefaultBitmap()); pErr) return false;

            auto Attributes = std::array
            { xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_X),     .m_Format = xgpu::vertex_descriptor::format::FLOAT_3D }
            , xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_U),     .m_Format = xgpu::vertex_descriptor::format::FLOAT_2D }
            , xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_Color), .m_Format = xgpu::vertex_descriptor::format::UINT8_4D_NORMALIZED }
            };
            if (!Ok(Device.Create(m_GridVD, xgpu::vertex_descriptor::setup{ .m_VertexSize = sizeof(e19::draw_vert), .m_Attributes = Attributes }))) return false;
            if (!Ok(Device.Create(m_LineVD, xgpu::vertex_descriptor::setup{ .m_Topology = xgpu::vertex_descriptor::topology::LINE_LIST, .m_VertexSize = sizeof(e19::draw_vert), .m_Attributes = Attributes }))) return false;

            if (!Ok(Device.Create(m_GridUBO, { .m_Type = xgpu::buffer::type::UNIFORM, .m_Usage = xgpu::buffer::setup::usage::CPU_WRITE_GPU_READ, .m_EntryByteSize = sizeof(grid_uniform), .m_EntryCount = 10 }))) return false;

            auto Shader = [&](xgpu::shader& Out, xgpu::shader::type::bit Type, const std::uint32_t* pCode, std::size_t nWords)
            {
                return Ok(Device.Create(Out, { .m_Type = Type, .m_Sharer = xgpu::shader::setup::raw_data{ std::span{ (std::int32_t*)pCode, nWords } } }));
            };

            {
                xgpu::shader Vert, Frag;
                if (!Shader(Vert, xgpu::shader::type::bit::VERTEX,   g_GridVertShader, std::size(g_GridVertShader))) return false;
                if (!Shader(Frag, xgpu::shader::type::bit::FRAGMENT, g_GridFragShader, std::size(g_GridFragShader))) return false;

                auto Binds    = std::array{ xgpu::pipeline::uniform_binds{ .m_BindIndex = 0, .m_Usage = { .m_bVertex = true, .m_bFragment = true }, .m_Type = xgpu::pipeline::uniform_binds::type::UBO_DYNAMIC } };
                auto Samplers = std::array{ xgpu::pipeline::sampler{ .m_AddressMode = std::array{ xgpu::pipeline::sampler::address_mode::CLAMP, xgpu::pipeline::sampler::address_mode::CLAMP, xgpu::pipeline::sampler::address_mode::CLAMP } } };
                auto Shaders  = std::array<const xgpu::shader*, 2>{ &Frag, &Vert };
                // The ground reads the same from both sides, so no culling: the preview's own viewport convention flips the winding
                if (!Ok(Device.Create(m_GridPipeline, xgpu::pipeline::setup{ .m_VertexDescriptor = m_GridVD, .m_Shaders = Shaders, .m_UniformBinds = Binds, .m_Samplers = Samplers
                    , .m_Primitive = { .m_Cull = xgpu::pipeline::primitive::cull::NONE }, .m_Blend = xgpu::pipeline::blend::getAlphaOriginal() }))) return false;
                if (m_bShadow)
                {
                    if (!Ok(Device.Create(m_ShadowMap, { .m_Format = xgpu::texture::format::DEPTH_U16, .m_Width = 1024, .m_Height = 1024, .m_isGamma = false }))) return false;
                    std::array<xgpu::renderpass::attachment, 1> Attachments{ m_ShadowMap };
                    if (!Ok(Device.Create(m_ShadowPass, { .m_Attachments = Attachments }))) return false;
                }
                auto Bindings = std::array{ xgpu::pipeline_instance::sampler_binding{ m_bShadow ? m_ShadowMap : m_White } };
                if (!Ok(Device.Create(m_GridInstance, { .m_PipeLine = m_GridPipeline, .m_SamplersBindings = Bindings }))) return false;
            }

            {
                xgpu::shader Vert, Frag;
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

        // The pipelines and the white texture go back to the device (a dropped one crashes when the device empties its queue); buffers drop by themselves
        void Release() noexcept
        {
            xeditor::DestroyGpu(m_pDevice, m_GridInstance, m_LineInstance, m_GridPipeline, m_LinePipeline, m_ShadowPass, m_ShadowMap, m_White);
        }

        // Right drag turns the camera, middle drag pans, the wheel zooms. Call right after the canvas item was submitted.
        void HandleInput() noexcept
        {
            if (!ImGui::IsItemHovered() && !ImGui::IsItemActive()) return;
            auto& io = ImGui::GetIO();

            if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
            {
                m_Angles.m_Pitch.m_Value -= 0.01f * io.MouseDelta.y;
                m_Angles.m_Yaw.m_Value   -= 0.01f * io.MouseDelta.x;
            }
            if (ImGui::IsMouseDown(ImGuiMouseButton_Middle))
            {
                m_Target += m_View.getWorldYVector() * (0.005f * io.MouseDelta.y);
                m_Target += m_View.getWorldXVector() * (0.005f * io.MouseDelta.x);
            }
            if (m_Distance != -1)
            {
                m_Distance += m_Distance * -0.2f * io.MouseWheel;
                if (m_Distance < 0.5f)
                {
                    m_Target += m_View.getWorldZVector() * (0.5f * (0.5f - m_Distance));
                    m_Distance = 0.5f;
                }
            }
        }

        // Sizes the view to the panel, frames the subject when asked to, and places the camera. Viewport
        // is the panel's ABSOLUTE on-screen rect (matching the original E23 example exactly - Min is the
        // panel's own screen position), not a zero-based one - RayFromScreen takes its own Viewport.Min
        // into account internally, so a zero-based viewport paired with pre-subtracted mouse coords should
        // be equivalent in theory, but matching the proven-working original exactly, byte for byte, is the
        // reliable move over re-deriving equivalence by hand a second time.
        void UpdateView(const ImVec2& Min, float ViewW, float ViewH) noexcept
        {
            m_View.setViewport({ static_cast<int>(Min.x), static_cast<int>(Min.y), static_cast<int>(Min.x + ViewW), static_cast<int>(Min.y + ViewH) });
            // View defaults to aspect 1 (square). FOV projection uses ScaleX = -Focal/aspect, so a
            // non-square panel must get the real W/H here or the projected scene (and any screen math
            // that shares it) drifts as the dock is resized.
            m_View.setAspect(ViewW / ViewH);
            if (m_bReframe)
            {
                m_bReframe = false;
                xgpu::tools::editors::ReframeOrbitCamera(m_View, m_Radius, m_Center, m_Distance, m_Target);
            }
            m_View.LookAt(m_Distance, m_Angles, m_Target);
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

        // Clip space to texture space: the remap every shadow matrix fed to a shader goes through
        static xmath::fmat4 ClipToTextureSpace() noexcept
        {
            xmath::fmat4 M;
            M.setupSRT({ 0.5f, 0.5f, 1.0f }, { 0_xdeg }, { 0.5f, 0.5f, 0.0f });
            return M;
        }

        // The ground grid, fixed at the world origin so it always shows where zero is. From the panel's render callback.
        void DrawGrid(xgpu::cmd_buffer& CmdBuffer) noexcept
        {
            if (!m_bReady) return;
            const xmath::fmat4 W2C = m_View.getW2C();

            CmdBuffer.setPipelineInstance(m_GridInstance);
            auto& Uniform = m_GridUBO.allocEntry<grid_uniform>();
            Uniform.m_WorldSpaceCameraPos = m_View.getPosition();
            Uniform.m_L2W          = xmath::fmat4(xmath::fvec3(100.f, 100.0f, 1.f), xmath::radian3(-90_xdeg, 0_xdeg, 0_xdeg), xmath::fvec3(0, 0, 0));
            Uniform.m_W2C          = W2C;
            Uniform.m_L2CTShadow   = m_bShadow ? ClipToTextureSpace() * m_ShadowL2C * Uniform.m_L2W : xmath::fmat4::fromZero();
            Uniform.m_MajorGridDiv = 10.0f;
            CmdBuffer.setDynamicUBO(m_GridUBO, 0);
            m_Meshes.Rendering(CmdBuffer, e19::mesh_manager::model::PLANE3D);
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
