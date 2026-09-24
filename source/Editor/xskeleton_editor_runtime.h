#ifndef XSKELETON_EDITOR_RUNTIME_H
#define XSKELETON_EDITOR_RUNTIME_H
#pragma once

// The skeleton, animation and skin editors' shared bone-wedge GPU runtime: the fill's translucent
// shading and the shadow it casts. Split out of xskeleton_editor.h (which still owns the interactive
// session built on top of this) so a consumer that only needs the runtime - xskeleton_thumbnail.h,
// specifically - can include it without pulling in the whole session and, more importantly, without
// the circular include that would otherwise create (xskeleton_editor.h registers the thumbnail
// renderer, so it must be able to include xskeleton_thumbnail.h, not the other way around).
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_scene.h"

namespace xskeleton_editor
{
    // The fill's own shaders (its push constants carry the shadow matrix) and the light's depth-only pair
    inline constexpr std::uint32_t g_FillVertShader[] =
    {
        #include "E23_WedgeFill_vert.h"
    };
    inline constexpr std::uint32_t g_FillFragShader[] =
    {
        #include "E23_WedgeFill_frag.h"
    };
    inline constexpr std::uint32_t g_ShadowVertShader[] =
    {
        #include "E23_ShadowGeneration_vert.h"
    };
    inline constexpr std::uint32_t g_ShadowFragShader[] =
    {
        #include "E23_ShadowGeneration_frag.h"
    };

    // Matches E23_WedgeFill_vert/frag.glsl's push constant block: both stages declare all of it, so the fields land at the same offsets
    struct wedge_fill_push_constants
    {
        xmath::fmat4    m_L2C;
        xmath::fmat4    m_ShadowL2C;
        float           m_Boost;
    };

    struct shadow_generation_push_constants
    {
        xmath::fmat4    m_L2C;
    };

    //--------------------------------------------------------------------------------------------
    // The scene: the shared one with the bones' translucent fill and the shadow they cast
    //--------------------------------------------------------------------------------------------
    struct runtime : scene
    {
        xgpu::tools::view           m_LightView;
        xgpu::vertex_descriptor     m_ShadowVD;
        xgpu::buffer                m_FillVerts;
        xgpu::pipeline              m_FillPipeline, m_ShadowPipeline;
        xgpu::pipeline_instance     m_FillInstance, m_ShadowInstance;
        std::size_t                 m_nFill = 0;

        bool Init(xgpu::device& Device) noexcept
        {
            if (m_bReady) return true;
            if (!scene::Init(Device, true)) return false;
            m_bReady = false;
            m_LightView.setFov(50_xdeg);
            m_LightView.setViewport({ 0, 0, m_ShadowMap.getTextureDimensions()[0], m_ShadowMap.getTextureDimensions()[1] });

            if (!Ok(Device.Create(m_FillVerts, { .m_Type = xgpu::buffer::type::VERTEX, .m_Usage = xgpu::buffer::setup::usage::CPU_WRITE_GPU_READ, .m_EntryByteSize = sizeof(e19::draw_vert), .m_EntryCount = g_MaxWedgeVertices }))) return false;

            auto Shader = [&](xgpu::shader& Out, xgpu::shader::type::bit Type, const std::uint32_t* pCode, std::size_t nWords)
            {
                return Ok(Device.Create(Out, { .m_Type = Type, .m_Sharer = xgpu::shader::setup::raw_data{ std::span{ (std::int32_t*)pCode, nWords } } }));
            };

            // The fill: triangles, alpha blended, tested against the depth but not written to it (the fills are drawn far to near, so one blends over another)
            {
                xgpu::shader Vert, Frag;
                if (!Shader(Vert, xgpu::shader::type::bit::VERTEX,   g_FillVertShader, std::size(g_FillVertShader))) return false;
                if (!Shader(Frag, xgpu::shader::type::bit::FRAGMENT, g_FillFragShader, std::size(g_FillFragShader))) return false;
                auto Samplers = std::array{ xgpu::pipeline::sampler{}, xgpu::pipeline::sampler{ .m_AddressMode = std::array{ xgpu::pipeline::sampler::address_mode::CLAMP, xgpu::pipeline::sampler::address_mode::CLAMP, xgpu::pipeline::sampler::address_mode::CLAMP } } };
                auto Shaders  = std::array<const xgpu::shader*, 2>{ &Frag, &Vert };
                if (!Ok(Device.Create(m_FillPipeline, xgpu::pipeline::setup
                    { .m_VertexDescriptor = m_GridVD, .m_Shaders = Shaders, .m_PushConstantsSize = sizeof(wedge_fill_push_constants), .m_Samplers = Samplers
                    , .m_DepthStencil = { .m_bDepthWriteEnable = false }, .m_Blend = xgpu::pipeline::blend::getAlphaOriginal() }))) return false;
                auto Bindings = std::array{ xgpu::pipeline_instance::sampler_binding{ m_White }, xgpu::pipeline_instance::sampler_binding{ m_ShadowMap } };
                if (!Ok(Device.Create(m_FillInstance, { .m_PipeLine = m_FillPipeline, .m_SamplersBindings = Bindings }))) return false;
            }

            // The light's view of the fills: positions only, depth only
            {
                xgpu::shader Vert, Frag;
                if (!Shader(Vert, xgpu::shader::type::bit::VERTEX,   g_ShadowVertShader, std::size(g_ShadowVertShader))) return false;
                if (!Shader(Frag, xgpu::shader::type::bit::FRAGMENT, g_ShadowFragShader, std::size(g_ShadowFragShader))) return false;
                auto Attributes = std::array{ xgpu::vertex_descriptor::attribute{ .m_Offset = offsetof(e19::draw_vert, m_X), .m_Format = xgpu::vertex_descriptor::format::FLOAT_3D } };
                if (!Ok(Device.Create(m_ShadowVD, xgpu::vertex_descriptor::setup{ .m_VertexSize = sizeof(e19::draw_vert), .m_Attributes = Attributes }))) return false;
                auto Shaders = std::array<const xgpu::shader*, 2>{ &Vert, &Frag };
                if (!Ok(Device.Create(m_ShadowPipeline, xgpu::pipeline::setup
                    { .m_VertexDescriptor = m_ShadowVD, .m_Shaders = Shaders, .m_PushConstantsSize = sizeof(shadow_generation_push_constants)
                    , .m_DepthStencil = { .m_DepthBiasConstantFactor = 1.25f, .m_DepthBiasSlopeFactor = 2.3f, .m_bDepthBiasEnable = true, .m_bDepthClampEnable = true } }))) return false;
                if (!Ok(Device.Create(m_ShadowInstance, { .m_PipeLine = m_ShadowPipeline }))) return false;
            }

            m_bReady = true;
            return true;
        }

        void Release() noexcept
        {
            xeditor::DestroyGpu(m_pDevice, m_FillInstance, m_ShadowInstance, m_FillPipeline, m_ShadowPipeline);
            scene::Release();
        }

        void SetFill(const std::vector<e19::draw_vert>& Verts) noexcept
        {
            m_nFill = std::min<std::size_t>(Verts.size(), g_MaxWedgeVertices);
            if (m_nFill) (void)m_FillVerts.MemoryMap(0, static_cast<int>(m_nFill), [&](void* pData) { std::memcpy(pData, Verts.data(), m_nFill * sizeof(e19::draw_vert)); });
        }

        // The fills seen from a light that keeps its direction while the camera orbits, refitted to the skeleton every frame. Opens its own render
        // pass on the window: it must run before the frame's UI is rendered.
        void RenderShadow(xgpu::window& Window) noexcept
        {
            const float VerticalFov = m_LightView.getFov().m_Value;
            const float HFov        = 2.0f * std::atan(m_LightView.getAspect() * std::tan(VerticalFov * 0.5f));
            const float Distance    = (m_Radius + 0.01f) / std::tan(std::min(VerticalFov, HFov) * 0.5f);
            // The near plane stays a fraction of the distance: a fixed floor collapses the depth precision once the subject is bigger
            m_LightView.setNearZ(Distance * 0.1f);
            m_LightView.setFarZ(Distance + m_Radius * 4.0f);
            m_LightView.LookAt(Distance, xmath::radian3(-50_xdeg, 35_xdeg, 0_xdeg), m_Center);
            m_ShadowL2C = m_LightView.getW2C();

            if (!m_nFill) return;
            auto CmdBuffer = Window.StartRenderPass(m_ShadowPass);
            CmdBuffer.setPipelineInstance(m_ShadowInstance);
            CmdBuffer.setBuffer(m_LineIndices);
            CmdBuffer.setBuffer(m_FillVerts);
            CmdBuffer.setPushConstants(shadow_generation_push_constants{ .m_L2C = m_ShadowL2C });
            CmdBuffer.Draw(static_cast<int>(m_nFill));
        }

        // The ground, all the outlines, then the fills far to near on top (the builder sorted them)
        void DrawBones(xgpu::cmd_buffer& CmdBuffer) noexcept
        {
            DrawGrid(CmdBuffer);
            DrawLines(CmdBuffer);
            if (!m_nFill) return;
            CmdBuffer.setPipelineInstance(m_FillInstance);
            CmdBuffer.setBuffer(m_LineIndices);
            CmdBuffer.setBuffer(m_FillVerts);
            CmdBuffer.setPushConstants(wedge_fill_push_constants{ .m_L2C = m_View.getW2C(), .m_ShadowL2C = ClipToTextureSpace() * m_ShadowL2C, .m_Boost = g_WedgeFillBoost });
            CmdBuffer.Draw(static_cast<int>(m_nFill));
        }
    };
}

#endif // XSKELETON_EDITOR_RUNTIME_H
