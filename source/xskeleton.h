#ifndef XGEOM_STATIC_RUNTIME_H
#define XGEOM_STATIC_RUNTIME_H
#pragma once

#include "dependencies/xmath/source/xmath_fshapes.h"
#include "dependencies/xserializer/source/xserializer.h"
#include <span>  // Add for std::span

namespace xgeom_static
{
    struct geom
    {
        inline static constexpr auto xserializer_version_v = 1;
        struct mesh
        {
            std::array<char, 32>    m_Name;
            float                   m_WorldPixelSize;   // Average World Pixel size for this SubMesh
            xmath::fbbox            m_BBox;
            std::uint16_t           m_nLODs;
            std::uint16_t           m_iLOD;
        };

        struct lod
        {
            float                   m_ScreenArea;
            std::uint16_t           m_iSubmesh;         // Start the submeshes
            std::uint16_t           m_nSubmesh;
        };

        struct submesh
        {
            std::uint16_t           m_iCluster;         // Where the index starts
            std::uint16_t           m_nCluster;         // Where the index starts
            std::uint16_t           m_iMaterial;        // Index of the Material that this SubMesh uses
        };

        struct vec3
        {
            float m_X, m_Y, m_Z;
        };

        struct vec2
        {
            float m_X, m_Y;
        };

        struct vec4
        {
            float m_X, m_Y, m_Z, m_W;
        };

        struct cluster
        {
            vec4                    m_PosScaleAndUScale;        // XYZ scale for the cluster, W = U Scale
            vec4                    m_PosTrasnlationAndVScale;  // XYZ scale for the cluster, W = V Scale
            vec2                    m_UVTranslation;            // UV Translation.            
            xmath::fbbox            m_BBox;                     // Optional fine-grained CPU culling (e.g., per-cluster frustum/occlusion)
            std::uint32_t           m_iIndex;                   // Where the index starts
            std::uint32_t           m_nIndices;                 // number of
            std::uint32_t           m_iVertex;                  // Where the vertex starts
            std::uint32_t           m_nVertices;                // number of
        };

        struct vertex
        {
            int16_t m_XPos, m_YPos, m_ZPos;
            int16_t m_Extra;                            // Bit 0: binormal sign (0:+1, 1:-1)
        };

        struct vertex_extras
        {
            std::array<std::uint16_t,2>     m_UV;
            std::array<std::uint8_t, 2>     m_OctNormal;
            std::array<std::uint8_t, 2>     m_OctTangent;
        };

        using runtime_allocation = std::array<std::size_t, 3*2>;

        //-------------------------------------------------------------------------

                                                        geom                        (void)                                      noexcept = default;
        inline                                          geom                        (xserializer::stream& Steaming)             noexcept;
        inline void                                     Kill                        (void)                                      noexcept;
        inline void                                     Initialize                  (void)                                      noexcept;
        inline int                                      findMeshIndex               (const char* pName)                 const   noexcept;
        inline int                                      getSubMeshIndex             (int iMesh, int iMaterialInstance)  const   noexcept;
        inline std::span<mesh>                          getMeshes                   (void)                              const   noexcept { return { m_pMesh, m_nMeshes }; }
        inline std::span<lod>                           getLODs                     (void)                              const   noexcept { return { m_pLOD, m_nLODs }; }
        inline std::span<submesh>                       getSubmeshes                (void)                              const   noexcept { return { m_pSubMesh, m_nSubMeshs }; }
        inline std::span<cluster>                       getClusters                 (void)                              const   noexcept { return { m_pCluster, m_nClusters }; }
        inline std::span<vertex>                        getVertices                 (void)                              const   noexcept { return { reinterpret_cast<vertex*>(m_pData + m_VertexOffset), m_nVertices }; }
        inline std::span<vertex_extras>                 getVertexExtras             (void)                              const   noexcept { return { reinterpret_cast<vertex_extras*>(m_pData + m_VertexExtrasOffset), m_nVertices }; }
        inline std::span<std::uint16_t>                 getIndices                  (void)                              const   noexcept { return { reinterpret_cast<std::uint16_t*>(m_pData + m_IndicesOffset), m_nIndices }; }
        inline std::span<xrsc::material_instance_ref>   getDefaultMaterialInstances (void)                              const   noexcept { return { m_pDefaultMaterialInstances, m_nDefaultMaterialInstances }; }

        xmath::fbbox                    m_BBox;
        char*                           m_pData;  // Contiguous buffer for GPU data ( vertices, extras, indices)
        mesh*                           m_pMesh;  // Separate allocations for CPU-persistent data
        lod*                            m_pLOD;
        submesh*                        m_pSubMesh;
        cluster*                        m_pCluster;
        xrsc::material_instance_ref*    m_pDefaultMaterialInstances;
        runtime_allocation              m_RunTimeSpace;
        std::size_t                     m_DataSize;
        std::size_t                     m_VertexOffset;
        std::size_t                     m_VertexExtrasOffset;
        std::size_t                     m_IndicesOffset;
        std::uint16_t                   m_nMeshes;
        std::uint16_t                   m_nLODs;
        std::uint16_t                   m_nSubMeshs;
        std::uint16_t                   m_nClusters;
        std::uint32_t                   m_nIndices;
        std::uint32_t                   m_nVertices;
        std::uint16_t                   m_nDefaultMaterialInstances;
    };

    //-------------------------------------------------------------------------

    geom::geom(xserializer::stream& Steaming) noexcept
    {
        //xassert( Steaming.getResourceVersion() == xgeom::VERSION );
    }

    //-------------------------------------------------------------------------

    void geom::Initialize(void) noexcept
    {
        std::memset(this, 0, sizeof(*this));
    }

    //-------------------------------------------------------------------------
    void geom::Kill(void) noexcept
    {
        if (m_pMesh)                        delete[] m_pMesh;
        if (m_pLOD)                         delete[] m_pLOD;
        if (m_pSubMesh)                     delete[] m_pSubMesh;
        if (m_pCluster)                     delete[] m_pCluster;
        if (m_pDefaultMaterialInstances)    delete[] m_pDefaultMaterialInstances;
        if (m_pData)                        delete[] m_pData;

        Initialize();
    }

    //-------------------------------------------------------------------------

    int geom::findMeshIndex(const char* pName) const noexcept
    {
        auto meshes = getMeshes();
        for (auto i = 0u; i < meshes.size(); i++)
        {
            if (!std::strcmp(meshes[i].m_Name.data(), pName))
            {
                return i;
            }
        }
        return -1;
    }
}

//-------------------------------------------------------------------------
// serializer
//-------------------------------------------------------------------------
namespace xserializer::io_functions
{
    //-------------------------------------------------------------------------
    template<> inline
        xerr SerializeIO<xgeom_static::geom::lod>(xserializer::stream& Stream, const xgeom_static::geom::lod& Lod) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Lod.m_ScreenArea))
            || (Err = Stream.Serialize(Lod.m_iSubmesh))
            || (Err = Stream.Serialize(Lod.m_nSubmesh))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xgeom_static::geom::mesh>(xserializer::stream& Stream, const xgeom_static::geom::mesh& Mesh) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Mesh.m_Name))
            || (Err = Stream.Serialize(Mesh.m_WorldPixelSize))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Min.m_X))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Min.m_Y))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Min.m_Z))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Max.m_X))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Max.m_Y))
            || (Err = Stream.Serialize(Mesh.m_BBox.m_Max.m_Z))
            || (Err = Stream.Serialize(Mesh.m_nLODs))
            || (Err = Stream.Serialize(Mesh.m_iLOD))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xgeom_static::geom::submesh>(xserializer::stream& Stream, const xgeom_static::geom::submesh& Submesh) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Submesh.m_iCluster))
            || (Err = Stream.Serialize(Submesh.m_nCluster))
            || (Err = Stream.Serialize(Submesh.m_iMaterial))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xgeom_static::geom::cluster>(xserializer::stream& Stream, const xgeom_static::geom::cluster& Cluster) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Cluster.m_PosScaleAndUScale.m_X))
            || (Err = Stream.Serialize(Cluster.m_PosScaleAndUScale.m_Y))
            || (Err = Stream.Serialize(Cluster.m_PosScaleAndUScale.m_Z))
            || (Err = Stream.Serialize(Cluster.m_PosScaleAndUScale.m_W))

            || (Err = Stream.Serialize(Cluster.m_PosTrasnlationAndVScale.m_X))
            || (Err = Stream.Serialize(Cluster.m_PosTrasnlationAndVScale.m_Y))
            || (Err = Stream.Serialize(Cluster.m_PosTrasnlationAndVScale.m_Z))
            || (Err = Stream.Serialize(Cluster.m_PosTrasnlationAndVScale.m_W))

            || (Err = Stream.Serialize(Cluster.m_UVTranslation.m_X))
            || (Err = Stream.Serialize(Cluster.m_UVTranslation.m_Y))

            || (Err = Stream.Serialize(Cluster.m_nVertices))
            || (Err = Stream.Serialize(Cluster.m_nIndices))
            || (Err = Stream.Serialize(Cluster.m_iIndex))
            || (Err = Stream.Serialize(Cluster.m_iVertex))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Min.m_X))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Min.m_Y))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Min.m_Z))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Max.m_X))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Max.m_Y))
            || (Err = Stream.Serialize(Cluster.m_BBox.m_Max.m_Z))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xrsc::material_instance_ref>(xserializer::stream& Stream, const xrsc::material_instance_ref& IR) noexcept
    {
        return Stream.Serialize(IR.m_Instance.m_Value);
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xgeom_static::geom>(xserializer::stream& Stream, const xgeom_static::geom& Geom) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Geom.m_nMeshes))
            || (Err = Stream.Serialize(Geom.m_pMesh,                        Geom.m_nMeshes))
            || (Err = Stream.Serialize(Geom.m_nLODs))
            || (Err = Stream.Serialize(Geom.m_pLOD,                         Geom.m_nLODs))
            || (Err = Stream.Serialize(Geom.m_nSubMeshs))
            || (Err = Stream.Serialize(Geom.m_pSubMesh,                     Geom.m_nSubMeshs))
            || (Err = Stream.Serialize(Geom.m_nClusters))
            || (Err = Stream.Serialize(Geom.m_pCluster,                     Geom.m_nClusters))
            || (Err = Stream.Serialize(Geom.m_nDefaultMaterialInstances))
            || (Err = Stream.Serialize(Geom.m_pDefaultMaterialInstances,    Geom.m_nDefaultMaterialInstances))
            || (Err = Stream.Serialize(Geom.m_DataSize))
            || (Err = Stream.Serialize(Geom.m_pData,                        Geom.m_DataSize))
            || (Err = Stream.Serialize(Geom.m_RunTimeSpace))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Min.m_X))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Min.m_Y))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Min.m_Z))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Max.m_X))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Max.m_Y))
            || (Err = Stream.Serialize(Geom.m_BBox.m_Max.m_Z))
            || (Err = Stream.Serialize(Geom.m_VertexOffset))
            || (Err = Stream.Serialize(Geom.m_VertexExtrasOffset))
            || (Err = Stream.Serialize(Geom.m_IndicesOffset))
            || (Err = Stream.Serialize(Geom.m_nVertices))
            || (Err = Stream.Serialize(Geom.m_nIndices))
            ;
        return Err;
    }
}

#endif

