#ifndef XGEOM_STATIC_DESCRIPTOR_H
#define XGEOM_STATIC_DESCRIPTOR_H
#pragma once

#include "plugins/xmaterial_instance.plugin/source/xmaterial_instance_xgpu_rsc_loader.h"

namespace xgeom_static
{
    // While this should be just a type... it also happens to be an instance... the instance of the texture_plugin
    // So while generating the type guid we must treat it as an instance.
    inline static constexpr auto    resource_type_guid_v    = xresource::type_guid(xresource::guid_generator::Instance64FromString("GeomStatic"));

    static constexpr wchar_t        mesh_filter_v[]         = L"Mesh\0 *.fbx; *.obj\0Any Thing\0 *.*\0";
    inline static constexpr auto    merged_mesh_name_v      = "MERGED_MESH!!";

    struct lod
    {
        float               m_LODReduction  = 0.7f;
        float               m_ScreenArea    = 1;            // in pixels
        XPROPERTY_DEF
        ( "lod", lod
        , obj_member<"LODReduction",    &lod::m_LODReduction >
        , obj_member<"ScreenArea",      &lod::m_ScreenArea >
        )
    };
    XPROPERTY_REG(lod)

    struct mesh
    {
        std::string                 m_OriginalName          = {};
        std::vector<std::string>    m_MaterialList          = {};
        bool                        m_bMerge                = true;
        std::uint32_t               m_MeshGUID              = {};
        std::vector<lod>            m_LODs                  = {};

        XPROPERTY_DEF
        ( "mesh", mesh
        , obj_member<"OriginalName",        &mesh::m_OriginalName, member_flags< flags::SHOW_READONLY> >
        , obj_member<"Materials",           &mesh::m_MaterialList, member_flags< flags::SHOW_READONLY>, member_flags<flags::DONT_SAVE> >
        , obj_member<"Merge",               &mesh::m_bMerge, member_dynamic_flags < +[](const mesh& O)
            {
                xproperty::flags::type Flags = {};
                Flags.m_bShowReadOnly = O.m_OriginalName == merged_mesh_name_v;
                return Flags;
            } >>
        , obj_member<"MeshGUID",            &mesh::m_MeshGUID, member_dynamic_flags < +[](const mesh& O)
            {
                xproperty::flags::type Flags = {};
                Flags.m_bDontShow         = O.m_bMerge;
                return Flags;
            } >>
        , obj_member<"LODs",                &mesh::m_LODs >
        )
    };
    XPROPERTY_REG(mesh)

    struct pre_transform
    {
        xmath::fvec3        m_Scale         = xmath::fvec3::fromOne();
        xmath::fvec3        m_Rotation      = xmath::fvec3::fromZero();
        xmath::fvec3        m_Translation   = xmath::fvec3::fromZero();

        XPROPERTY_DEF
        ( "preTransform", pre_transform
        , obj_member<"Scale",           &pre_transform::m_Scale >
        , obj_member<"Rotation",        &pre_transform::m_Rotation >
        , obj_member<"Translation",     &pre_transform::m_Translation >
        )
    };
    XPROPERTY_REG(pre_transform)

/*
    struct data
    {
        int                 m_nUVs          = 1;
        int                 m_nColors       = 0;

        XPROPERTY_DEF
        ( "data", data
        , obj_member<"NumUVs",           &data::m_nUVs >
        , obj_member<"NumColors",        &data::m_nColors >
        )
    };
    XPROPERTY_REG(data)
*/

    struct descriptor : xresource_pipeline::descriptor::base
    {
        using parent = xresource_pipeline::descriptor::base;

        void SetupFromSource(std::string_view FileName) override
        {
        }

        void Validate(std::vector<std::string>& Errors) const noexcept override
        {
        }

        int findMesh(std::string_view Name)
        {
            for ( auto&E : m_MeshList)
                if ( E.m_OriginalName == Name ) return static_cast<int>(&E - m_MeshList.data());
            return -1;
        }

        int findMaterial( std::string_view Name )
        {
            for (auto& E : m_MaterialInstNamesList)
                if (E == Name) return static_cast<int>(&E - m_MaterialInstNamesList.data());
            return -1;
        }

        bool hasMergedMesh()
        {
            return findMesh(merged_mesh_name_v) != -1 ;
        }

        void AddMergedMesh()
        {
            if (hasMergedMesh() == false)
            {
                auto& MergedMesh            = m_MeshList.emplace_back();
                MergedMesh.m_OriginalName   = merged_mesh_name_v;
                MergedMesh.m_bMerge         = false;
                m_MeshNoncollapseVisibleList.push_back(&MergedMesh);
            }
        }

        void RemoveMergedMesh()
        {
            if (auto I = findMesh(merged_mesh_name_v); I != -1)
            {
                for (auto& E : m_MeshNoncollapseVisibleList)
                {
                    if (E == &m_MeshList[I])
                    {
                        m_MeshNoncollapseVisibleList.erase(m_MeshNoncollapseVisibleList.begin() + static_cast<int>(&E - m_MeshNoncollapseVisibleList.data()));
                        break;
                    }
                }

                m_MeshList.erase(m_MeshList.begin() + I);
            }
        }

        std::wstring                                m_ImportAsset                   = {};
        pre_transform                               m_PreTranslation                = {};
        bool                                        m_bMergeMeshes                  = true;
        bool                                        m_bHideCopasedMeshes            = true;
        std::vector<mesh>                           m_MeshList                      = {};
        std::vector<xrsc::material_instance_ref>    m_MaterialInstRefList           = {};
        std::vector<std::string>                    m_MaterialInstNamesList         = {};
        std::vector<mesh*>                          m_MeshNoncollapseVisibleList    = {};

        XPROPERTY_VDEF
        ( "GeomStatic", descriptor
        , obj_member<"ImportAsset",         &descriptor::m_ImportAsset, member_ui<std::wstring>::file_dialog<mesh_filter_v, true, 1> >
        , obj_member<"PreTranslation",      &descriptor::m_PreTranslation >
        , obj_member<"bMergeMeshes",        +[](descriptor& O, bool bRead, bool& Value)
            {
                if (bRead) Value = O.m_bMergeMeshes;
                else
                {
                    if (Value) O.AddMergedMesh();
                    else       O.RemoveMergedMesh();
                    O.m_bMergeMeshes = Value;
                }
            }>

        , obj_member<"bHideCopasedMeshes", +[](descriptor& O, bool bRead, bool& Value )
            {
                if (bRead) Value = O.m_bHideCopasedMeshes;
                else
                {
                    if (Value)
                    {
                        O.m_MeshNoncollapseVisibleList.clear();
                        for (auto& E : O.m_MeshList)
                        {
                            if (E.m_bMerge == false || E.m_OriginalName == merged_mesh_name_v)
                                O.m_MeshNoncollapseVisibleList.push_back(&E);
                        }
                    }

                    O.m_bHideCopasedMeshes = Value;
                }
            }, member_dynamic_flags < +[](const descriptor& O)
            {
                xproperty::flags::type Flags = {};
                Flags.m_bDontShow = !O.m_bMergeMeshes;
                return Flags;
            } >>
        , obj_member<"MeshList",            &descriptor::m_MeshList, member_dynamic_flags<+[](const descriptor& O)
            {
                xproperty::flags::type Flags = {};
                Flags.m_bShowReadOnly   = false;
                Flags.m_bDontShow       = O.m_bMergeMeshes && O.m_bHideCopasedMeshes;
                return Flags;
            } >>
        , obj_member<"MaterialInstNames", &descriptor::m_MaterialInstNamesList, member_flags<flags::DONT_SHOW>>
        , obj_member<"MeshListNonCollapse", &descriptor::m_MeshNoncollapseVisibleList, member_dynamic_flags<+[](const descriptor& O)
            {
                xproperty::flags::type Flags = {};
                Flags.m_bShowReadOnly   = false;
                Flags.m_bDontShow       = !O.m_bHideCopasedMeshes || !O.m_bMergeMeshes;
                Flags.m_bDontSave       = true;
                return Flags;
            }>>

        , obj_member<"MaterialInstance",    &descriptor::m_MaterialInstRefList, member_ui_open<true> >
        )
    };
    XPROPERTY_VREG(descriptor)

    //--------------------------------------------------------------------------------------

    struct factory final : xresource_pipeline::factory_base
    {
        using xresource_pipeline::factory_base::factory_base;

        std::unique_ptr<xresource_pipeline::descriptor::base> CreateDescriptor(void) const noexcept override
        {
            return std::make_unique<descriptor>();
        };

        xresource::type_guid ResourceTypeGUID(void) const noexcept override
        {
            return resource_type_guid_v;
        }

        const char* ResourceTypeName(void) const noexcept override
        {
            return "GeomStatic";
        }

        const xproperty::type::object& ResourceXPropertyObject(void) const noexcept override
        {
            return *xproperty::getObjectByType<descriptor>();
        }
    };

    inline static factory g_Factory{};
}
#endif