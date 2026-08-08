#ifndef XSKELETON_DESCRIPTOR_H
#define XSKELETON_DESCRIPTOR_H
#pragma once

#include "plugins/xmaterial_instance.plugin/source/xmaterial_instance_xgpu_rsc_loader.h"
#include "plugins/xskeleton.plugin/source/xskeleton_details.h"
#include <functional>
#include <format>

namespace xskeleton_desc
{
    inline static constexpr auto    resource_type_guid_v    = xresource::type_guid(xresource::guid_generator::Instance64FromString("xSkeleton"));

    static constexpr wchar_t        mesh_filter_v[]         = L"Skeleton\0 *.fbx; *.obj\0Any Thing\0 *.*\0";

    // Author-time transform: Euler radians, not a quaternion - nothing here is ever blended or
    // interpolated (it's a static authored value, converted once by the compiler), so there's no
    // correctness reason to pay for quaternion editing, and Euler is friendlier in a property grid.
    // Same shape as xgeom_static's pre_transform.
    struct transform
    {
        xmath::fvec3        m_Scale         = xmath::fvec3::fromOne();
        xmath::fvec3        m_Rotation      = xmath::fvec3::fromZero();   // radians
        xmath::fvec3        m_Translation   = xmath::fvec3::fromZero();

        XPROPERTY_DEF
        ( "Transform", transform
        , obj_member<"Scale",           &transform::m_Scale >
        , obj_member<"Rotation",        &transform::m_Rotation >
        , obj_member<"Translation",     &transform::m_Translation >
        )
    };
    XPROPERTY_REG(transform)

    // A bone is a bone, whether or not any animation clip ever supplies it a curve.
    // NORMAL   - Expected to receive animation curve data. m_Transform is the delta from the imported bind pose.
    // VIRTUAL  - Never receives curve data from a clip. Its pose is supplied by something outside the
    //            core animation evaluator (a weighted average of other bones, physics/IK, or a static
    //            authored offset for an attachment socket). m_Transform is the delta from the parent,
    //            and is used verbatim until/unless whoever drives this bone overwrites it.
    enum class bone_type : std::uint8_t
    { NORMAL
    , VIRTUAL
    };

    static constexpr auto bone_type_v = std::array
    { xproperty::settings::enum_item("NORMAL",  bone_type::NORMAL)
    , xproperty::settings::enum_item("VIRTUAL", bone_type::VIRTUAL)
    };

    // A sparse override entry for one imported bone, matched BY NAME against the raw import (see
    // xskeleton_compiler.cpp's CollectOverrides, which flattens this tree into a name->override map
    // before compiling - tree position doesn't affect matching). m_Bones (children) is still here
    // because the tree SHAPE mirrors the real skeleton hierarchy (kept in sync by MergeWithDetails),
    // which is what makes this browsable/editable as a real hierarchy in the property panel instead
    // of a flat, order-independent bag of 70+ entries.
    struct bone
    {
        std::string             m_Name          = {};                    // Unique within this skeleton. Bones are matched by name across skin/animation/retargeting at author time - stays the RAW imported name so overrides keep matching across re-imports even after a rename (see m_Rename).
        std::string             m_Rename        = {};                    // If non-empty, this bone's compiled/output name instead of m_Name (mirrors xgeom_static's ungroup_mesh: original identity kept separate from the editable output name).
        transform                m_Transform     = {};                    // See bone_type for how this delta is interpreted.
        bone_type               m_Type          = bone_type::NORMAL;
        bool                    m_bExpose       = false;                 // Show as a socket/attachment point in editors that consume this skeleton.
        bool                    m_bDeleteBone   = false;                 // Marks this (imported) bone for removal from the compiled skeleton.
        int                     m_LODLevel      = 0;                     // Highest LOD index at which this bone is still active; frozen at its rest pose beyond it. 0 = always active.
        std::vector<bone>       m_Bones         = {};                    // Children - see the struct comment.

        XPROPERTY_DEF
        ( "Bone", bone
        , obj_member<"Name",         &bone::m_Name, member_flags< flags::SHOW_READONLY> >
        , obj_member<"Rename",       &bone::m_Rename >
        , obj_member<"Transform",    &bone::m_Transform >
        , obj_member<"Type",         &bone::m_Type, member_enum_span<bone_type_v> >
        , obj_member<"Expose",       &bone::m_bExpose >
        , obj_member<"DeleteBone",   &bone::m_bDeleteBone >
        , obj_member<"LODLevel",     &bone::m_LODLevel >
        , obj_member<"Bones",        &bone::m_Bones >
        )
    };
    XPROPERTY_REG(bone)

    struct mask_entry
    {
        std::string    m_BoneName  = {};
        float          m_Weight    = 1.0f;

        XPROPERTY_DEF
        ( "MaskEntry", mask_entry
        , obj_member<"BoneName",    &mask_entry::m_BoneName >
        , obj_member<"Weight",      &mask_entry::m_Weight >
        )
    };
    XPROPERTY_REG(mask_entry)

    // A named per-bone weight table (e.g. "UpperBody", "LowerBody"). Sparse here (bones not listed
    // default to weight 0) - the compiler bakes it into a dense, fixed-point array sized to the full
    // skeleton. Masking/blending math itself is the animation system's job; the skeleton only owns
    // this name -> weight-table mapping.
    struct mask_group
    {
        std::string                 m_Name      = {};
        std::vector<mask_entry>     m_Entries   = {};

        XPROPERTY_DEF
        ( "MaskGroup", mask_group
        , obj_member<"Name",        &mask_group::m_Name >
        , obj_member<"Entries",     &mask_group::m_Entries >
        )
    };
    XPROPERTY_REG(mask_group)

    struct descriptor : xresource_pipeline::descriptor::base
    {
        using parent = xresource_pipeline::descriptor::base;

        void SetupFromSource(std::string_view FileName) override
        {
        }

        void Validate(std::vector<std::string>& Errors) const noexcept override
        {
        }

        int findMaskGroup(std::string_view Name)
        {
            for (auto& E : m_MaskGroups)
                if (E.m_Name == Name) return static_cast<int>(&E - m_MaskGroups.data());
            return -1;
        }

        static bone* FindChildBone(bone& Node, std::string_view Name) noexcept
        {
            for (auto& Child : Node.m_Bones)
                if (Child.m_Name == Name) return &Child;
            return nullptr;
        }

        // Same job as xgeom_static::descriptor::MergeWithDetails for meshes/nodes - reconciles this
        // override tree against a freshly (re-)imported skeleton so it always has "one entry per
        // bone", browsable/editable as a whole, not just whichever bones someone has already curated.
        // Called from the editor on selection/reload, not the compiler (which only ever reads this,
        // matching by name - see CollectOverrides - so tree SHAPE doesn't affect correctness; it's
        // purely for human browsing/editing).
        //
        // Walks m_RootBone and Details.m_RootBone TOGETHER, level by level, so a new bone gets
        // inserted under its real parent - not flatly dumped under root regardless of where it
        // actually sits - and a bone removed/renamed/reparented in the source gets pruned from
        // wherever it used to be. m_RootBone.m_Name gets kept in sync with the real root's name here
        // too: leaving it blank was a real bug - the "is this bone the root" check the caller used to
        // do elsewhere never matched anything, so the real root bone ended up ALSO added as an
        // ordinary child - two "root bones" in the property panel, one anonymous (m_RootBone itself)
        // and one named (the duplicate). Since m_RootBone is an ordinary `bone` (see its own comment)
        // being the root is just "reachable via this field instead of someone else's m_Bones", not a
        // different shape - so keeping its name in sync here needs no special-casing either.
        std::vector<std::string> MergeWithDetails(const details& Details)
        {
            std::vector<std::string> Messages;

            std::function<void(bone&, const details::bone&)> Reconcile = [&](bone& Node, const details::bone& Src)
            {
                Node.m_Name = Src.m_Name;

                for (int i = 0; i < static_cast<int>(Node.m_Bones.size()); ++i)
                {
                    bool bStillAChild = false;
                    for (auto& SrcChild : Src.m_Children)
                        if (SrcChild.m_Name == Node.m_Bones[i].m_Name) { bStillAChild = true; break; }
                    if (bStillAChild) continue;

                    Messages.push_back(std::format("WARNING: Bone [{}] no longer found under its parent in the imported skeleton - removing its override.", Node.m_Bones[i].m_Name));
                    Node.m_Bones.erase(Node.m_Bones.begin() + i);
                    --i;
                }

                for (auto& SrcChild : Src.m_Children)
                {
                    bone* pChild = FindChildBone(Node, SrcChild.m_Name);
                    if (!pChild)
                    {
                        Node.m_Bones.push_back(bone{ .m_Name = SrcChild.m_Name });
                        pChild = &Node.m_Bones.back();
                    }
                    Reconcile(*pChild, SrcChild);
                }
            };

            if (!Details.m_RootBone.m_Name.empty())
                Reconcile(m_RootBone, Details.m_RootBone);

            return Messages;
        }

        std::wstring                m_ImportAsset   = {};
        transform                    m_PreTransform  = {};    // Author-time scale/origin correction applied before compiling (imported skeletons are often at the wrong scale/pivot).
        bone                         m_RootBone      = {};    // A skeleton has exactly one root (matches the existing runtime convention: bone 0 is always the sole root) - reached through this field rather than through someone else's m_Bones, but otherwise an ordinary `bone`, same shape as any other.
        std::vector<mask_group>     m_MaskGroups    = {};

        XPROPERTY_VDEF
        ( "Skeleton", descriptor
        , obj_member<"ImportAsset",     &descriptor::m_ImportAsset, member_ui<std::wstring>::file_dialog<mesh_filter_v, true, 1> >
        , obj_member<"PreTransform",    &descriptor::m_PreTransform >
        , obj_member<"RootBone",        &descriptor::m_RootBone >
        , obj_member<"MaskGroups",      &descriptor::m_MaskGroups >
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
            return "Skeleton";
        }

        const xproperty::type::object& ResourceXPropertyObject(void) const noexcept override
        {
            return *xproperty::getObjectByType<descriptor>();
        }
    };

    inline static factory g_Factory{};
}
#endif
