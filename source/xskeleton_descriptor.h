#ifndef XSKELETON_DESCRIPTOR_H
#define XSKELETON_DESCRIPTOR_H
#pragma once

#include "plugins/xmaterial_instance.plugin/source/xmaterial_instance_xgpu_rsc_loader.h"
#include "plugins/xskeleton.plugin/source/xskeleton_details.h"
#include "plugins/xskeleton.plugin/source/xskeleton.h"
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
        xmath::fvec3        m_Rotation      = xmath::fvec3::fromZero();   // degrees (X pitch, Y yaw, Z roll: the compiler's ApplyPreTransform)
        xmath::fvec3        m_Translation   = xmath::fvec3::fromZero();
        bool                m_EditorLockScale = true;       // editor only: the Scale lock toggle (editing one axis scales all three)

        XPROPERTY_DEF
        ( "Transform", transform
        , obj_scope<"Scale", xproperty::settings::vector3_group
            , obj_member<"X", &xmath::LockedScaleAxis<transform, &transform::m_Scale, &transform::m_EditorLockScale, 0>>
            , obj_member<"Y", &xmath::LockedScaleAxis<transform, &transform::m_Scale, &transform::m_EditorLockScale, 1>>
            , obj_member<"Z", &xmath::LockedScaleAxis<transform, &transform::m_Scale, &transform::m_EditorLockScale, 2>>
            >
        , obj_member<"EditorLockScale", &transform::m_EditorLockScale, member_flags<flags::DONT_SHOW>>
        , obj_member<"Rotation",        &transform::m_Rotation >
        , obj_member<"Translation",     &transform::m_Translation >
        )
    };
    XPROPERTY_REG(transform)

    // A bone is a bone, whether or not any animation clip ever supplies it a curve.
    // NORMAL   - Expected to receive animation curve data.
    // VIRTUAL  - Never receives curve data from a clip. Its pose is supplied by something outside the
    //            core animation evaluator (a weighted average of other bones, physics/IK, or a static
    //            authored offset for an attachment socket).
    enum class bone_type : std::uint8_t
    { NORMAL
    , VIRTUAL
    };

    // A bone's intent as the user set it: AUTO follows xskeleton::SuggestIntent (the name), the rest are the user's choice (NONE included)
    enum class bone_intent : std::uint8_t
    { AUTO
    , NONE
    , MOTION
    , FACING
    , LOOK_AT
    , IK_TARGET
    };

    static constexpr auto bone_intent_v = std::array
    { xproperty::settings::enum_item("Auto",      bone_intent::AUTO)
    , xproperty::settings::enum_item("None",      bone_intent::NONE)
    , xproperty::settings::enum_item("Motion",    bone_intent::MOTION)
    , xproperty::settings::enum_item("Facing",    bone_intent::FACING)
    , xproperty::settings::enum_item("Look-at",   bone_intent::LOOK_AT)
    , xproperty::settings::enum_item("IK target", bone_intent::IK_TARGET)
    };

    // The intent a bone ends up with: the user's, or the suggestion from its (raw) name
    inline xskeleton::intent ResolveIntent(bone_intent Override, std::string_view RawName) noexcept
    {
        switch (Override)
        {
        case bone_intent::AUTO:      return xskeleton::SuggestIntent(RawName);
        case bone_intent::MOTION:    return xskeleton::intent::MOTION;
        case bone_intent::FACING:    return xskeleton::intent::FACING;
        case bone_intent::LOOK_AT:   return xskeleton::intent::LOOK_AT;
        case bone_intent::IK_TARGET: return xskeleton::intent::IK_TARGET;
        default:                     return xskeleton::intent::NONE;
        }
    }

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
        bone_type               m_Type          = bone_type::NORMAL;
        bool                    m_bExpose       = false;                 // Show as a socket/attachment point in editors that consume this skeleton.
        bool                    m_bDeleteBone   = false;                 // Marks this (imported) bone for removal from the compiled skeleton.
        int                     m_LODLevel      = 0;                     // Highest LOD index at which this bone is still active; frozen at its rest pose beyond it. 0 = always active.
        bone_intent             m_Intent        = bone_intent::AUTO;     // What the bone is for (xskeleton::intent); AUTO: the suggestion from its name
        std::vector<bone>       m_Bones         = {};                    // Children - see the struct comment.

        XPROPERTY_DEF
        ( "Bone", bone
        , obj_member<"Name",         &bone::m_Name, member_flags< flags::SHOW_READONLY> >
        , obj_member<"Rename",       &bone::m_Rename >
        , obj_member<"Type",         &bone::m_Type, member_enum_span<bone_type_v> >
        , obj_member<"Expose",       &bone::m_bExpose >
        , obj_member<"DeleteBone",   &bone::m_bDeleteBone >
        , obj_member<"LODLevel",     &bone::m_LODLevel >
        , obj_member<"Intent",       &bone::m_Intent, member_enum_span<bone_intent_v> >
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
        transform                    m_PostTransform = {};    // The correction between the frozen pose / the animations and the bind data of the skins (the model was rotated while it was animated): applied to the frozen pose (the rest of the root) BEFORE the PreTransform, and to the root of every clip of the animations that use this skeleton, in the same way: both agree. Not to the bind data nor the skins.
        bone                         m_RootBone      = {};    // A skeleton has exactly one root (matches the existing runtime convention: bone 0 is always the sole root) - reached through this field rather than through someone else's m_Bones, but otherwise an ordinary `bone`, same shape as any other.
        std::vector<mask_group>     m_MaskGroups    = {};
        bool                        m_bCompactTree  = false;  // the bone tree is too deep for xproperty's paths: RootBone is empty in the properties, the tree follows as the "BoneOverrides" table

        static int Depth(const bone& B) noexcept
        {
            int d = 0;
            for (auto& C : B.m_Bones) d = std::max(d, Depth(C));
            return d + 1;
        }

        // The tree as a flat pre-order table: parent index and every field of a bone (xproperty builds each path in a 256 char buffer)
        static xerr StreamCompactTree(xtextfile::stream& File, bone& Tree, bool bLegacyColumns) noexcept
        {
            struct row { int m_Parent; bone m_Bone; };
            std::vector<row> Rows;
            if (!File.isReading())
            {
                const auto Flatten = [&](auto&& Self, const bone& B, int Parent) -> void
                {
                    const int Me = static_cast<int>(Rows.size());
                    Rows.push_back({ Parent, B });
                    Rows.back().m_Bone.m_Bones.clear();
                    for (auto& C : B.m_Bones) Self(Self, C, Me);
                };
                Flatten(Flatten, Tree, -1);
            }

            if (auto Err = File.Record("BoneOverrides"
                , [&](std::size_t& Count, xerr&) { if (File.isReading()) Rows.resize(Count); else Count = Rows.size(); }
                , [&](std::size_t i, xerr& Err)
                {
                    auto& B     = Rows[i].m_Bone;
                    float Legacy[9] = {}; int LegacyLock = 0;
                    int   Type  = static_cast<int>(B.m_Type), Expose = B.m_bExpose, Delete = B.m_bDeleteBone;
                    false
                    || (Err = File.Field("Parent",   Rows[i].m_Parent))
                    || (Err = File.Field("Name",     B.m_Name))
                    || (Err = File.Field("Rename",   B.m_Rename))
                    || (Err = File.Field("Type",     Type))
                    || (Err = File.Field("Expose",   Expose))
                    || (Err = File.Field("Delete",   Delete))
                    || (Err = File.Field("LOD",      B.m_LODLevel))
                    || (bLegacyColumns && ((Err = File.Field("Scale",    Legacy[0], Legacy[1], Legacy[2]))        // the per-bone Transform of descriptors before 1.1: read and dropped
                                        || (Err = File.Field("Rotation", Legacy[3], Legacy[4], Legacy[5]))
                                        || (Err = File.Field("Position", Legacy[6], Legacy[7], Legacy[8]))
                                        || (Err = File.Field("Lock",     LegacyLock))));
                    B.m_Type = static_cast<bone_type>(Type); B.m_bExpose = Expose != 0; B.m_bDeleteBone = Delete != 0;
                }); Err) return Err;

            // The intents the user set, in their own record (only the bones off AUTO): a descriptor saved before intents existed has none
            {
                struct intent_row { int m_Row; int m_Intent; };
                std::vector<intent_row> Intents;
                if (!File.isReading())
                    for (std::size_t i = 0; i < Rows.size(); ++i)
                        if (Rows[i].m_Bone.m_Intent != bone_intent::AUTO) Intents.push_back({ static_cast<int>(i), static_cast<int>(Rows[i].m_Bone.m_Intent) });
                const bool bRead = File.isReading() && !File.isEOF() && File.getRecordName() == std::string_view("BoneIntents");
                if (!File.isReading() ? !Intents.empty() : bRead)
                {
                    if (auto Err = File.Record("BoneIntents"
                        , [&](std::size_t& Count, xerr&) { if (File.isReading()) Intents.resize(Count); else Count = Intents.size(); }
                        , [&](std::size_t i, xerr& Err) { false || (Err = File.Field("Row", Intents[i].m_Row)) || (Err = File.Field("Intent", Intents[i].m_Intent)); }); Err) return Err;
                    if (File.isReading())
                        for (auto& E : Intents)
                            if (E.m_Row >= 0 && E.m_Row < static_cast<int>(Rows.size())) Rows[E.m_Row].m_Bone.m_Intent = static_cast<bone_intent>(E.m_Intent);
                }
            }

            if (File.isReading())
            {
                for (int i = static_cast<int>(Rows.size()) - 1; i > 0; --i)    // pre-order: a child always comes after its parent
                    Rows[Rows[i].m_Parent].m_Bone.m_Bones.insert(Rows[Rows[i].m_Parent].m_Bone.m_Bones.begin(), std::move(Rows[i].m_Bone));
                if (!Rows.empty()) Tree = std::move(Rows[0].m_Bone);
            }
            return {};
        }

        xerr Serialize(bool isReading, std::wstring_view FileName, xproperty::settings::context& Context) noexcept override
        {
            xtextfile::stream Stream;
            if (auto Err = Stream.Open(isReading, FileName, xtextfile::file_type::TEXT); Err) return Err;
            if (!isReading) m_Version.m_Minor = 1;     // 1.1: a bone has no Transform of its own (nothing ever read it)
            if (auto Err = m_Version.SerializeVersion(Stream); Err) return Err;

            bone Tree;
            if (isReading) m_bCompactTree = false;
            else
            {
                m_bCompactTree = Depth(m_RootBone) > details::compact_depth_v;
                if (m_bCompactTree) std::swap(Tree, m_RootBone);
            }
            auto Err = xproperty::sprop::serializer::Stream<xproperty::settings::atomic_types_tuple>(Stream, *this, Context);
            if (!isReading && m_bCompactTree) std::swap(Tree, m_RootBone);
            if (Err) return Err;
            return m_bCompactTree ? StreamCompactTree(Stream, m_RootBone, isReading && m_Version.m_Minor < 1) : xerr{};
        }

        XPROPERTY_VDEF
        ( "Skeleton", descriptor
        , obj_member<"ImportAsset",     &descriptor::m_ImportAsset, member_ui<std::wstring>::file_dialog<mesh_filter_v, true, 1> >
        , obj_member<"PreTransform",    &descriptor::m_PreTransform >
        , obj_member<"PostTransform",   &descriptor::m_PostTransform, member_help<"Applied to the frozen pose (the rest of the root bone) before the PreTransform, and to the root bone of every frame of every clip of the animations that use this skeleton, in the same way. Use it when the frozen pose and the animations are rotated against the bind pose the skin is bound with (the model was rotated while it was animated). It does not move the bind pose or the skins: those follow the PreTransform only. It is baked in when they compile: no runtime cost.">>
        , obj_member<"RootBone",        &descriptor::m_RootBone >
        , obj_member<"MaskGroups",      &descriptor::m_MaskGroups >
        , obj_member<"CompactTree",     &descriptor::m_bCompactTree, member_flags<flags::DONT_SHOW> >
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
