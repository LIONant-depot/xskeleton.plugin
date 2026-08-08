#ifndef XSKELETON_BONE_MANIFEST_H
#define XSKELETON_BONE_MANIFEST_H
#pragma once

namespace xskeleton_desc
{
    // Compile-time export for downstream plugins that need to bind data to this skeleton's exact
    // bone layout (e.g. xanim_package.plugin's animation curves, keyed by bone name hash) without
    // reading this plugin's compiled binary. Unlike `details` (the RAW, pre-override import), this
    // is the FINAL, post-merge bone list/order/hashes - exactly what ends up in the compiled
    // xskeleton::skeleton::m_pBones/m_pBoneNames arrays, index for index. Written by the compiler to
    // this resource's own log folder as "AnimPackage.txt" (named after the consumer, matching the
    // convention xmaterial.plugin already uses for its own MaterialInstance.txt export) - see
    // xskeleton_compiler.cpp's onCompile().
    struct bone_manifest
    {
        struct bone_entry
        {
            std::string     m_Name      = {};   // final compiled name (post rename/merge/LOD-sort)
            std::uint32_t   m_NameHash  = {};   // same CRC32 baked into xskeleton::skeleton::name::m_NameHash

            XPROPERTY_DEF
            ( "bone_entry", bone_entry
            , obj_member<"Name",     &bone_entry::m_Name,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"NameHash", &bone_entry::m_NameHash, member_flags<flags::SHOW_READONLY> >
            )
        };

        std::vector<bone_entry>    m_Bones    = {};   // final compiled bone order - index i matches xskeleton::skeleton::m_pBones[i]
        int                        m_NumBones = 0;

        XPROPERTY_DEF
        ( "bone_manifest", bone_manifest
        , obj_member<"Bones",     &bone_manifest::m_Bones,    member_flags<flags::SHOW_READONLY> >
        , obj_member<"NumBones",  &bone_manifest::m_NumBones, member_flags<flags::SHOW_READONLY> >
        )
    };
    XPROPERTY_REG(bone_manifest)
    XPROPERTY_REG2(anim_bone_entry_, bone_manifest::bone_entry)
}

#endif
