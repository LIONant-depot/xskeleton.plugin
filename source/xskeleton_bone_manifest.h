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
            std::string     m_Name          = {};   // final compiled name (post rename/merge/LOD-sort)
            std::uint32_t   m_NameHash      = {};   // same CRC32 baked into xskeleton::skeleton::name::m_NameHash

            // Same semantics as xskeleton::skeleton::bone::m_iParent (-1 = no parent, topologically
            // sorted so a parent's index is always < its own). A downstream consumer needs this to
            // find THIS skeleton's own root bone directly - the equivalent bone in a raw animation
            // import is often NOT the raw file's own m_iParent==-1 bone (assimp/FBX frequently injects
            // a synthetic "RootNode" wrapper above the real, named root), so guessing from the raw
            // clip's own hierarchy silently picks the wrong bone (or none at all).
            std::int32_t    m_iParent       = -1;

            // Final compiled bind/rest-pose local transform, same fields/units as
            // xskeleton::skeleton::rest::m_RestPose - carried here so downstream consumers (e.g.
            // xanim_package.plugin) can fall back to this skeleton's own rest pose for any bone a
            // given clip doesn't animate, without reading this plugin's compiled binary.
            // Rotation is stored as raw quaternion components (X/Y/Z/W), not xmath::fquat itself -
            // xmath::fquat has no XPROPERTY registration anywhere in this codebase (only fvec2/fvec3/
            // fbbox are bridged, see dependencies/xmath/source/bridge/xmath_to_xproperty.h), matching
            // the same field-by-field convention xskeleton.h's own binary serializer already uses.
            xmath::fvec3    m_RestScale     = {};
            float           m_RestRotX      = {};
            float           m_RestRotY      = {};
            float           m_RestRotZ      = {};
            float           m_RestRotW      = {};
            xmath::fvec3    m_RestPosition  = {};

            XPROPERTY_DEF
            ( "bone_entry", bone_entry
            , obj_member<"Name",         &bone_entry::m_Name,         member_flags<flags::SHOW_READONLY> >
            , obj_member<"NameHash",     &bone_entry::m_NameHash,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"iParent",      &bone_entry::m_iParent,      member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestScale",    &bone_entry::m_RestScale,    member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestRotX",     &bone_entry::m_RestRotX,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestRotY",     &bone_entry::m_RestRotY,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestRotZ",     &bone_entry::m_RestRotZ,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestRotW",     &bone_entry::m_RestRotW,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"RestPosition", &bone_entry::m_RestPosition, member_flags<flags::SHOW_READONLY> >
            )
        };

        std::vector<bone_entry>    m_Bones    = {};   // final compiled bone order - index i matches xskeleton::skeleton::m_pBones[i]
        int                        m_NumBones = 0;

        // The descriptor's own author-time PreTransform (xskeleton_desc::descriptor::m_PreTransform) -
        // exported RAW (scale, Euler degrees, translation - same units/fields as the descriptor's own
        // `transform` struct) rather than baked in, because it needs to be applied to a downstream
        // consumer's OWN animated data, not to anything already in this manifest. The skeleton
        // compiler applies this correction to its root bone's bind pose before anything else runs
        // (see xskeleton_compiler.cpp's ApplyPreTransform) - every bone entry above already reflects
        // that. But xanim_package.plugin imports its OWN, separate raw animation curves, which never
        // pass through this skeleton's import/PreTransform step at all - it needs the raw values here
        // to apply the identical correction to its own animated root bone, or the animated skeleton
        // renders at the pre-correction scale/offset while its rest pose (and every unanimated bone's
        // fallback) is already correct, a visible scale/position mismatch between the two.
        xmath::fvec3               m_PreTransformScale       = xmath::fvec3::fromOne();
        xmath::fvec3               m_PreTransformRotationDeg = xmath::fvec3::fromZero();
        xmath::fvec3               m_PreTransformTranslation = xmath::fvec3::fromZero();

        XPROPERTY_DEF
        ( "bone_manifest", bone_manifest
        , obj_member<"Bones",                     &bone_manifest::m_Bones,                     member_flags<flags::SHOW_READONLY> >
        , obj_member<"NumBones",                  &bone_manifest::m_NumBones,                  member_flags<flags::SHOW_READONLY> >
        , obj_member<"PreTransformScale",         &bone_manifest::m_PreTransformScale,         member_flags<flags::SHOW_READONLY> >
        , obj_member<"PreTransformRotationDeg",   &bone_manifest::m_PreTransformRotationDeg,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"PreTransformTranslation",   &bone_manifest::m_PreTransformTranslation,   member_flags<flags::SHOW_READONLY> >
        )
    };
    XPROPERTY_REG(bone_manifest)
    XPROPERTY_REG2(anim_bone_entry_, bone_manifest::bone_entry)
}

#endif
