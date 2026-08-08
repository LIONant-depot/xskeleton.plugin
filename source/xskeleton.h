#ifndef XSKELETON_RUNTIME_H
#define XSKELETON_RUNTIME_H
#pragma once

#include "dependencies/xmath/source/xmath_fmat4.h"
#include "dependencies/xmath/source/xmath_ftransform.h"
#include "dependencies/xserializer/source/xserializer.h"
#include <span>

namespace xskeleton
{
    // Per-bone flags, packed into one byte (see bone's own "hot, cache-tight" comment) - independent
    // bits, not a mutually-exclusive enum, so a bone can be both at once (e.g. a weapon-socket bone
    // that's also VIRTUAL, since it has no animation curve of its own - it's driven entirely by
    // whoever holds it).
    // VIRTUAL - Never receives curve data from the animation system. Its pose is filled in by whoever
    //           needs it (a weighted average of other bones, physics/IK, or it's left at
    //           rest::m_RestPose as a static attachment point). The core animation evaluator must
    //           leave VIRTUAL slots untouched. Unset = expected to receive animation curve data.
    // SOCKET  - Exposed as an attachment point for external entities (weapons, props, hats, etc.) -
    //           see xskeleton_desc::bone::m_bExpose, which is where this gets authored.
    union bone_flags
    {
        enum mask : std::uint8_t
        { VIRTUAL   = (1 << 0)
        , SOCKET    = (1 << 1)
        , DEFAULT   = 0
        };

        std::uint8_t m_Value = mask::DEFAULT;

        struct
        {
            std::uint8_t
                  m_bVirtual : 1
                , m_bSocket  : 1
                ;
        };
    };

    struct skeleton
    {
        inline static constexpr auto xserializer_version_v = 1;

        // Hot: read every frame, for every bone, while walking the hierarchy and building skin matrices.
        // Fields ordered by alignment (largest/most-aligned first) to keep this struct - and the cache
        // lines the hot loop touches - as tight as possible. Bone identity (name) and rest pose (only
        // needed for LOD-frozen/VIRTUAL bones) live in their own parallel arrays below instead of being
        // interleaved in here.
        struct bone
        {
            xmath::fmat4       m_InvBindPose   {};                        // World-space inverse bind, for the final skinning matrix.
            std::int16_t       m_iParent       {-1};                      // Always < this bone's own index; -1 only for bone 0 (the sole root).
            bone_flags         m_Flags         {};
        };

        // Cold, parallel to bone[]: only touched for name-based lookups (retargeting, tools, debug),
        // never in the per-frame hierarchy/skinning loop.
        struct name
        {
            std::uint32_t    m_NameHash  {};   // CRC32 of the authored name. Unique within this skeleton (the compiler errors out on a collision rather than silently aliasing two bones).
        };

        // Cold, parallel to bone[]: only touched for bones past their LOD cutoff (frozen at this pose),
        // for a VIRTUAL bone's default value, or when first initializing a fresh pose buffer.
        struct rest
        {
            xmath::transform3    m_RestPose  {};
        };

        // One fixed-point [0,1] blend weight (0 = 0x0000, 1 = 0xFFFF) for a single (group, bone) pair.
        // Masking/blending math itself belongs to the animation system - the skeleton only owns the
        // name -> weight-table mapping.
        struct mask
        {
            std::uint16_t    m_Mask  {};
        };

        // A named per-bone weight table (e.g. "UpperBody", "LowerBody"). Always exactly m_nBones wide,
        // so its offset into skeleton::m_pMasks is just index * m_nBones - no stored offset/count needed.
        struct mask_group
        {
            std::uint32_t    m_NameHash  {};
        };

        //-------------------------------------------------------------------------

                                                skeleton                (void)                                  noexcept = default;
        inline                                  skeleton                (xserializer::stream& Stream)           noexcept;
        inline void                             Kill                    (void)                                  noexcept;
        inline void                             Initialize              (void)                                  noexcept;
        inline int                              findBoneIndex           (std::uint32_t NameHash)         const   noexcept;
        inline int                              findMaskGroupIndex      (std::uint32_t NameHash)         const   noexcept;
        inline std::span<bone>                  getBones                (void)                           const   noexcept { return { m_pBones,     m_nBones }; }
        inline std::span<name>                  getBoneNames            (void)                           const   noexcept { return { m_pBoneNames, m_nBones }; }
        inline std::span<rest>                  getBoneRests            (void)                           const   noexcept { return { m_pBoneRests, m_nBones }; }
        inline std::span<std::uint16_t>         getLODBoneCounts        (void)                           const   noexcept { return { m_pLODBoneCount, m_nLODs }; }
        inline std::span<mask_group>            getMaskGroups           (void)                           const   noexcept { return { m_pMaskGroups, m_nMaskGroups }; }
        inline std::span<mask>                  getMaskWeights          (int iGroup)                     const   noexcept { return { m_pMasks + static_cast<std::size_t>(iGroup) * m_nBones, m_nBones }; }

        bone*                m_pBones            {nullptr};   // m_nBones entries. Hot - see bone's own comment.
        name*                m_pBoneNames        {nullptr};   // m_nBones entries, parallel to m_pBones.
        rest*                m_pBoneRests        {nullptr};   // m_nBones entries, parallel to m_pBones.
        std::uint16_t*       m_pLODBoneCount     {nullptr};   // m_nLODs entries; LOD L uses bones [0, m_pLODBoneCount[L])
        mask_group*          m_pMaskGroups       {nullptr};   // m_nMaskGroups entries
        mask*                m_pMasks            {nullptr};   // m_nMaskGroups * m_nBones entries; group g's weights are m_pMasks[g*m_nBones .. (g+1)*m_nBones)
        std::uint16_t         m_nBones            {0};
        std::uint16_t         m_nLODs             {0};
        std::uint16_t         m_nMaskGroups       {0};
    };

    //-------------------------------------------------------------------------

    skeleton::skeleton(xserializer::stream& Stream) noexcept
    {
        //xassert( Stream.getResourceVersion() == skeleton::xserializer_version_v );
    }

    //-------------------------------------------------------------------------

    void skeleton::Initialize(void) noexcept
    {
        std::memset(this, 0, sizeof(*this));
    }

    //-------------------------------------------------------------------------

    void skeleton::Kill(void) noexcept
    {
        if (m_pBones)           delete[] m_pBones;
        if (m_pBoneNames)       delete[] m_pBoneNames;
        if (m_pBoneRests)       delete[] m_pBoneRests;
        if (m_pLODBoneCount)    delete[] m_pLODBoneCount;
        if (m_pMaskGroups)      delete[] m_pMaskGroups;
        if (m_pMasks)           delete[] m_pMasks;

        Initialize();
    }

    //-------------------------------------------------------------------------

    int skeleton::findBoneIndex(std::uint32_t NameHash) const noexcept
    {
        for (auto i = 0u; i < m_nBones; ++i)
            if (m_pBoneNames[i].m_NameHash == NameHash) return static_cast<int>(i);
        return -1;
    }

    //-------------------------------------------------------------------------

    int skeleton::findMaskGroupIndex(std::uint32_t NameHash) const noexcept
    {
        for (auto i = 0u; i < m_nMaskGroups; ++i)
            if (m_pMaskGroups[i].m_NameHash == NameHash) return static_cast<int>(i);
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
    xerr SerializeIO<xskeleton::skeleton::bone>(xserializer::stream& Stream, const xskeleton::skeleton::bone& Bone) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Bone.m_InvBindPose.m_Elements))
            || (Err = Stream.Serialize(Bone.m_iParent))
            || (Err = Stream.Serialize(Bone.m_Flags.m_Value))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xskeleton::skeleton::name>(xserializer::stream& Stream, const xskeleton::skeleton::name& Name) noexcept
    {
        return Stream.Serialize(Name.m_NameHash);
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xskeleton::skeleton::rest>(xserializer::stream& Stream, const xskeleton::skeleton::rest& Rest) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Scale.m_X))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Scale.m_Y))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Scale.m_Z))

            || (Err = Stream.Serialize(Rest.m_RestPose.m_Rotation.m_X))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Rotation.m_Y))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Rotation.m_Z))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Rotation.m_W))

            || (Err = Stream.Serialize(Rest.m_RestPose.m_Position.m_X))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Position.m_Y))
            || (Err = Stream.Serialize(Rest.m_RestPose.m_Position.m_Z))
            ;
        return Err;
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xskeleton::skeleton::mask>(xserializer::stream& Stream, const xskeleton::skeleton::mask& Mask) noexcept
    {
        return Stream.Serialize(Mask.m_Mask);
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xskeleton::skeleton::mask_group>(xserializer::stream& Stream, const xskeleton::skeleton::mask_group& Group) noexcept
    {
        return Stream.Serialize(Group.m_NameHash);
    }

    //-------------------------------------------------------------------------
    template<> inline
    xerr SerializeIO<xskeleton::skeleton>(xserializer::stream& Stream, const xskeleton::skeleton& Skeleton) noexcept
    {
        xerr Err;
        false
            || (Err = Stream.Serialize(Skeleton.m_nBones))
            || (Err = Stream.Serialize(Skeleton.m_pBones,          Skeleton.m_nBones))
            || (Err = Stream.Serialize(Skeleton.m_pBoneNames,      Skeleton.m_nBones))
            || (Err = Stream.Serialize(Skeleton.m_pBoneRests,      Skeleton.m_nBones))
            || (Err = Stream.Serialize(Skeleton.m_nLODs))
            || (Err = Stream.Serialize(Skeleton.m_pLODBoneCount,   Skeleton.m_nLODs))
            || (Err = Stream.Serialize(Skeleton.m_nMaskGroups))
            || (Err = Stream.Serialize(Skeleton.m_pMaskGroups,     Skeleton.m_nMaskGroups))
            || (Err = Stream.Serialize(Skeleton.m_pMasks,          static_cast<std::size_t>(Skeleton.m_nMaskGroups) * Skeleton.m_nBones))
            ;
        return Err;
    }
}

#endif
