#ifndef XSKELETON_MERGE_H
#define XSKELETON_MERGE_H
#pragma once

// The final bone list of a skeleton: the raw import merged with the descriptor's overrides. Shared by the Skeleton compiler and by the skin
// compiler, which makes the bone manifest of the skeleton it imports itself (that skeleton is compiled only after the skin), so that the two
// can never disagree on the bone order.
#include "dependencies/xraw3d/source/xraw3d.h"
#include "dependencies/xstrtool/source/xstrtool.h"
#include "plugins/xskeleton.plugin/source/xskeleton.h"
#include "plugins/xskeleton.plugin/source/xskeleton_descriptor.h"
#include "plugins/xskeleton.plugin/source/xskeleton_bone_manifest.h"

#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <functional>

namespace xskeleton_desc::merge
{
    //--------------------------------------------------------------------------------------
    // A node is a virtual bone purely by its (case-insensitive) "vbone_" name prefix. xraw3d's
    // importer already knows to include such nodes even with no skin weight at all (see the
    // gathering pass added to ImportSkeleton() in xraw3d_assimp_import_v3.h); here we decide
    // bone_type from the tag and strip it before it becomes the bone's real, stored name.
    inline bool IsVBoneTag(std::string_view Name) noexcept
    {
        constexpr std::string_view Tag = "vbone_";
        if (Name.size() < Tag.size()) return false;
        for (auto i = 0u; i < Tag.size(); ++i)
            if (std::tolower(static_cast<unsigned char>(Name[i])) != Tag[i]) return false;
        return true;
    }

    inline std::string StripVBoneTag(std::string_view Name) noexcept
    {
        return IsVBoneTag(Name) ? std::string(Name.substr(6)) : std::string(Name);
    }

    //--------------------------------------------------------------------------------------
    // The descriptor's bone tree is a sparse OVERRIDE layer, not a mirror of the imported
    // skeleton: a bone only needs an entry if you want to curate it (delete it, change its type,
    // expose it as a socket, cap its LOD). Anything not mentioned keeps its constructed defaults
    // (NORMAL unless vbone_-tagged, LOD 0, not deleted, not exposed) applied straight to the
    // freshly imported bind pose - no separate "seed the descriptor from import" step needed.
    inline void CollectOverrides(const bone& Node, std::unordered_map<std::string, const bone*>& Map)
    {
        if (Node.m_Name.empty() == false)
            Map[Node.m_Name] = &Node;

        for (auto& Child : Node.m_Bones)
            CollectOverrides(Child, Map);
    }

    //--------------------------------------------------------------------------------------

    struct merged_bone
    {
        std::string             m_Name;
        std::int32_t            m_iParent;
        xskeleton::bone_flags   m_Flags;
        int                     m_LODLevel;
        xmath::fvec3            m_BindScale;
        xmath::fquat            m_BindRotation;
        xmath::fvec3            m_BindTranslation;
        xmath::fmat4            m_InvBindPose;
    };

    //--------------------------------------------------------------------------------------
    // Drops deleted bones (re-parenting their children onto the nearest surviving ancestor),
    // applies type/LOD overrides, then enforces and sorts by the LOD invariant: a bone's LOD
    // level can never be lower than its parent's (a parent can't be a "more complex", higher-
    // detail-only bone than its own children) - clamp-and-warn if the descriptor says otherwise.
    template<typename T_STATE>
    xerr BuildMergedBones(const descriptor& Descriptor, const xraw3d::anim& Skeleton, std::vector<merged_bone>& Out, const std::function<void(std::string)>& Warning)
    {
        std::unordered_map<std::string, const bone*> Overrides;
        CollectOverrides(Descriptor.m_RootBone, Overrides);

        std::vector<int> RemapIndex(Skeleton.m_Bone.size(), -1);

        for (int i = 0; i < static_cast<int>(Skeleton.m_Bone.size()); ++i)
        {
            const auto& Src = Skeleton.m_Bone[i];
            const auto  It  = Overrides.find(Src.m_Name);
            const auto* pOv = (It == Overrides.end()) ? nullptr : It->second;

            if (pOv && pOv->m_bDeleteBone)
                continue;

            const bool bVirtual = pOv ? (pOv->m_Type == bone_type::VIRTUAL) : IsVBoneTag(Src.m_Name);
            const bool bSocket  = pOv && pOv->m_bExpose;

            merged_bone Bone;
            Bone.m_Name            = (pOv && !pOv->m_Rename.empty()) ? pOv->m_Rename : StripVBoneTag(Src.m_Name);
            Bone.m_Flags.m_bVirtual = bVirtual;
            Bone.m_Flags.m_bSocket  = bSocket;
            Bone.m_Flags.m_bJoint   = Src.m_Name.find("$AssimpFbx$") == std::string::npos;
            Bone.m_LODLevel        = pOv ? pOv->m_LODLevel : 0;
            // World-space bind inverse (used for skinning) - untouched by any reparenting below,
            // since that's specifically designed to preserve this bone's world bind pose exactly.
            Bone.m_InvBindPose     = Src.m_BindMatrixInv;

            // Re-parent onto the nearest surviving ancestor if anything in between got deleted.
            std::vector<int> Skipped; // deleted ancestors between Src and its surviving parent, child-to-root order
            {
                int iParent = Src.m_iParent;
                while (iParent != -1 && RemapIndex[iParent] == -1)
                {
                    Skipped.push_back(iParent);
                    iParent = Skeleton.m_Bone[iParent].m_iParent;
                }
                Bone.m_iParent = (iParent == -1) ? -1 : RemapIndex[iParent];
            }

            if (Skipped.empty())
            {
                Bone.m_BindScale       = Src.m_BindScale;
                Bone.m_BindRotation    = Src.m_BindRotation;
                Bone.m_BindTranslation = Src.m_BindTranslation;
            }
            else
            {
                // A deleted bone's own local bind transform doesn't just vanish - its surviving
                // children need it composed into THEIRS, or they visibly jump to a different world
                // position the moment their immediate parent is gone (their local transform alone
                // was only ever meaningful relative to that now-missing parent). Composed top-down
                // (the surviving ancestor's immediate child's local transform first) even though
                // Skipped was collected bottom-up, matching how a "world = parent * local" chain is
                // built, then Src's own local transform is applied last.
                xmath::fmat4 AccumLocal = xmath::fmat4::fromIdentity();
                for (auto It2 = Skipped.rbegin(); It2 != Skipped.rend(); ++It2)
                {
                    const auto& D = Skeleton.m_Bone[*It2];
                    AccumLocal = AccumLocal * xmath::fmat4(D.m_BindScale, D.m_BindRotation, D.m_BindTranslation);
                }

                const xmath::fmat4 SrcLocal(Src.m_BindScale, Src.m_BindRotation, Src.m_BindTranslation);
                const xmath::fmat4 NewLocal = AccumLocal * SrcLocal;
                Bone.m_BindScale       = NewLocal.ExtractScale();
                Bone.m_BindRotation    = NewLocal.ExtractRotation();
                Bone.m_BindTranslation = NewLocal.ExtractPosition();
            }

            RemapIndex[i] = static_cast<int>(Out.size());
            Out.emplace_back(std::move(Bone));
        }

        if (Out.empty())
            return xerr::create_f<T_STATE, "No bones survived import/deletion - nothing to compile">();

        // Deleting the root is fine as long as exactly one bone ends up parentless - e.g. a
        // "container" root with a single child (common for FBX's RootNode/Armature nodes) has an
        // unambiguous replacement root once it's gone. It's only actually a problem if MULTIPLE
        // children survive a deleted root (or a deleted chain leading to it), since there's no
        // principled way to pick which of them should become the new sole root - every downstream
        // consumer (see xskeleton.h) assumes exactly one, at index 0.
        if (const int nRoots = static_cast<int>(std::count_if(Out.begin(), Out.end(), [](const merged_bone& B) { return B.m_iParent == -1; })); nRoots != 1)
        {
            for (auto& B : Out) if (B.m_iParent == -1) Warning(std::format("Root bone left: '{}'", B.m_Name));
            return xerr::create_f<T_STATE, "Deleting these bones would leave the skeleton with more than one root bone with no single, unambiguous replacement - delete fewer bones along this chain, or keep one of them">();
        }

        for (auto& Bone : Out)
        {
            if (Bone.m_iParent == -1) continue;
            auto& Parent = Out[Bone.m_iParent];
            if (Bone.m_LODLevel < Parent.m_LODLevel)
            {
                Warning(std::format("Bone '{}' has a lower LOD level ({}) than its parent '{}' ({}); raising it to match.", Bone.m_Name, Bone.m_LODLevel, Parent.m_Name, Parent.m_LODLevel));
                Bone.m_LODLevel = Parent.m_LODLevel;
            }
        }

        // Stable sort by ascending LOD level. Since the list is already parent-before-child, and
        // every child's LOD level is now >= its parent's (just enforced above), a stable sort keeps
        // every parent before all of its children: equal-LOD ties preserve original (parent-first)
        // order, and a strictly-higher-LOD child always sorts after its (lower-LOD) parent's group.
        std::vector<int> NewOrder(Out.size());
        for (int i = 0; i < static_cast<int>(Out.size()); ++i) NewOrder[i] = i;
        std::stable_sort(NewOrder.begin(), NewOrder.end(), [&](int A, int B) { return Out[A].m_LODLevel < Out[B].m_LODLevel; });

        std::vector<int> FinalRemap(Out.size());
        for (int i = 0; i < static_cast<int>(NewOrder.size()); ++i) FinalRemap[NewOrder[i]] = i;

        std::vector<merged_bone> Sorted(Out.size());
        for (int i = 0; i < static_cast<int>(NewOrder.size()); ++i) Sorted[i] = std::move(Out[NewOrder[i]]);
        for (auto& Bone : Sorted)
            if (Bone.m_iParent != -1) Bone.m_iParent = FinalRemap[Bone.m_iParent];

        Out = std::move(Sorted);

        for (int i = 0; i < static_cast<int>(Out.size()); ++i)
            if (Out[i].m_iParent >= i)
                return xerr::create_f<T_STATE, "Internal error: LOD sort broke the parent-before-child invariant">();

        return {};
    }

    //--------------------------------------------------------------------------------------
    // Bone identity at runtime is a CRC32 of the name, not the name itself. Two different bones
    // hashing to the same value is a hard compile error - the user is expected to rename one.
    template<typename T_STATE>
    xerr AssignNameHashesAndCheckCollisions(const std::vector<merged_bone>& Bones, std::vector<std::uint32_t>& OutHashes)
    {
        OutHashes.resize(Bones.size());
        std::unordered_map<std::uint32_t, std::string> Seen;

        for (int i = 0; i < static_cast<int>(Bones.size()); ++i)
        {
            const auto Hash = xstrtool::CRC32(Bones[i].m_Name);
            if (auto It = Seen.find(Hash); It != Seen.end() && It->second != Bones[i].m_Name)
            {
                return xerr::create_f<T_STATE, "Two differently-named bones hash to the same identifier - rename one of them">();
            }
            Seen[Hash]    = Bones[i].m_Name;
            OutHashes[i]  = Hash;
        }
        return {};
    }

    //--------------------------------------------------------------------------------------
    // The final, post-merge bone order/hashes for downstream plugins (xanim_package.plugin, the
    // skin) that need to bind data to this exact layout without reading the compiled binary.
    inline void FillManifest(const descriptor& Descriptor, const std::vector<merged_bone>& Bones, const std::vector<std::uint32_t>& Hashes, bone_manifest& Manifest)
    {
        Manifest.m_Bones.clear();
        Manifest.m_Bones.reserve(Bones.size());
        for (int i = 0; i < static_cast<int>(Bones.size()); ++i)
            Manifest.m_Bones.push_back
            ( { .m_Name         = Bones[i].m_Name
              , .m_NameHash     = Hashes[i]
              , .m_iParent      = Bones[i].m_iParent
              , .m_RestScale    = Bones[i].m_BindScale
              , .m_RestRotX     = Bones[i].m_BindRotation.m_X
              , .m_RestRotY     = Bones[i].m_BindRotation.m_Y
              , .m_RestRotZ     = Bones[i].m_BindRotation.m_Z
              , .m_RestRotW     = Bones[i].m_BindRotation.m_W
              , .m_RestPosition = Bones[i].m_BindTranslation
              }
            );
        Manifest.m_NumBones = static_cast<int>(Bones.size());

        Manifest.m_PreTransformScale       = Descriptor.m_PreTransform.m_Scale;
        Manifest.m_PreTransformRotationDeg = Descriptor.m_PreTransform.m_Rotation;
        Manifest.m_PreTransformTranslation = Descriptor.m_PreTransform.m_Translation;
    }

    // The whole manifest of a skeleton without overrides (what an imported skeleton starts as): what the skin compiler binds its bones to
    template<typename T_STATE>
    xerr BuildManifest(const descriptor& Descriptor, const xraw3d::anim& Skeleton, bone_manifest& Manifest, const std::function<void(std::string)>& Warning)
    {
        std::vector<merged_bone>   Bones;
        std::vector<std::uint32_t> Hashes;
        if (auto Err = BuildMergedBones<T_STATE>(Descriptor, Skeleton, Bones, Warning); Err) return Err;
        if (auto Err = AssignNameHashesAndCheckCollisions<T_STATE>(Bones, Hashes); Err) return Err;
        FillManifest(Descriptor, Bones, Hashes, Manifest);
        return {};
    }
}

#endif
