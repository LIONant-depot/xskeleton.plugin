#include "xskeleton_compiler.h"

#include "dependencies/xraw3d/source/xraw3d.h"
#include "dependencies/xraw3d/source/details/xraw3d_assimp_import_v3.h"

#include "dependencies/xstrtool/source/xstrtool.h"

#include "../xskeleton_descriptor.h"
#include "../xskeleton.h"
#include "../xskeleton_details.h"
#include "../xskeleton_bone_manifest.h"

#include "dependencies/xproperty/source/xcore/my_properties.cpp"
#include "dependencies/xmath/source/bridge/xmath_to_xproperty.h"

#include <unordered_map>
#include <algorithm>
#include <cctype>

namespace xskeleton_compiler
{
    struct implementation : xskeleton_compiler::instance
    {
        //--------------------------------------------------------------------------------------

        implementation()
        {
            m_FinalSkeleton.Initialize();
        }

        //--------------------------------------------------------------------------------------
        // A node is a virtual bone purely by its (case-insensitive) "vbone_" name prefix. xraw3d's
        // importer already knows to include such nodes even with no skin weight at all (see the
        // gathering pass added to ImportSkeleton() in xraw3d_assimp_import_v3.h); here we decide
        // bone_type from the tag and strip it before it becomes the bone's real, stored name.
        static bool IsVBoneTag(std::string_view Name) noexcept
        {
            constexpr std::string_view Tag = "vbone_";
            if (Name.size() < Tag.size()) return false;
            for (auto i = 0u; i < Tag.size(); ++i)
                if (std::tolower(static_cast<unsigned char>(Name[i])) != Tag[i]) return false;
            return true;
        }

        static std::string StripVBoneTag(std::string_view Name) noexcept
        {
            return IsVBoneTag(Name) ? std::string(Name.substr(6)) : std::string(Name);
        }

        //--------------------------------------------------------------------------------------

        xerr LoadRaw(const std::wstring_view Path)
        {
            xraw3d::assimp_v3::importer           Importer;
            xraw3d::assimp_v3::importer::settings Settings;

            Settings.m_bStaticGeometry = false;
            Settings.m_pSkeleton       = &m_Skeleton;

            if (auto Err = Importer.Import(Path, Settings); Err)
                return xerr::create_f<state, "Failed to import the asset">(Err);

            return {};
        }

        //--------------------------------------------------------------------------------------
        // The root's LOCAL translation/rotation/scale is the only place the correction needs to be
        // applied directly for the FROZEN/rest pose - every descendant inherits it for free through
        // the normal parent-multiply FK chain when that pose gets reconstructed.
        //
        // m_BindMatrixInv does NOT get that same free ride: it's each bone's own WORLD-space bind
        // inverse (used directly for skinning/the BIND pose view, never recomposed through the
        // hierarchy at runtime), so prepending an outer transform above the root leaves every OTHER
        // bone's world bind pose exactly where it was. Only the root's own inverse was being
        // corrected here, leaving the BIND pose - and the root-sphere/framing radius derived from it -
        // still reflecting the un-transformed asset for every bone below the root while the FROZEN
        // pose (and the rest of the UI) had already moved on: the visible symptom was a correctly
        // shrunk/repositioned skeleton with a wildly oversized root sphere, since its size is derived
        // from the overall bounding radius across BOTH pose views.
        void ApplyPreTransform()
        {
            const auto& PT = m_Descriptor.m_PreTransform;
            if (   PT.m_Scale       == xmath::fvec3::fromOne()
                && PT.m_Rotation    == xmath::fvec3::fromZero()
                && PT.m_Translation == xmath::fvec3::fromZero() )
                return;

            xmath::radian3 Rot;
            Rot.m_Roll  = xmath::radian{ xmath::DegToRad(PT.m_Rotation.m_Z) };
            Rot.m_Pitch = xmath::radian{ xmath::DegToRad(PT.m_Rotation.m_X) };
            Rot.m_Yaw   = xmath::radian{ xmath::DegToRad(PT.m_Rotation.m_Y) };

            const xmath::fquat PreQuat(Rot);

            // fmat4's 3-arg ctor is (scale, rotation, translation) despite the header's parameter
            // names claiming (translation, rotation, scale) - verified against the .inl implementation.
            const xmath::fmat4 M(PT.m_Scale, PreQuat, PT.m_Translation);
            const xmath::fmat4 MInv = M.Inverse();

            for (auto& Bone : m_Skeleton.m_Bone)
            {
                // Every bone with REAL bind data shifts its world bind inverse by the same outer M,
                // root or not - but an identity m_BindMatrixInv is a SENTINEL meaning "this bone never
                // had real skin data, fall back to the rest-pose FK chain instead" (see
                // ComputeBoneWorldsAndFraming in the editor). Multiplying that identity by MInv turns
                // it into a plausible-looking but meaningless matrix, silently breaking the sentinel:
                // every such bone then reported the SAME single point (wherever M places the origin)
                // as its BIND position instead of falling back, producing the degenerate/collapsed
                // geometry seen in the BIND pose view.
                if (!Bone.m_BindMatrixInv.isIdentity())
                    Bone.m_BindMatrixInv = Bone.m_BindMatrixInv * MInv;

                if (Bone.m_iParent != -1) continue;

                Bone.m_BindTranslation = M * Bone.m_BindTranslation;
                Bone.m_BindRotation    = PreQuat * Bone.m_BindRotation;
                Bone.m_BindScale       = PT.m_Scale * Bone.m_BindScale;
            }
        }

        //--------------------------------------------------------------------------------------
        // The descriptor's bone tree is a sparse OVERRIDE layer, not a mirror of the imported
        // skeleton: a bone only needs an entry if you want to curate it (delete it, change its type,
        // expose it as a socket, cap its LOD). Anything not mentioned keeps its constructed defaults
        // (NORMAL unless vbone_-tagged, LOD 0, not deleted, not exposed) applied straight to the
        // freshly imported bind pose - no separate "seed the descriptor from import" step needed.
        void CollectOverrides(const xskeleton_desc::bone& Node, std::unordered_map<std::string, const xskeleton_desc::bone*>& Map)
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
        xerr BuildMergedBones(std::vector<merged_bone>& Out)
        {
            std::unordered_map<std::string, const xskeleton_desc::bone*> Overrides;
            CollectOverrides(m_Descriptor.m_RootBone, Overrides);

            std::vector<int> RemapIndex(m_Skeleton.m_Bone.size(), -1);

            for (int i = 0; i < static_cast<int>(m_Skeleton.m_Bone.size()); ++i)
            {
                const auto& Src = m_Skeleton.m_Bone[i];
                const auto  It  = Overrides.find(Src.m_Name);
                const auto* pOv = (It == Overrides.end()) ? nullptr : It->second;

                if (pOv && pOv->m_bDeleteBone)
                    continue;

                const bool bVirtual = pOv ? (pOv->m_Type == xskeleton_desc::bone_type::VIRTUAL) : IsVBoneTag(Src.m_Name);
                const bool bSocket  = pOv && pOv->m_bExpose;

                merged_bone Bone;
                Bone.m_Name            = (pOv && !pOv->m_Rename.empty()) ? pOv->m_Rename : StripVBoneTag(Src.m_Name);
                Bone.m_Flags.m_bVirtual = bVirtual;
                Bone.m_Flags.m_bSocket  = bSocket;
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
                        iParent = m_Skeleton.m_Bone[iParent].m_iParent;
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
                        const auto& D = m_Skeleton.m_Bone[*It2];
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
                return xerr::create_f<state, "No bones survived import/deletion - nothing to compile">();

            // Deleting the root is fine as long as exactly one bone ends up parentless - e.g. a
            // "container" root with a single child (common for FBX's RootNode/Armature nodes) has an
            // unambiguous replacement root once it's gone. It's only actually a problem if MULTIPLE
            // children survive a deleted root (or a deleted chain leading to it), since there's no
            // principled way to pick which of them should become the new sole root - every downstream
            // consumer (see xskeleton.h) assumes exactly one, at index 0.
            if (const int nRoots = static_cast<int>(std::count_if(Out.begin(), Out.end(), [](const merged_bone& B) { return B.m_iParent == -1; })); nRoots != 1)
                return xerr::create_f<state, "Deleting these bones would leave the skeleton with more than one root bone with no single, unambiguous replacement - delete fewer bones along this chain, or keep one of them">();

            for (auto& Bone : Out)
            {
                if (Bone.m_iParent == -1) continue;
                auto& Parent = Out[Bone.m_iParent];
                if (Bone.m_LODLevel < Parent.m_LODLevel)
                {
                    LogMessage(xresource_pipeline::msg_type::WARNING
                        , std::format("Bone '{}' has a lower LOD level ({}) than its parent '{}' ({}); raising it to match.", Bone.m_Name, Bone.m_LODLevel, Parent.m_Name, Parent.m_LODLevel));
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
                    return xerr::create_f<state, "Internal error: LOD sort broke the parent-before-child invariant">();

            return {};
        }

        //--------------------------------------------------------------------------------------
        // Bone identity at runtime is a CRC32 of the name, not the name itself. Two different bones
        // hashing to the same value is a hard compile error - the user is expected to rename one.
        xerr AssignNameHashesAndCheckCollisions(const std::vector<merged_bone>& Bones, std::vector<std::uint32_t>& OutHashes)
        {
            OutHashes.resize(Bones.size());
            std::unordered_map<std::uint32_t, std::string> Seen;

            for (int i = 0; i < static_cast<int>(Bones.size()); ++i)
            {
                const auto Hash = xstrtool::CRC32(Bones[i].m_Name);
                if (auto It = Seen.find(Hash); It != Seen.end() && It->second != Bones[i].m_Name)
                {
                    return xerr::create_f<state, "Two differently-named bones hash to the same identifier - rename one of them">();
                }
                Seen[Hash]    = Bones[i].m_Name;
                OutHashes[i]  = Hash;
            }
            return {};
        }

        //--------------------------------------------------------------------------------------
        // LOD L is active for bones[0, OutCounts[L]) - the bound is just "how many bones have an
        // LOD level <= L", since the list is already sorted by ascending LOD level.
        void BuildLODTable(const std::vector<merged_bone>& Bones, std::vector<std::uint16_t>& OutCounts)
        {
            int MaxLevel = 0;
            for (auto& B : Bones) MaxLevel = std::max(MaxLevel, B.m_LODLevel);

            OutCounts.resize(MaxLevel + 1);
            for (int L = 0; L <= MaxLevel; ++L)
            {
                int Count = 0;
                for (auto& B : Bones) if (B.m_LODLevel <= L) ++Count;
                OutCounts[L] = static_cast<std::uint16_t>(Count);
            }
        }

        //--------------------------------------------------------------------------------------
        // Each mask group becomes one dense, fixed-point [0,1] weight per bone in this skeleton.
        // A mask entry naming a bone that no longer exists (deleted, renamed, typo) is a warning,
        // not a hard failure - the rest of the group still compiles.
        void BuildMasks(const std::vector<merged_bone>& Bones, const std::unordered_map<std::string, int>& NameToIndex)
        {
            m_FinalSkeleton.m_nMaskGroups = static_cast<std::uint16_t>(m_Descriptor.m_MaskGroups.size());
            if (m_FinalSkeleton.m_nMaskGroups == 0) return;

            m_FinalSkeleton.m_pMaskGroups = new xskeleton::skeleton::mask_group[m_FinalSkeleton.m_nMaskGroups];
            m_FinalSkeleton.m_pMasks      = new xskeleton::skeleton::mask[static_cast<std::size_t>(m_FinalSkeleton.m_nMaskGroups) * Bones.size()];

            for (int g = 0; g < static_cast<int>(m_Descriptor.m_MaskGroups.size()); ++g)
            {
                auto& Group = m_Descriptor.m_MaskGroups[g];
                m_FinalSkeleton.m_pMaskGroups[g].m_NameHash = xstrtool::CRC32(Group.m_Name);

                auto* pWeights = m_FinalSkeleton.m_pMasks + static_cast<std::size_t>(g) * Bones.size();
                for (std::size_t b = 0; b < Bones.size(); ++b) pWeights[b].m_Mask = 0;

                for (auto& Entry : Group.m_Entries)
                {
                    auto It = NameToIndex.find(Entry.m_BoneName);
                    if (It == NameToIndex.end())
                    {
                        LogMessage(xresource_pipeline::msg_type::WARNING
                            , std::format("Mask group '{}' references unknown bone '{}'", Group.m_Name, Entry.m_BoneName));
                        continue;
                    }

                    const float W = std::clamp(Entry.m_Weight, 0.0f, 1.0f);
                    pWeights[It->second].m_Mask = static_cast<std::uint16_t>(std::lround(W * 65535.0f));
                }
            }
        }

        //--------------------------------------------------------------------------------------

        xerr BuildFinalSkeleton()
        {
            std::vector<merged_bone> Bones;
            if (auto Err = BuildMergedBones(Bones); Err) return Err;

            std::vector<std::uint32_t> Hashes;
            if (auto Err = AssignNameHashesAndCheckCollisions(Bones, Hashes); Err) return Err;

            // Export the final, post-merge bone order/hashes for downstream plugins (e.g.
            // xanim_package.plugin) that need to bind data to this exact layout without reading our
            // compiled binary - see xskeleton_bone_manifest.h and onCompile()'s AnimPackage.txt write.
            m_BoneManifest.m_Bones.clear();
            m_BoneManifest.m_Bones.reserve(Bones.size());
            for (int i = 0; i < static_cast<int>(Bones.size()); ++i)
                m_BoneManifest.m_Bones.push_back
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
            m_BoneManifest.m_NumBones = static_cast<int>(Bones.size());

            m_BoneManifest.m_PreTransformScale       = m_Descriptor.m_PreTransform.m_Scale;
            m_BoneManifest.m_PreTransformRotationDeg = m_Descriptor.m_PreTransform.m_Rotation;
            m_BoneManifest.m_PreTransformTranslation = m_Descriptor.m_PreTransform.m_Translation;

            std::vector<std::uint16_t> LODCounts;
            BuildLODTable(Bones, LODCounts);

            std::unordered_map<std::string, int> NameToIndex;
            for (int i = 0; i < static_cast<int>(Bones.size()); ++i) NameToIndex[Bones[i].m_Name] = i;

            m_FinalSkeleton.m_nBones     = static_cast<std::uint16_t>(Bones.size());
            m_FinalSkeleton.m_pBones     = new xskeleton::skeleton::bone[Bones.size()];
            m_FinalSkeleton.m_pBoneNames = new xskeleton::skeleton::name[Bones.size()];
            m_FinalSkeleton.m_pBoneRests = new xskeleton::skeleton::rest[Bones.size()];

            for (int i = 0; i < static_cast<int>(Bones.size()); ++i)
            {
                auto& Src = Bones[i];

                m_FinalSkeleton.m_pBones[i].m_InvBindPose = Src.m_InvBindPose;
                m_FinalSkeleton.m_pBones[i].m_iParent     = static_cast<std::int16_t>(Src.m_iParent);
                m_FinalSkeleton.m_pBones[i].m_Flags       = Src.m_Flags;

                m_FinalSkeleton.m_pBoneNames[i].m_NameHash = Hashes[i];

                m_FinalSkeleton.m_pBoneRests[i].m_RestPose.m_Scale    = Src.m_BindScale;
                m_FinalSkeleton.m_pBoneRests[i].m_RestPose.m_Rotation = Src.m_BindRotation;
                m_FinalSkeleton.m_pBoneRests[i].m_RestPose.m_Position = Src.m_BindTranslation;
            }

            m_FinalSkeleton.m_nLODs        = static_cast<std::uint16_t>(LODCounts.size());
            m_FinalSkeleton.m_pLODBoneCount = new std::uint16_t[LODCounts.size()];
            std::copy(LODCounts.begin(), LODCounts.end(), m_FinalSkeleton.m_pLODBoneCount);

            BuildMasks(Bones, NameToIndex);

            return {};
        }

        //--------------------------------------------------------------------------------------
        // Editor-facing view of what was actually imported - unaffected by descriptor overrides,
        // since it's meant to show the raw truth of the source asset, not the curated result.
        void ComputeDetailStructure()
        {
            m_Details.m_BoneList.clear();
            for (auto& B : m_Skeleton.m_Bone)
                m_Details.m_BoneList.push_back(B.m_Name);
            m_Details.m_NumBones = static_cast<int>(m_Skeleton.m_Bone.size());

            std::function<void(xskeleton_desc::details::bone&, int)> Build = [&](xskeleton_desc::details::bone& Dst, int iSrc)
            {
                Dst.m_Name = m_Skeleton.m_Bone[iSrc].m_Name;
                for (int i = 0; i < static_cast<int>(m_Skeleton.m_Bone.size()); ++i)
                    if (m_Skeleton.m_Bone[i].m_iParent == iSrc)
                        Build(Dst.m_Children.emplace_back(), i);
            };

            for (int i = 0; i < static_cast<int>(m_Skeleton.m_Bone.size()); ++i)
            {
                if (m_Skeleton.m_Bone[i].m_iParent == -1)
                {
                    Build(m_Details.m_RootBone, i);
                    break;
                }
            }
        }

        //--------------------------------------------------------------------------------------

        xerr onCompile(void) noexcept override
        {
            //
            // Read the descriptor file...
            //
            displayProgressBar("Loading Descriptor", 0);
            {
                xproperty::settings::context    Context{};
                auto                             DescriptorFileName = std::format(L"{}/{}/Descriptor.txt", m_ProjectPaths.m_Project, m_InputSrcDescriptorPath);

                if (auto Err = m_Descriptor.Serialize(true, DescriptorFileName, Context); Err)
                    return Err;
            }

            //
            // Do a quick validation of the descriptor
            //
            {
                std::vector<std::string> Errors;
                m_Descriptor.Validate(Errors);
                if (not Errors.empty())
                {
                    for (auto& E : Errors)
                        LogMessage(xresource_pipeline::msg_type::ERROR, std::move(E));

                    return xerr::create_f<state, "Validation Errors">();
                }
            }
            displayProgressBar("Loading Descriptor", 1);

            //
            // Load the source data
            //
            displayProgressBar("Importing Skeleton", 0);
            if (auto Err = LoadRaw(std::format(L"{}/{}", m_ProjectPaths.m_Project, m_Descriptor.m_ImportAsset)); Err)
                return Err;
            displayProgressBar("Importing Skeleton", 1);

            //
            // Fill the detail structure (raw import, pre-curation - for editor display only)
            //
            ComputeDetailStructure();

            //
            // OK, time to compile
            //
            try
            {
                ApplyPreTransform();

                m_FinalSkeleton.Initialize();

                displayProgressBar("Building Skeleton", 0);
                if (auto Err = BuildFinalSkeleton(); Err)
                    return Err;
                displayProgressBar("Building Skeleton", 1);
            }
            catch (std::runtime_error Error)
            {
                LogMessage(xresource_pipeline::msg_type::ERROR, std::format("{}", Error.what()));
                return xerr::create_f<state, "Exception thrown">();
            }

            //
            // Serialize the details structure
            //
            {
                xtextfile::stream File;
                if (auto Err = File.Open(false, std::format(L"{}\\Details.txt", m_ResourceLogPath), xtextfile::file_type::TEXT); Err)
                    return xerr::create_f<state, "Failed while opening the details.txt so it can't be saved">(Err);

                xproperty::settings::context C{};
                if (auto Err = xproperty::sprop::serializer::Stream(File, m_Details, C); Err)
                    return xerr::create_f<state, "Failed while serializing details.txt">(Err);
            }

            //
            // Export the bone manifest for downstream plugins (xanim_package.plugin) - named after
            // the consumer, same convention as xmaterial.plugin's own MaterialInstance.txt export.
            //
            {
                xtextfile::stream File;
                if (auto Err = File.Open(false, std::format(L"{}\\AnimPackage.txt", m_ResourceLogPath), xtextfile::file_type::TEXT); Err)
                    return xerr::create_f<state, "Failed while opening AnimPackage.txt so it can't be saved">(Err);

                xproperty::settings::context C{};
                if (auto Err = xproperty::sprop::serializer::Stream(File, m_BoneManifest, C); Err)
                    return xerr::create_f<state, "Failed while serializing AnimPackage.txt">(Err);
            }

            //
            // Export
            //
            int Count = 0;
            for (auto& T : m_Target)
            {
                displayProgressBar("Serializing", Count++ / (float)m_Target.size());

                if (T.m_bValid)
                {
                    Serialize(T.m_DataPath);
                }
            }
            displayProgressBar("Serializing", 1);
            return {};
        }

        //--------------------------------------------------------------------------------------

        void Serialize(const std::wstring_view FilePath)
        {
            xserializer::stream Serializer;
            if (auto Err = Serializer.Save
                ( FilePath
                , m_FinalSkeleton
                , m_OptimizationType == optimization_type::O0 ? xserializer::compression_level::FAST : m_OptimizationType == optimization_type::O1 ? xserializer::compression_level::MEDIUM : xserializer::compression_level::HIGH
                ); Err)
            {
                throw(std::runtime_error(std::string(Err.getMessage())));
            }
        }

        xskeleton_desc::details        m_Details;
        xskeleton_desc::descriptor     m_Descriptor;
        xskeleton_desc::bone_manifest  m_BoneManifest;

        xskeleton::skeleton          m_FinalSkeleton;
        xraw3d::anim                 m_Skeleton;
    };

    //------------------------------------------------------------------------------------

    std::unique_ptr<instance> instance::Create(void)
    {
        return std::make_unique<implementation>();
    }
}
