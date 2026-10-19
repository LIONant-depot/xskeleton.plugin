#include "xskeleton_compiler.h"

#include "dependencies/xraw3d/source/xraw3d.h"
#include "dependencies/xraw3d/source/details/xraw3d_assimp_import_v3.h"

#include "dependencies/xstrtool/source/xstrtool.h"

#include "../xskeleton_descriptor.h"
#include "../xskeleton.h"
#include "../xskeleton_details.h"
#include "../xskeleton_bone_manifest.h"
#include "xskeleton_merge.h"

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

        // The merge of the import with the descriptor's overrides: xskeleton_merge.h (the skin compiler makes the same bone list)
        using merged_bone = xskeleton_desc::merge::merged_bone;

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
            if (auto Err = xskeleton_desc::merge::BuildMergedBones<state>(m_Descriptor, m_Skeleton, Bones, [&](std::string Text) { LogMessage(xresource_pipeline::msg_type::WARNING, std::move(Text)); }); Err) return Err;

            std::vector<std::uint32_t> Hashes;
            if (auto Err = xskeleton_desc::merge::AssignNameHashesAndCheckCollisions<state>(Bones, Hashes); Err) return Err;

            // Export the final, post-merge bone order/hashes for downstream plugins (e.g.
            // xanim_package.plugin) that need to bind data to this exact layout without reading our
            // compiled binary - see xskeleton_bone_manifest.h and onCompile()'s AnimPackage.txt write.
            xskeleton_desc::merge::FillManifest(m_Descriptor, Bones, Hashes, m_BoneManifest);

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

            // The source file is a dependency of this resource, registered BEFORE loading it (a broken link still shows as a dependent of the file): it is what the Assets tab counts
            // (the #N of the file) and lists (Find Resource / Open Resource), and what makes a change of the file compile this resource again.
            m_Dependencies.m_Assets.push_back(m_Descriptor.m_ImportAsset);

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

                // Where the joints are in the bind pose (its PreTransform applied): the space the skins bound to this skeleton are in. Only the bones with
                // bind data of their own (an identity inverse bind pose means "none": see ApplyPreTransform)
                {
                    std::vector<xmath::fvec3> Joints;
                    for (int i = 0; i < m_FinalSkeleton.m_nBones; ++i)
                        if (!m_FinalSkeleton.m_pBones[i].m_InvBindPose.isIdentity())
                            Joints.push_back(m_FinalSkeleton.m_pBones[i].m_InvBindPose.Inverse() * xmath::fvec3::fromZero());
                    if (!Joints.empty())
                    {
                        const xmath::fbbox Box{ std::span<const xmath::fvec3>(Joints) };
                        const auto         Size = Box.getSize(), Center = Box.getCenter();
                        LogMessage(xresource_pipeline::msg_type::INFO, std::format("Joints {} Size {:.4f} {:.4f} {:.4f} Center {:.4f} {:.4f} {:.4f}", Joints.size(), Size.m_X, Size.m_Y, Size.m_Z, Center.m_X, Center.m_Y, Center.m_Z));
                    }
                }
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
                // A tree too deep for xproperty's paths is saved apart, in its own table after the properties
                m_Details.m_bCompactTree = xskeleton_desc::details::Depth(m_Details.m_RootBone) > xskeleton_desc::details::compact_depth_v;
                xskeleton_desc::details::bone Tree;
                if (m_Details.m_bCompactTree) std::swap(Tree, m_Details.m_RootBone);
                auto Err = xproperty::sprop::serializer::Stream(File, m_Details, C);
                if (m_Details.m_bCompactTree) std::swap(Tree, m_Details.m_RootBone);
                if (Err) return xerr::create_f<state, "Failed while serializing details.txt">(Err);
                if (m_Details.m_bCompactTree)
                    if (auto Err2 = m_Details.StreamCompactTree(File, m_Details.m_RootBone); Err2)
                        return xerr::create_f<state, "Failed while serializing the bone tree of details.txt">(Err2);
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
