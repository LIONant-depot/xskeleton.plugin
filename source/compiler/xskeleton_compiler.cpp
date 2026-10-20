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

#include <fstream>
#include <filesystem>
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
        // The frozen pose is put in the engine's space by the two corrections of the descriptor, the PostTransform first, then the PreTransform.
        //  - the PreTransform (scale/pivot of the import) moves everything of the skeleton: its rest pose AND the bind data the skins are bound with (the inverse bind
        //    matrices); the skins follow it (their vertices take it too);
        //  - the PostTransform (the model was rotated while it was animated) is for what lives in the space of the animation: the frozen pose (the rest of the root) and
        //    the root of every clip, which take it in the same way (the manifest carries it). It does not touch the bind data nor the skins: where the skin is bound is
        //    where the mesh is, the correction is only between that and the rest/animation space.
        void ApplyPreTransform()
        {
            ApplyTransform(m_Descriptor.m_PostTransform, false);
            ApplyTransform(m_Descriptor.m_PreTransform,  true);
        }

        void ApplyTransform(const xskeleton_desc::transform& PT, bool bAlsoTheBindData)
        {
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
                if (bAlsoTheBindData && !Bone.m_BindMatrixInv.isIdentity())
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
            // The runtime skeleton keeps only the joints; the other source nodes (assimp's pivot helpers, the groups above the
            // skeleton) are folded into them - their transforms baked in - and stay in the Details for the editor
            // (documentation/Editors/skeleton_runtime_folding_plan.md)
            //
            {
                auto       Joints  = m_Skeleton.RuntimeJoints();
                xskeleton_desc::merge::KeepIntentBones(m_Descriptor, m_Skeleton, Joints.m_Keep);
                const auto nBefore = m_Skeleton.m_Bone.size();
                for (auto& Name : m_Skeleton.FoldBones(Joints.m_Keep, Joints.m_bSyntheticRoot))
                    LogMessage(xresource_pipeline::msg_type::WARNING, std::format("Node '{}' kept: folding it would not fit translation/rotation/scale", Name));
                LogMessage(xresource_pipeline::msg_type::INFO, std::format("Runtime joints: {} of {} source nodes{}", m_Skeleton.m_Bone.size() - (Joints.m_bSyntheticRoot ? 1 : 0), nBefore, Joints.m_bSyntheticRoot ? std::format(", under a synthetic root '{}'", xraw3d::anim::synthetic_root_name_v) : ""));
            }

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

                // The bones with an intent, per intent (the user's marks and the suggestions from the names)
                for (int k = 1; k < static_cast<int>(xskeleton::intent::COUNT); ++k)
                {
                    std::string Names;
                    for (int i = 0; i < m_FinalSkeleton.m_nBones; ++i)
                        if (m_FinalSkeleton.m_pBones[i].m_Flags.m_Intent == k) Names += (Names.empty() ? "" : ", ") + m_BoneManifest.m_Bones[i].m_Name;
                    if (!Names.empty()) LogMessage(xresource_pipeline::msg_type::INFO, std::format("Intent {}: {}", xskeleton::intent_names_v[k], Names));
                }

                // Where the joints are in the bind pose (its PreTransform applied): the space the skins bound to this skeleton are in. Only the bones with
                // bind data of their own (an identity inverse bind pose means "none": see ApplyPreTransform)
                {
                    std::vector<xmath::fvec3> Joints;
                    for (int i = 0; i < m_FinalSkeleton.m_nBones; ++i)
                        if (!m_FinalSkeleton.m_pBones[i].m_InvBindPose.isIdentity())
                            Joints.push_back(m_FinalSkeleton.m_pBones[i].m_InvBindPose.Inverse() * xmath::fvec3::fromZero());
                    // How the frozen pose (the rest of the root bone) turns into the bind pose the skin is bound with: where the three axes of the rest go. The identity when the two
                    // agree; when they do not, the Frozen view of a skin shows the mesh turned, and this is the rotation a PostTransform would have to give the frozen pose.
                    if (m_FinalSkeleton.m_nBones > 0 && !m_FinalSkeleton.m_pBones[0].m_InvBindPose.isIdentity())
                    {
                        const xmath::fmat4 Rest  = m_FinalSkeleton.m_pBoneRests[0].m_RestPose.toMatrix();
                        const xmath::fmat4 Bind  = m_FinalSkeleton.m_pBones[0].m_InvBindPose.Inverse();
                        const xmath::fmat4 Turn  = Bind * Rest.Inverse();
                        const auto Dir = [&](const xmath::fvec3& V) { const xmath::fvec3 R = Turn * V - Turn * xmath::fvec3::fromZero(); return R.Length() > 1.0e-12f ? R.NormalizeCopy() : R; };
                        const auto X = Dir({ 1, 0, 0 }), Y = Dir({ 0, 1, 0 }), Z = Dir({ 0, 0, 1 });
                        LogMessage(xresource_pipeline::msg_type::INFO, std::format("Root rest to bind: X goes to {:.3f} {:.3f} {:.3f}, Y to {:.3f} {:.3f} {:.3f}, Z to {:.3f} {:.3f} {:.3f}", X.m_X, X.m_Y, X.m_Z, Y.m_X, Y.m_Y, Y.m_Z, Z.m_X, Z.m_Y, Z.m_Z));
                    }
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

    //------------------------------------------------------------------------------------------
    // -SELFTEST: xraw3d's fold on synthetic rigs (documentation/Editors/skeleton_runtime_folding_plan.md)

    int SelfTest()
    {
        int nFailed = 0;
        const auto Check = [&](bool b, const char* pWhat) { if (!b) { ++nFailed; std::printf("SELFTEST FAILED: %s\n", pWhat); } };

        using anim = xraw3d::anim;
        const auto MakeBone = [](const char* pName, int Parent, bool bBound, xmath::fvec3 S, xmath::fquat R, xmath::fvec3 T)
        {
            anim::bone B{};
            B.m_Name = pName; B.m_iParent = Parent; B.m_nChildren = 0; B.m_bHasBind = bBound;
            B.m_BindScale = S; B.m_BindRotation = R; B.m_BindTranslation = T;
            B.m_bScaleKeys = B.m_bRotationKeys = B.m_bTranslationKeys = B.m_bIsMasked = false;
            B.m_BindMatrix = xmath::fmat4::fromIdentity(); B.m_BindMatrixInv = xmath::fmat4::fromIdentity(); B.m_NeutralPose = xmath::fmat4::fromIdentity();
            return B;
        };
        const auto Key = [](xmath::fvec3 S, xmath::fquat R, xmath::fvec3 T) { anim::key_frame K; K.m_Scale = S; K.m_Rotation = R; K.m_Position = T; return K; };
        const auto Global = [](const anim& A, const std::string& Name, int Frame)
        {
            xmath::fmat4 M; A.ComputeRawBoneL2W(A.GetBoneIDFromName(Name), M, Frame); return M.ExtractPosition();
        };
        const auto Near = [](xmath::fvec3 A, xmath::fvec3 B) { return (A - B).Length() < 1.0e-3f; };
        const xmath::fvec3 One(1, 1, 1);
        const xmath::fquat Id = xmath::fquat::fromIdentity();
        const xmath::fquat Q90 = xmath::fquat(xmath::radian3(0_xdeg, 90_xdeg, 0_xdeg));

        // Scene -> Hip (bound) -> helper ($AssimpFbx$, animated) -> Spine (bound) -> Head (unbound end joint)
        {
            anim A; A.m_nFrames = 2; A.m_FPS = 30;
            A.m_Bone = { MakeBone("Scene", -1, false, One, Id, {0, 0, 0})
                       , MakeBone("Hip", 0, true, One, Id, {0, 1, 0})
                       , MakeBone("Hip_$AssimpFbx$_Rotation", 1, false, One, Q90, {0, 0.1f, 0})
                       , MakeBone("Spine", 2, true, One, Id, {0.2f, 0.3f, 0})
                       , MakeBone("Head", 3, false, One, Id, {0, 0.4f, 0}) };
            for (int f = 0; f < 2; ++f)
                for (auto& B : A.m_Bone)
                    A.m_KeyFrame.push_back(Key(B.m_BindScale, (f == 1 && B.m_Name.find("$AssimpFbx$") != std::string::npos) ? xmath::fquat(xmath::radian3(0_xdeg, 45_xdeg, 0_xdeg)) : B.m_BindRotation, B.m_BindTranslation));

            const auto J = A.RuntimeJoints();
            Check(!J.m_Keep[0] && J.m_Keep[1] && !J.m_Keep[2] && J.m_Keep[3] && J.m_Keep[4] && !J.m_bSyntheticRoot, "keep set: the bound chain and its end joint, not the scene node or the helper; Hip is the root");

            const xmath::fvec3 Spine0 = Global(A, "Spine", 0), Spine1 = Global(A, "Spine", 1), Head1 = Global(A, "Head", 1);
            const auto Kept = A.FoldBones(J.m_Keep, J.m_bSyntheticRoot);
            Check(Kept.empty() && A.m_Bone.size() == 3, "the scene node and the helper are folded");
            Check(A.m_Bone[0].m_Name == "Hip" && A.m_Bone[0].m_iParent == -1 && A.m_Bone[1].m_Name == "Spine" && A.m_Bone[1].m_iParent == 0, "Spine hangs from Hip");
            Check(Near(Global(A, "Spine", 0), Spine0) && Near(Global(A, "Spine", 1), Spine1) && Near(Global(A, "Head", 1), Head1), "the helper's animation is baked: every kept joint is where it was, every frame");
        }

        // A group (unbound) holding two bound branches under the scene root -> a synthetic root
        {
            anim A; A.m_nFrames = 1; A.m_FPS = 30;
            A.m_Bone = { MakeBone("RootNode", -1, false, One, Id, {0, 0, 0})
                       , MakeBone("Group", 0, false, One, Q90, {0, 0, 1})
                       , MakeBone("Body", 1, true, One, Id, {0, 1, 0})
                       , MakeBone("Face", 0, true, One, Id, {0, 1.6f, 0}) };
            for (auto& B : A.m_Bone) A.m_KeyFrame.push_back(Key(B.m_BindScale, B.m_BindRotation, B.m_BindTranslation));
            const auto J    = A.RuntimeJoints();
            const auto Body = Global(A, "Body", 0);
            Check(J.m_bSyntheticRoot && !J.m_Keep[0] && !J.m_Keep[1], "two branches under the scene root: a synthetic root, the group folded");
            A.FoldBones(J.m_Keep, J.m_bSyntheticRoot);
            Check(A.m_Bone.size() == 3 && A.m_Bone[0].m_Name == anim::synthetic_root_name_v && A.m_Bone[1].m_iParent == 0 && A.m_Bone[2].m_iParent == 0, "both branches hang from the synthetic root");
            Check(Near(Global(A, "Body", 0), Body), "the group's transform is baked into the branch");
        }

        // A non-uniform scale above a rotated child makes shear: the fold would not fit, the node stays
        {
            anim A; A.m_nFrames = 1; A.m_FPS = 30;
            A.m_Bone = { MakeBone("Hip", -1, true, One, Id, {0, 1, 0})
                       , MakeBone("Squash_$AssimpFbx$_Scaling", 0, false, xmath::fvec3(1, 3, 1), Id, {0, 0, 0})
                       , MakeBone("Arm", 1, true, One, xmath::fquat(xmath::radian3(0_xdeg, 0_xdeg, 45_xdeg)), {0.2f, 0, 0}) };
            for (auto& B : A.m_Bone) A.m_KeyFrame.push_back(Key(B.m_BindScale, B.m_BindRotation, B.m_BindTranslation));
            const auto J    = A.RuntimeJoints();
            const auto Kept = A.FoldBones(J.m_Keep, J.m_bSyntheticRoot);
            Check(Kept.size() == 1 && Kept[0] == "Squash_$AssimpFbx$_Scaling" && A.m_Bone.size() == 3, "a fold that needs shear keeps the node and names it");
        }

        // Intents suggested from the names rigs commonly use
        {
            using xskeleton::intent;
            Check(xskeleton::SuggestIntent("MTP_Global_a")     == intent::FACING,    "MTP_Global_* faces");
            Check(xskeleton::SuggestIntent("JNT_C_B_000_Root") == intent::MOTION,    "a *_Root bone is the motion root");
            Check(xskeleton::SuggestIntent("MTP_Head_b")       == intent::LOOK_AT,   "MTP_Head_* is a look-at");
            Check(xskeleton::SuggestIntent("MTP_L_Foot_a")     == intent::IK_TARGET, "MTP_*_Foot_* is an IK target");
            Check(xskeleton::SuggestIntent("IK_Hand_L")        == intent::IK_TARGET, "IK_* is an IK target");
            Check(xskeleton::SuggestIntent("CC_Base_Spine01")  == intent::NONE,      "an ordinary joint has none");
            Check(xskeleton::SuggestIntent("MTP_Hips_a")       == intent::NONE,      "MTP_Hips_* is not guessed");
        }

        // A descriptor saved before 1.1 (its compact bone table has the per-bone Transform columns, its properties the RootBone Transform) still loads;
        // saved again it has no Transform at all
        {
            const auto Dir = std::filesystem::temp_directory_path() / L"xskeleton_selftest";
            std::error_code Ec;
            std::filesystem::create_directories(Dir, Ec);
            const auto Old = Dir / L"old.txt", New = Dir / L"new.txt";
            {
                std::ofstream(Old, std::ios::binary) << R"(
[ DescriptorVersion ]
{ Major:d  Minor:d }
//-------  -------
     1        0

// New Types
< s32:d u32:g s16:C u16:H s8:c u8:h f32:f f64:F string:s wstring:S u64:G s64:D bool:c full_guid:GG enum:s >

[ xProperties : 3 ]
{ Name:s                                         Value:?                                    }
//---------------------------------------------  ------------------------------------------
  "Skeleton/RootBone/Transform/Scale/X"          ;f32     #3F800000
  "Skeleton/RootBone/Transform/EditorLockScale"  ;bool    0
  "Skeleton/CompactTree"                         ;bool    1

[ BoneOverrides : 2 ]
{ Parent:d  Name:s    Rename:s  Type:d  Expose:d  Delete:d  LOD:d  Scale:fff                      Rotation:fff  Position:fff  Lock:d }
//--------  --------  --------  ------  --------  --------  -----  -----------------------------  ------------  ------------  ------
     -1     "Root"      ""       0        0         0        0    #3F800000 #3F800000 #3F800000    #0 #0 #0      #0 #0 #0      1
      0     "Child"     "Kid"    1        1         0        2    #3F800000 #3F800000 #3F800000    #0 #0 #0      #0 #0 #0      0
)";
            }
            xskeleton_desc::descriptor D;
            xproperty::settings::context C;
            Check(!D.Serialize(true, Old.wstring(), C), "a descriptor saved before 1.1 loads");
            Check(D.m_RootBone.m_Name == "Root" && D.m_RootBone.m_Bones.size() == 1, "the old bone table keeps its tree");
            if (!D.m_RootBone.m_Bones.empty())
            {
                const auto& Child = D.m_RootBone.m_Bones[0];
                Check(Child.m_Name == "Child" && Child.m_Rename == "Kid" && Child.m_Type == xskeleton_desc::bone_type::VIRTUAL && Child.m_bExpose && Child.m_LODLevel == 2, "the old bone table keeps the fields that stay");
            }

            Check(!D.Serialize(false, New.wstring(), C), "a descriptor saves");
            std::ifstream In(New, std::ios::binary);
            const std::string Text((std::istreambuf_iterator<char>(In)), std::istreambuf_iterator<char>());
            Check(Text.find("Rotation:fff") == std::string::npos && Text.find("RootBone/Transform") == std::string::npos, "a saved descriptor has no bone Transform");
            xskeleton_desc::descriptor E;
            Check(!E.Serialize(true, New.wstring(), C) && E.m_RootBone.m_Bones.size() == 1 && E.m_RootBone.m_Bones[0].m_Rename == "Kid", "a saved descriptor loads again");
            std::filesystem::remove_all(Dir, Ec);
        }

        std::printf(nFailed ? "SELFTEST: %d failed\n" : "SELFTEST: OK\n", nFailed);
        return nFailed;
    }
}
