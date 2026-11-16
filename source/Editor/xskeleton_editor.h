#ifndef XSKELETON_EDITOR_H
#define XSKELETON_EDITOR_H
#pragma once

// The Skeleton editor: opens a skeleton resource from the asset browser in its own window. The bones are drawn as octahedral wedges (an outline
// and a translucent fill that casts a shadow on the ground); a click in the view, or in the bone hierarchy, selects bones. What the compiler does
// with each bone (a new name, virtual, deleted, exposed as a socket, its LOD, its weight in each mask layer) is edited in the hierarchy or with
// commands, all undoable. Hosts include this header and open editors through xeditor::open_resource_editors.
#include "source/Tools/Editor/xeditor_descriptor_editor.h"
#include "source/Tools/Editor/xeditor_camera_toolbar.h"
#include "source/Tools/Editor/xeditor_transform_gizmo.h"
#include "source/Tools/Editor/xeditor_scale_reference.h"
#include "dependencies/xresource_pipeline_v2/source/editor/xresource_editor_resources.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_scene.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_runtime.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_thumbnail.h"
#include "plugins/xskeleton.plugin/source/xskeleton_descriptor.h"
#include "plugins/xskeleton.plugin/source/xskeleton_joints.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_posing_handles.h"
#include "plugins/xskeleton.plugin/source/xskeleton_joint_evidence.h"
#include <filesystem>
#include <array>
#include <set>
#include <tuple>

#include <charconv>
#include <functional>
#include <unordered_map>

namespace xskeleton_editor
{
    //--------------------------------------------------------------------------------------------
    // Bone names. The compiled skeleton has only a CRC per bone: names come from the compiler's log of what it imported and from the
    // descriptor, hashed the way the compiler does.
    //--------------------------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------------------------
    // View settings: not part of the resource, not saved
    //--------------------------------------------------------------------------------------------
    static constexpr auto pose_mode_v = std::array
    { xproperty::settings::enum_item("Frozen", pose_mode::FROZEN)
    , xproperty::settings::enum_item("Bind",   pose_mode::BIND)
    };

    struct view_settings
    {
        pose_mode           m_Pose              = pose_mode::BIND;      // as Unreal (its reference pose is the bind) and Blender show a rig: inside its mesh
        bool                m_bAlwaysShowNames  = false;
        bool                m_bNormalizeSize    = true;         // the view is scaled so the skeleton is about a meter: the grid, depth range and wedge sizes assume a human-sized subject
        render_color_mode   m_ColorMode         = render_color_mode::NORMAL_RENDER;
        std::string         m_ActiveLayer;                      // the mask layer whose weights the hierarchy shows and Active Layer colouring visualises

        XPROPERTY_DEF
        ( "SkeletonView", view_settings
        , obj_member<"Pose",                &view_settings::m_Pose, member_enum_span<pose_mode_v> >
        , obj_member<"AlwaysShowNames",     &view_settings::m_bAlwaysShowNames >
        , obj_member<"ResizeSkeletonTo1m",  &view_settings::m_bNormalizeSize >
        , obj_member<"RenderColor",         &view_settings::m_ColorMode, member_enum_span<render_color_mode_v> >
        , obj_member<"ActiveLayer",         &view_settings::m_ActiveLayer, member_help<"The mask layer (by name) whose weights the hierarchy shows; empty shows none.">>
        )
    };
    XPROPERTY_REG(view_settings)

    //--------------------------------------------------------------------------------------------
    // Edits of what the compiler does with bones. A bone is named by its raw import name; several are given one per line. Every one snapshots
    // the descriptor first, so undo puts back everything it touched, including the entries it had to create.
    //--------------------------------------------------------------------------------------------
    inline xskeleton_desc::bone* FindBoneOverride(xskeleton_desc::bone& Node, std::string_view Name) noexcept
    {
        if (Node.m_Name == Name) return &Node;
        for (auto& Child : Node.m_Bones)
            if (auto* pFound = FindBoneOverride(Child, Name)) return pFound;
        return nullptr;
    }

    inline const xskeleton_desc::bone* FindBoneOverride(const xskeleton_desc::bone& Node, std::string_view Name) noexcept
    {
        if (Node.m_Name == Name) return &Node;
        for (auto& Child : Node.m_Bones)
            if (auto* pFound = FindBoneOverride(Child, Name)) return pFound;
        return nullptr;
    }

    // Where the compiler looks for an override is by name, wherever it sits in the tree: a new one goes directly under the root
    inline xskeleton_desc::bone& FindOrCreateBoneOverride(xskeleton_desc::descriptor& Descriptor, std::string_view Name) noexcept
    {
        if (auto* pFound = FindBoneOverride(Descriptor.m_RootBone, Name)) return *pFound;
        xskeleton_desc::bone NewOverride;
        NewOverride.m_Name = std::string(Name);
        Descriptor.m_RootBone.m_Bones.push_back(std::move(NewOverride));
        return Descriptor.m_RootBone.m_Bones.back();
    }

    inline const xskeleton_desc::details::bone* FindDetailsBone(const xskeleton_desc::details::bone& Node, std::string_view Name) noexcept
    {
        if (Node.m_Name == Name) return &Node;
        for (auto& Child : Node.m_Children)
            if (auto* pFound = FindDetailsBone(Child, Name)) return pFound;
        return nullptr;
    }

    // The compiler enforces "a bone's LOD is never lower than its parent's" and only warns when the descriptor disagrees: mirrored here, the editor
    // never lets one author something the compiler overrides anyway. The just changed LOD is pushed down the subtree, raising any that is lower.
    inline void ClampDescendantLODs(xskeleton_desc::descriptor& D, const xskeleton_desc::details::bone& Node, int MinLOD) noexcept
    {
        for (auto& Child : Node.m_Children)
        {
            auto& Override = FindOrCreateBoneOverride(D, Child.m_Name);
            if (Override.m_LODLevel < MinLOD) Override.m_LODLevel = MinLOD;
            ClampDescendantLODs(D, Child, Override.m_LODLevel);
        }
    }

    // A bulk edit, unlike the clamp above: every descendant is set to exactly this LOD
    inline void MatchDescendantLODs(xskeleton_desc::descriptor& D, const xskeleton_desc::details::bone& Node, int LOD) noexcept
    {
        for (auto& Child : Node.m_Children)
        {
            FindOrCreateBoneOverride(D, Child.m_Name).m_LODLevel = LOD;
            MatchDescendantLODs(D, Child, LOD);
        }
    }

    // Mask layers are sparse: a bone with no entry has weight 0, and writing 0 removes the entry again
    inline float GetMaskWeight(xskeleton_desc::descriptor& D, int iGroup, std::string_view Bone) noexcept
    {
        if (iGroup < 0 || iGroup >= int(D.m_MaskGroups.size())) return 0.0f;
        for (auto& E : D.m_MaskGroups[iGroup].m_Entries)
            if (E.m_BoneName == Bone) return E.m_Weight;
        return 0.0f;
    }

    inline void SetMaskWeight(xskeleton_desc::descriptor& D, int iGroup, std::string_view Bone, float Weight) noexcept
    {
        if (iGroup < 0 || iGroup >= int(D.m_MaskGroups.size())) return;
        auto& Group = D.m_MaskGroups[iGroup];
        Weight = std::clamp(Weight, 0.0f, 1.0f);
        auto It = std::ranges::find_if(Group.m_Entries, [&](const xskeleton_desc::mask_entry& E) { return E.m_BoneName == Bone; });
        if (It != Group.m_Entries.end()) { if (Weight <= 0.0f) Group.m_Entries.erase(It); else It->m_Weight = Weight; }
        else if (Weight > 0.0f)          Group.m_Entries.push_back({ std::string(Bone), Weight });
    }

    inline void MatchDescendantMaskWeights(xskeleton_desc::descriptor& D, const xskeleton_desc::details::bone& Node, int iGroup, float Weight) noexcept
    {
        for (auto& Child : Node.m_Children)
        {
            SetMaskWeight(D, iGroup, Child.m_Name, Weight);
            MatchDescendantMaskWeights(D, Child, iGroup, Weight);
        }
    }

    inline std::vector<std::string> SplitLines(const std::string& Text) noexcept
    {
        std::vector<std::string> Out;
        std::size_t Start = 0;
        while (Start <= Text.size())
        {
            const auto End = Text.find('\n', Start);
            const auto Line = Text.substr(Start, End == std::string::npos ? std::string::npos : End - Start);
            if (!Line.empty()) Out.push_back(Line);
            if (End == std::string::npos) break;
            Start = End + 1;
        }
        return Out;
    }

    struct edit_cmd : xundo::command_base
    {
        xeditor::descriptor_document&           m_Doc;
        const xskeleton_desc::details&          m_Details;
        const char*                             m_pHelp;

        edit_cmd(xundo::system& System, xeditor::descriptor_document& Doc, const xskeleton_desc::details& Details, const char* pName, const char* pHelp) noexcept
            : command_base(System, pName, nullptr), m_Doc(Doc), m_Details(Details), m_pHelp(pHelp) {}

        const char* getCommandHelp() const noexcept override { return m_pHelp; }
        xskeleton_desc::descriptor& Desc() noexcept { return static_cast<xskeleton_desc::descriptor&>(*m_Doc.m_pDescriptor); }

        // The bones the command names: at least one, each one an imported bone (the compiler matches overrides by that name)
        std::string Bones(std::vector<std::string>& Out) noexcept
        {
            std::string Text;
            if (!xeditor::cmd_util::GetArg(m_Parser, m_hBones, Text)) return "bad arguments";
            Out = SplitLines(Text);
            if (Out.empty()) return "no bones named";
            for (auto& Name : Out)
                if (!FindDetailsBone(m_Details.m_RootBone, Name)) return std::format("no bone '{}' (compile the skeleton first)", Name);
            return {};
        }

        void BackupCurrenState(xundo::undo_file& File) noexcept override { xeditor::WriteString(File, m_Doc.Snapshot()); }
        void Undo(xundo::undo_file& File) noexcept override               { m_Doc.Restore(xeditor::ReadString(File)); }

        xcmdline::parser::handle m_hBones;
    };

    // SetBone: any of the options for every named bone
    struct set_bone_cmd : edit_cmd
    {
        set_bone_cmd(xundo::system& System, xeditor::descriptor_document& Doc, const xskeleton_desc::details& Details) noexcept
            : edit_cmd(System, Doc, Details, "SetBone", "Edits what the compiler does with bones (undoable). Usage: SetBone -Bones \"one raw name per line\" [-Rename \"name\"] [-Virtual true|false] [-Delete true|false] [-Expose true|false] [-LOD level] [-Intent auto|none|motion|facing|lookat|iktarget]")
        { RegisterArguments(); }

        void RegisterArguments() noexcept override
        {
            m_hBones   = m_Parser.addOption("Bones",   "Raw import names, one per line",     true,  1);
            m_hRename  = m_Parser.addOption("Rename",  "The compiled name (empty: none)",     false, 1);
            m_hVirtual = m_Parser.addOption("Virtual", "true: a virtual bone, false: a normal one",   false, 1);
            m_hDelete  = m_Parser.addOption("Delete",  "true: leave the bone out of the skeleton",    false, 1);
            m_hExpose  = m_Parser.addOption("Expose",  "true: the bone is a socket",                  false, 1);
            m_hLOD     = m_Parser.addOption("LOD",     "The highest LOD the bone is active at",       false, 1);
            m_hIntent  = m_Parser.addOption("Intent",  "What the bone is for: auto (from its name), none, motion, facing, lookat, iktarget", false, 1);
        }

        static bool ParseIntent(const std::string& Text, xskeleton_desc::bone_intent& Out) noexcept
        {
            using bi = xskeleton_desc::bone_intent;
            static constexpr std::pair<std::string_view, bi> Map[]
            { {"auto", bi::AUTO}, {"none", bi::NONE}, {"motion", bi::MOTION}, {"facing", bi::FACING}, {"lookat", bi::LOOK_AT}, {"iktarget", bi::IK_TARGET} };
            for (auto& [Name, Value] : Map) if (Text == Name) { Out = Value; return true; }
            return false;
        }

        static bool Bool(const std::string& Text, bool& Out) noexcept
        {
            if (Text != "true" && Text != "false") return false;
            Out = Text == "true";
            return true;
        }

        std::string Redo() noexcept override
        {
            if (!m_Doc.isLoaded()) return "SetBone: nothing loaded";
            std::vector<std::string> Names;
            if (auto Err = Bones(Names); !Err.empty()) return "SetBone: " + Err;

            std::string Rename, Virtual, Delete, Expose, LOD, Intent;
            const bool bRename = xeditor::cmd_util::GetArg(m_Parser, m_hRename, Rename), bVirtual = xeditor::cmd_util::GetArg(m_Parser, m_hVirtual, Virtual);
            const bool bDelete = xeditor::cmd_util::GetArg(m_Parser, m_hDelete, Delete), bExpose = xeditor::cmd_util::GetArg(m_Parser, m_hExpose, Expose), bLOD = xeditor::cmd_util::GetArg(m_Parser, m_hLOD, LOD);
            const bool bIntent = xeditor::cmd_util::GetArg(m_Parser, m_hIntent, Intent);
            if (!bRename && !bVirtual && !bDelete && !bExpose && !bLOD && !bIntent) return "SetBone: nothing to change";
            xskeleton_desc::bone_intent NewIntent{};
            if (bIntent && !ParseIntent(Intent, NewIntent)) return "SetBone: Intent takes auto, none, motion, facing, lookat or iktarget";

            bool bV = false, bD = false, bE = false;
            int iLOD = 0;
            if ((bVirtual && !Bool(Virtual, bV)) || (bDelete && !Bool(Delete, bD)) || (bExpose && !Bool(Expose, bE))) return "SetBone: Virtual, Delete and Expose take true or false";
            if (bLOD && (std::from_chars(LOD.data(), LOD.data() + LOD.size(), iLOD).ec != std::errc() || iLOD < 0)) return "SetBone: LOD takes a level from 0";

            auto& D = Desc();
            for (auto& Name : Names)
            {
                auto& B = FindOrCreateBoneOverride(D, Name);
                if (bRename)  B.m_Rename      = Rename;
                if (bVirtual) B.m_Type        = bV ? xskeleton_desc::bone_type::VIRTUAL : xskeleton_desc::bone_type::NORMAL;
                if (bDelete)  B.m_bDeleteBone = bD;
                if (bExpose)  B.m_bExpose     = bE;
                if (bIntent)  B.m_Intent      = NewIntent;
                if (bLOD)
                {
                    B.m_LODLevel = iLOD;
                    if (auto* pNode = FindDetailsBone(m_Details.m_RootBone, Name)) ClampDescendantLODs(D, *pNode, iLOD);
                }
            }
            m_Doc.m_bDirty = true;
            return {};
        }

        xcmdline::parser::handle m_hRename, m_hVirtual, m_hDelete, m_hExpose, m_hLOD, m_hIntent;
    };

    // MatchLOD: every descendant of the named bones gets exactly the bone's own LOD
    struct match_lod_cmd : edit_cmd
    {
        match_lod_cmd(xundo::system& System, xeditor::descriptor_document& Doc, const xskeleton_desc::details& Details) noexcept
            : edit_cmd(System, Doc, Details, "MatchLOD", "Gives every descendant of the bones the bone's own LOD (undoable). Usage: MatchLOD -Bones \"one raw name per line\"")
        { RegisterArguments(); }
        void RegisterArguments() noexcept override { m_hBones = m_Parser.addOption("Bones", "Raw import names, one per line", true, 1); }

        std::string Redo() noexcept override
        {
            if (!m_Doc.isLoaded()) return "MatchLOD: nothing loaded";
            std::vector<std::string> Names;
            if (auto Err = Bones(Names); !Err.empty()) return "MatchLOD: " + Err;
            auto& D = Desc();
            for (auto& Name : Names)
            {
                const auto* pOverride = FindBoneOverride(D.m_RootBone, Name);
                if (auto* pNode = FindDetailsBone(m_Details.m_RootBone, Name)) MatchDescendantLODs(D, *pNode, pOverride ? pOverride->m_LODLevel : 0);
            }
            m_Doc.m_bDirty = true;
            return {};
        }
    };

    // SetMaskWeight: the weight of bones in a mask layer, optionally down their whole subtrees
    struct set_mask_weight_cmd : edit_cmd
    {
        set_mask_weight_cmd(xundo::system& System, xeditor::descriptor_document& Doc, const xskeleton_desc::details& Details) noexcept
            : edit_cmd(System, Doc, Details, "SetMaskWeight", "Sets the weight (0 to 1) of bones in a mask layer (undoable). Usage: SetMaskWeight -Layer \"name\" -Bones \"one raw name per line\" -Weight w [-Children true]")
        { RegisterArguments(); }
        void RegisterArguments() noexcept override
        {
            m_hLayer    = m_Parser.addOption("Layer",    "The mask layer's name",             true,  1);
            m_hBones    = m_Parser.addOption("Bones",    "Raw import names, one per line",     true,  1);
            m_hWeight   = m_Parser.addOption("Weight",   "0 to 1",                                     true,  1);
            m_hChildren = m_Parser.addOption("Children", "true: their descendants get it too",         false, 1);
        }

        std::string Redo() noexcept override
        {
            if (!m_Doc.isLoaded()) return "SetMaskWeight: nothing loaded";
            std::vector<std::string> Names;
            if (auto Err = Bones(Names); !Err.empty()) return "SetMaskWeight: " + Err;

            std::string Layer, Weight, Children;
            float W = 0;
            if (!xeditor::cmd_util::GetArg(m_Parser, m_hLayer, Layer) || !xeditor::cmd_util::GetArg(m_Parser, m_hWeight, Weight)
             || std::from_chars(Weight.data(), Weight.data() + Weight.size(), W).ec != std::errc() || W < 0.0f || W > 1.0f) return "SetMaskWeight: Weight takes a number from 0 to 1";
            auto& D = Desc();
            const int iGroup = D.findMaskGroup(Layer);
            if (iGroup < 0) return std::format("SetMaskWeight: no mask layer '{}'", Layer);
            const bool bChildren = xeditor::cmd_util::GetArg(m_Parser, m_hChildren, Children) && Children == "true";

            for (auto& Name : Names)
            {
                SetMaskWeight(D, iGroup, Name, W);
                if (bChildren) if (auto* pNode = FindDetailsBone(m_Details.m_RootBone, Name)) MatchDescendantMaskWeights(D, *pNode, iGroup, W);
            }
            m_Doc.m_bDirty = true;
            return {};
        }

        xcmdline::parser::handle m_hLayer, m_hWeight, m_hChildren;
    };

    // The scene runtime (the bones' translucent fill and the shadow they cast) now lives in
    // xskeleton_editor_runtime.h - split out so xskeleton_thumbnail.h can use it without a circular
    // include back to this file (see that header's own top comment).

    //--------------------------------------------------------------------------------------------
    // The editor
    //--------------------------------------------------------------------------------------------
    struct session : xeditor::descriptor_editor
    {
        using desc = xskeleton_desc::descriptor;

        struct query_cmd : xundo::query_command_base
        {
            enum class kind { list_bones, select_bones, clear_selection, list_layers };
            session& m_Session; kind m_Kind; const char* m_pHelp;
            query_cmd(xundo::system& System, session& Session, kind Kind, const char* pName, const char* pHelp) noexcept
                : query_command_base(System, pName, nullptr), m_Session(Session), m_Kind(Kind), m_pHelp(pHelp) { RegisterArguments(); }
            const char* getCommandHelp() const noexcept override { return m_pHelp; }
            void RegisterArguments() noexcept override
            {
                if (m_Kind == kind::select_bones)
                {
                    m_hBones = m_Parser.addOption("Bones", "Raw import names, one per line", true, 1);
                    m_hAdd   = m_Parser.addOption("Add",   "true: keep the current selection and add to it", false, 1);
                }
            }
            std::string Query() noexcept override { return m_Session.RunQuery(*this); }
            xcmdline::parser::handle m_hBones, m_hAdd;
        };

        // Joint limits (design v2, J1): the commands every joint edit goes through - the panel, the gizmos and the console alike. An edit
        // (with its mirror and its review invalidation) is one undo step; a no-op leaves none; editing never marks a joint reviewed.
        struct joint_cmd : xundo::query_command_base
        {
            enum class kind { info, set, reset, mirror, review, mode, view, analyze, apply };
            session& m_Session; kind m_Kind; const char* m_pHelp;
            joint_cmd(xundo::system& System, session& Session, kind Kind, const char* pName, const char* pHelp) noexcept
                : query_command_base(System, pName, nullptr), m_Session(Session), m_Kind(Kind), m_pHelp(pHelp) { RegisterArguments(); }
            const char* getCommandHelp() const noexcept override { return m_pHelp; }
            void RegisterArguments() noexcept override
            {
                switch (m_Kind)
                {
                case kind::info:   m_hBone = m_Parser.addOption("Bone", "One bone (its compiled name); default: every bone", false, 1); break;
                case kind::mode:   m_hA = m_Parser.addOption("Mode", "On or Off", false, 1); m_hB = m_Parser.addOption("Profile", "The preset the joints without an override take: Humanoid v1, Humanoid v2 or None (one undo step; says how many joints change)", false, 1); break;
                case kind::view:
                    m_hA = m_Parser.addOption("On", "1: Joints mode on, 0: off", false, 1); m_hB = m_Parser.addOption("ResetPose", "1: the test pose back to rest", false, 1);
                    m_hC = m_Parser.addOption("Filter",  "Selected, Chain, Attention or All", false, 1);
                    m_hD = m_Parser.addOption("Include", "1: start Include a pose on the selected joint, Ball: make it a ball, Accept, Cancel", false, 1);
                    m_hE = m_Parser.addOption("Frame",   "Confirm, Snap (to the test pose's bend) or a roll in degrees: a pending frame change", false, 1);
                    m_hF = m_Parser.addOption("FrameApply", "Rotate (the range turns with the frame), Keep (the range stays) or Cancel", false, 1);
                    m_hG = m_Parser.addOption("Check", "1: start the guided check of the selected joint; Review: answer yes at its end", false, 1);
                    m_hMirror = m_Parser.addOption("Asym", "the asymmetric-pair question: Overwrite, This or Cancel", false, 1);
                    break;
                case kind::set:
                    m_hBone = m_Parser.addOption("Bone",     "The bone (its compiled name)", true, 1);
                    m_hA    = m_Parser.addOption("Geometry", "Free, Fixed, Hinge or Ball", false, 1);
                    m_hB    = m_Parser.addOption("Hinge",    "\"min max\" degrees about the frame's z", false, 1);
                    m_hC    = m_Parser.addOption("Swing",    "\"centre_y centre_z radius_y radius_z\" degrees", false, 1);
                    m_hD    = m_Parser.addOption("Twist",    "\"centre half\" degrees", false, 1);
                    m_hE    = m_Parser.addOption("Axes",     "\"swing_y swing_z twist\": each Free, Limited or Locked", false, 1);
                    m_hF    = m_Parser.addOption("Frame",    "\"x y z w\" (the joint frame in the bone's frame) or Suggested", false, 1);
                    m_hG    = m_Parser.addOption("Source",   "Custom (default), Fitted or Preset", false, 1);
                    m_hH    = m_Parser.addOption("Relaxed",  "\"swing_y swing_z twist\" degrees: the relaxed pose (a hinge: its angle is the z); 0 0 0 = the rest", false, 1);
                    m_hI    = m_Parser.addOption("Tone",     "how strongly it returns to the relaxed pose (x the body's passive stiffness; 0: none)", false, 1);
                    m_hMirror = m_Parser.addOption("Mirror", "1: the other side's bone gets the mirrored limit in the same step", false, 1);
                    m_hForce  = m_Parser.addOption("Force",  "1: overwrite a reviewed other side that differs", false, 1);
                    break;
                case kind::reset:  m_hBone = m_Parser.addOption("Bone", "The bone", true, 1); m_hMirror = m_Parser.addOption("Mirror", "1: the other side too", false, 1); break;
                case kind::mirror: m_hBone = m_Parser.addOption("Bone", "The bone whose limit goes to the other side", true, 1); m_hForce = m_Parser.addOption("Force", "1: overwrite a reviewed other side that differs", false, 1); break;
                case kind::review: m_hBone = m_Parser.addOption("Bone", "The bone", true, 1); m_hMirror = m_Parser.addOption("Mirror", "1: the other side too", false, 1); m_hG = m_Parser.addOption("By", "User: the Mark reviewed button (default: the console)", false, 1); break;
                case kind::analyze:
                    m_hA = m_Parser.addOption("Packages", "Package guids (hex, space separated) to use; default: every package on this skeleton", false, 1);
                    m_hB = m_Parser.addOption("IncludeReviewed", "1: reviewed joints may grow too (default: protected)", false, 1);
                    m_hBone = m_Parser.addOption("ShowWorst", "A bone of the proposal: its worst frame becomes the test pose", false, 1);
                    break;
                case kind::apply:  m_hA = m_Parser.addOption("Bones", "The proposal's bones to apply (space separated); default: every selected row", false, 1); break;
                }
            }
            std::string Query() noexcept override { return m_Session.RunJoint(*this); }
            xcmdline::parser::handle m_hBone, m_hA, m_hB, m_hC, m_hD, m_hE, m_hF, m_hG, m_hH, m_hI, m_hMirror, m_hForce;
        };
        joint_cmd                                               m_JointsInfo, m_JointSet, m_JointReset, m_JointMirror, m_JointReview, m_JointLimitsMode, m_JointsView, m_JointAnalyze, m_JointApplyFit;

        // The undo step every joint edit becomes: the joint section before and after, as text (xskeleton_joints::JointState)
        struct joint_edit_cmd : xundo::command_base
        {
            session& m_Session;
            joint_edit_cmd(xundo::system& System, session& Session) noexcept : command_base(System, "JointEdit", nullptr), m_Session(Session) { RegisterArguments(); }
            const char* getCommandHelp() const noexcept override { return "Applies a joint limits edit (undoable; what the Joint* commands record). Usage: JointEdit -Label text -Before state -After state"; }
            void RegisterArguments() noexcept override
            {
                m_hLabel  = m_Parser.addOption("Label",  "What the edit did", true, 1);
                m_hBefore = m_Parser.addOption("Before", "Joint state before", true, 1);
                m_hAfter  = m_Parser.addOption("After",  "Joint state after", true, 1);
            }
            std::string Redo() noexcept override
            {
                std::string After;
                if (!xeditor::cmd_util::GetArg(m_Parser, m_hAfter, After)) return "JointEdit: bad arguments";
                auto* pDesc = m_Session.Desc();
                if (!pDesc) return "JointEdit: nothing loaded";
                if (!xskeleton_joints::ApplyJointState(*pDesc, After)) return "JointEdit: unreadable state";
                m_Session.m_Document.m_bDirty = true;
                return {};
            }
            void BackupCurrenState(xundo::undo_file& File) noexcept override { std::string Before; xeditor::cmd_util::GetArg(m_Parser, m_hBefore, Before); xeditor::WriteString(File, Before); }
            void Undo(xundo::undo_file& File) noexcept override
            {
                const std::string Before = xeditor::ReadString(File);
                if (auto* pDesc = m_Session.Desc()) { xskeleton_joints::ApplyJointState(*pDesc, Before); m_Session.m_Document.m_bDirty = true; }
            }
            xcmdline::parser::handle m_hLabel, m_hBefore, m_hAfter;
        };
        joint_edit_cmd                                          m_JointEdit;

        xskeleton_desc::details                                 m_Details;              // the imported hierarchy, from the compiler's log
        set_bone_cmd                                            m_SetBone;
        match_lod_cmd                                           m_MatchLOD;
        set_mask_weight_cmd                                     m_SetMaskWeight;
        xeditor::set_preview_cmd<view_settings>                 m_SetPreview;
        xeditor::list_preview_cmd<view_settings>                m_ListPreview;
        query_cmd                                               m_ListBones, m_SelectBones, m_ClearSelection, m_ListLayers;

        runtime                                                 m_Scene;
        xeditor::camera_toolbar                                 m_CameraToolbar;
        xeditor::transform_gizmo::state                         m_Gizmo;                    // the Move / Rotate / Scale gizmo of the view: the descriptor's PreTransform
        xeditor::transform_gizmo::actions                       m_GizmoActions{ m_Gizmo };  // its keys (Preview/ToolMove...)
        xeditor::transform_gizmo::pre                           m_Compiled;                 // the PreTransform the loaded skeleton was compiled with: the view shows the one being edited against it
        xeditor::transform_gizmo::pre                           m_CompiledPost;             // and its PostTransform
        void RegisterActions(ximgui::actions::context& Ctx) noexcept override { Ctx.Scope(m_GizmoActions, "Preview"); }
        xeditor::camera_cmds                                    m_CameraCmds;
        view_settings                                           m_Settings;
        xeditor::inspector_panel                                m_SettingsInspector{ "Skeleton View" };

        xrsc::skeleton                                          m_Ref;
        std::string                                             m_ErrorMessage;
        std::unordered_map<std::uint32_t, std::string>          m_Names, m_RawNames;    // by CRC of the display name: the name, and the raw import name it came from
        std::vector<bone_world>                                 m_WorldFrozen, m_WorldBind, m_Scaled;
        std::vector<bool>                                       m_IsTwist;
        xmath::fvec3                                            m_Center = xmath::fvec3(0, 0, 0);
        float                                                   m_Radius = 1.0f;
        bool                                                    m_bLastNormalize = true;

        std::set<int>                                           m_Selected;
        int                                                     m_iScrollTo     = -1;   // the tree scrolls to (and opens down to) this bone once: one hovered or selected in the 3D view
        std::vector<bool>                                       m_ScrollPath;           // the bones on the way to it (opened)
        int                                                     m_iLastHover3D  = -1;
        std::set<int>                                           m_LastSelected;
        int                                                     m_iAnchor = -1;
        int                                                     m_iHovered = -1;
        bool                                                    m_bTreeHovered = false;
        std::string                                             m_RenamingBone, m_RenameBuffer;
        bool                                                    m_bRenameStarted = false;

        std::vector<e19::draw_vert>                             m_Outline, m_Fill;

        // Joints mode (J2): view state only - never saved, never in the undo history
        bool                                                    m_bJointsMode   = false;
        bool                                                    m_bJointSymmetry = true;
        std::string                                             m_JointKey;                 // what m_JointRes was resolved from

        // J4 "Analyze clips": evidence from the packages on this skeleton, and the proposal (never applied by itself)
        struct jfit_source { std::string m_Package; std::wstring m_Path; bool m_bUse = true; bool m_bStale = false; xskeleton_joint_evidence::evidence m_E; };
        struct jfit_row
        {
            int                             m_Bone = -1;
            xanimation::joint_limit         m_New;
            xanimation::joint_fit_report    m_Rep;
            int                             m_nCoverage = 0;            // distinct poses (5 degree cells), not raw frames
            int                             m_iSrc = -1, m_iClip = -1, m_iFrame = -1;     // the worst sample
            bool                            m_bProtected = false;       // reviewed: kept unless "include reviewed"
            bool                            m_bSelected  = true;
            bool                            m_bChanges   = false;       // the proposed limit differs from the current one (off-model samples alone change nothing)
        };
        std::vector<jfit_source>                                m_FitSources;
        std::vector<jfit_row>                                   m_FitRows;
        std::string                                             m_FitKey;               // the joint state + skeleton the proposal was made from
        int                                                     m_FitFrames = 0, m_FitFree = 0;
        bool                                                    m_bFitIncludeReviewed = false, m_bFitOpen = false;
        xskeleton_joints::resolution                            m_JointRes;
        std::vector<std::string>                                m_JNames;
        std::vector<std::int32_t>                               m_JParents;
        xanimation::pose                                        m_JRest;
        xanimation::body_ik                                     m_Ik;
        bool                                                    m_bTestPose     = false;    // the sandbox pose: stays after a pull until Reset pose / leaving the mode
        xanimation::pose                                        m_TestPose;
        std::vector<xmath::fquat>                               m_CurR;                     // model rotations of the pose shown (rest or sandbox)
        std::vector<xmath::fvec3>                               m_FrozenView;               // view positions before the sandbox pose
        bool                                                    m_bPulling = false, m_bPullHover = false;
        int                                                     m_PullBone = -1, m_PullEff = -1;
        xanimation::pose                                        m_PullBase;
        xmath::fvec3                                            m_PullPlane{}, m_PullTargetView{};
        xanimation::ik_task_report                              m_PullRep;
        std::string                                             m_PullNote, m_JointMsg;
        float                                                   m_JEdit[8] = {};            // the panel's numeric fields while one is being edited
        int                                                     m_JEditBone = -1;
        ImVec2                                                  m_HandleScreen{};
        // J3: direct handles, Include this pose, frame adjustment, the guided check, filters (view state; edits go through JointSet)
        enum class joint_filter : int { SELECTED, CHAIN, ATTENTION, ALL };
        joint_filter                                            m_JFilter = joint_filter::SELECTED;
        struct joint_handle { int m_Kind; ImVec2 m_Screen; };
        std::vector<joint_handle>                               m_JHandles;                 // the selected joint's handles this frame
        int                                                     m_JHandleHover = -1, m_JDrag = -1;
        bool                                                    m_JDragShift = false;
        xanimation::joint_limit                                 m_JDragStart, m_JPreview;
        ImVec2                                                  m_JDragMouse0{}, m_JDragHandle0{};
        std::string                                             m_JDragTip;
        bool                                                    m_bJAsymPopup = false;      // a mirrored edit met a reviewed, different other side
        std::string                                             m_JAsymMsg, m_JAsymName, m_JAsymArgs;
        bool                                                    m_bJInclude = false, m_bJIncludeBall = false;
        int                                                     m_JIncludeBone = -1;
        xanimation::joint_limit                                 m_JIncludeProposal;
        xanimation::joint_fit_report                            m_JIncludeRep;
        bool                                                    m_bJIncludeValid = false;
        bool                                                    m_bJFramePending = false;   // a frame change waiting for "rotate the envelope" / "keep the envelope"
        int                                                     m_JFrameBone = -1;
        xmath::fquat                                            m_JFrameNew = xmath::fquat::fromIdentity();
        float                                                   m_JFrameRoll = 0;
        float                                                   m_JCheckT = -1;             // the guided check: seconds into it; -1 off, -2 finished (asks)
        int                                                     m_JCheckBone = -1;
        std::string                                             m_JCheckLabel;

        session(xresource::full_guid Guid, xresource_editor::library::guid LibraryGuid, xgpu::device* pDevice) noexcept
            : descriptor_editor("Skeleton", Guid, LibraryGuid, pDevice)
            , m_SetBone(m_Undo, m_Document, m_Details), m_MatchLOD(m_Undo, m_Document, m_Details), m_SetMaskWeight(m_Undo, m_Document, m_Details)
            , m_SetPreview(m_Undo, m_Settings), m_ListPreview(m_Undo, m_Settings)
            , m_CameraCmds(m_Undo, m_Scene.Camera())
            , m_ListBones      (m_Undo, *this, query_cmd::kind::list_bones,      "ListBones",      "Every compiled bone: index, name, raw name, parent, flags, LOD and its override. Usage: ListBones")
            , m_SelectBones    (m_Undo, *this, query_cmd::kind::select_bones,    "SelectBones",    "Selects bones in the view. Usage: SelectBones -Bones \"one raw name per line\" [-Add true]")
            , m_ClearSelection (m_Undo, *this, query_cmd::kind::clear_selection, "ClearSelection", "Selects no bone. Usage: ClearSelection")
            , m_ListLayers     (m_Undo, *this, query_cmd::kind::list_layers,     "ListLayers",     "The mask layers and how many bones each weights. Usage: ListLayers")
            , m_JointsInfo     (m_Undo, *this, joint_cmd::kind::info,   "JointsInfo",  "Each bone's joint limit as it resolves now (override, preset or unspecified), in degrees, and whether limits are on. Usage: JointsInfo [-Bone name]")
            , m_JointSet       (m_Undo, *this, joint_cmd::kind::set,    "JointSet",    "Sets a bone's joint limit (an override; starts from what it resolves to now). One undo step; a reviewed joint becomes 'needs review'. Usage: JointSet -Bone name [-Geometry Ball] [-Hinge \"min max\"] [-Swing \"cy cz ry rz\"] [-Twist \"centre half\"] [-Axes \"y z twist\"] [-Frame \"x y z w\"|Suggested] [-Source Custom] [-Mirror 1] [-Force 1]")
            , m_JointReset     (m_Undo, *this, joint_cmd::kind::reset,  "JointReset",  "Removes a bone's override: it resolves from the profile again. Usage: JointReset -Bone name [-Mirror 1]")
            , m_JointMirror    (m_Undo, *this, joint_cmd::kind::mirror, "JointMirror", "Gives the bone on the other side the mirror of this bone's limit (frame aware). Refuses to overwrite a reviewed other side that differs unless -Force 1. Usage: JointMirror -Bone name [-Force 1]")
            , m_JointReview    (m_Undo, *this, joint_cmd::kind::review, "JointReview", "Marks a bone's limit reviewed (from the console: recorded as such). Usage: JointReview -Bone name [-Mirror 1]")
            , m_JointLimitsMode(m_Undo, *this, joint_cmd::kind::mode,   "JointLimits", "Turns the skeleton's joint limits on or off (off: every joint free, as before). Usage: JointLimits -Mode On|Off")
            , m_JointAnalyze   (m_Undo, *this, joint_cmd::kind::analyze, "JointAnalyze", "Analyze clips: what the packages on this skeleton do with each joint, against its current limits - a proposal (widen only, reviewed joints protected), never applied by itself. Usage: JointAnalyze [-Packages \"hex ...\"] [-IncludeReviewed 1] [-ShowWorst bone]")
            , m_JointApplyFit  (m_Undo, *this, joint_cmd::kind::apply,  "JointApplyProposal", "Applies the Analyze clips proposal (all selected rows, or -Bones) as ONE undo step; refused when the joints or the skeleton changed since. Usage: JointApplyProposal [-Bones \"a b\"]")
            , m_JointsView     (m_Undo, *this, joint_cmd::kind::view,   "JointsView",  "Joints mode (view only, not saved): on/off, reset the test pose; reports the selected joint, the pull handle's position (window pixels) and the last pull. Usage: JointsView [-On 1|0] [-ResetPose 1]")
            , m_JointEdit      (m_Undo, *this)
        {
            m_SettingsInspector.BindObject(*xproperty::getObjectByType<view_settings>(), &m_Settings);
            m_bLastNormalize = m_Settings.m_bNormalizeSize;

            AddPanel("Skeleton View",      dock::left,   [this] { m_SettingsInspector.Show(); });
            if (pDevice) m_CameraToolbar.Init(m_Document.m_Guid.m_Instance.m_Value, 390.0f, 270.0f);      // with the gizmo's tools
            AddPanel("Skeleton Viewport",  dock::center, [this] { RenderViewport(); }, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            AddPanel("Bone Hierarchy",     dock::right,  [this] { RenderHierarchy(); });
            AddPanel("Description",        dock::right,  [this] { m_DescriptorInspector.Show(); });
            AddPanel("Joints",             dock::right,  [this] { RenderJointsPanel(); });

            m_bReady = pDevice && m_Scene.Init(*pDevice);
            Reload();
        }

        ~session() noexcept override
        {
            xresource::g_Mgr.ReleaseRef(m_Ref);
            m_Scene.Release();
        }

        bool m_bReady = false;

        desc*                     Desc() noexcept { return m_Document.isLoaded() ? static_cast<desc*>(m_Document.m_pDescriptor.get()) : nullptr; }
        xskeleton::skeleton*      Skeleton() noexcept { return m_Ref.empty() ? nullptr : xresource::g_Mgr.getResource(m_Ref); }

        void OnCompileStarted() noexcept override { xresource::g_Mgr.ReleaseRef(m_Ref); m_Ref.clear(); }
        void OnCompiled()       noexcept override { Reload(); }
        void OnDescriptorReplaced() noexcept override { RebuildNames(); }

        //----------------------------------------------------------------------------------------
        // Loading
        //----------------------------------------------------------------------------------------

        // The names the compiler saw, and the descriptor's own: hashed the way the compiler hashes a bone's name
        void RebuildNames() noexcept
        {
            m_Names.clear();
            m_RawNames.clear();
            auto Add = [&](std::string_view Raw)
            {
                const std::string Display = StripVBoneTag(Raw);
                const auto Hash = xstrtool::CRC32(Display);
                m_Names[Hash]    = Display;
                m_RawNames[Hash] = std::string(Raw);
            };
            for (auto& Name : m_Details.m_BoneList) Add(Name);
            Add("RuntimeRoot");     // xraw3d::anim::synthetic_root_name_v: made by the compiler, not a source node (skeleton_runtime_folding_plan.md)
            if (auto* pDesc = Desc())
            {
                std::function<void(const xskeleton_desc::bone&)> Walk = [&](const xskeleton_desc::bone& Node) { Add(Node.m_Name); for (auto& Child : Node.m_Bones) Walk(Child); };
                Walk(pDesc->m_RootBone);
            }
        }

        // The compiled skeleton, the names, the poses and what frames the view. In memory only: the descriptor gets an entry for every imported
        // bone so the whole hierarchy is editable, but saving is what writes it.
        void Reload() noexcept
        {
            xresource::g_Mgr.ReleaseRef(m_Ref);
            m_Ref.clear();
            m_ErrorMessage.clear();
            m_Selected.clear();
            m_iAnchor = m_iHovered = -1;
            m_JointKey.clear(); m_bTestPose = m_bPulling = false;

            auto* pDesc = Desc();
            if (!pDesc) { m_ErrorMessage = "The descriptor could not be read."; return; }

            m_Details = {};
            xtextfile::stream File;
            if (!File.Open(true, m_Document.m_LogPath + L"\\Details.txt", {}))
            {
                xproperty::settings::context Context;
                if (xproperty::sprop::serializer::Stream(File, m_Details, Context)
                 || (m_Details.m_bCompactTree && m_Details.StreamCompactTree(File, m_Details.m_RootBone))) m_Details = {};
            }
            const auto Before = m_Document.Snapshot();
            pDesc->MergeWithDetails(m_Details);
            if (m_Document.Snapshot() != Before) m_Document.m_bDirty = true;
            RebuildNames();

            if (!std::filesystem::exists(m_Document.m_ResourcePath)) { m_ErrorMessage = "Not compiled yet: compile the skeleton to see it."; return; }
            m_Ref.m_Instance = m_Document.m_Guid.m_Instance;
            auto* pSkeleton = xresource::g_Mgr.getResource(m_Ref);
            if (!pSkeleton) { m_Ref.clear(); m_ErrorMessage = "The compiled skeleton could not be loaded."; return; }
            m_Compiled     = xeditor::transform_gizmo::PreOf(pDesc->m_PreTransform);
            m_CompiledPost = xeditor::transform_gizmo::PreOf(pDesc->m_PostTransform);

            ComputePoses(*pSkeleton);
            const auto Bones = pSkeleton->getBones();
            m_IsTwist.assign(Bones.size(), false);
            for (int i = 0; i < int(Bones.size()); ++i)
                if (auto It = m_Names.find(pSkeleton->getBoneNames()[i].m_NameHash); It != m_Names.end()) m_IsTwist[i] = xstrtool::findI(It->second, "Twist") != std::string::npos;
        }

        // FROZEN: forward kinematics of the rest pose. BIND: the pose the mesh was skinned in, where the compiler saw skinning; a bone with none
        // hangs off its parent by its rest offset, or it would jump to an unrelated placement.
        void ComputePoses(const xskeleton::skeleton& Skeleton) noexcept
        {
            const auto Bones = Skeleton.getBones();
            const auto Rests = Skeleton.getBoneRests();
            m_WorldFrozen.resize(Bones.size());
            m_WorldBind.resize(Bones.size());
            std::vector<xmath::fmat4> Mats(Bones.size());

            xmath::fvec3 Center(0.0f, 0.0f, 0.0f);
            for (std::size_t i = 0; i < Bones.size(); ++i)
            {
                const xmath::fmat4 Local = Rests[i].m_RestPose.toMatrix();
                Mats[i] = Bones[i].m_iParent < 0 ? Local : Mats[Bones[i].m_iParent] * Local;
                auto& W = m_WorldFrozen[i];
                W.m_Position = Mats[i].ExtractPosition(); W.m_Right = Mats[i].Right(); W.m_Up = Mats[i].Up();
                Center += W.m_Position;
            }
            // The bind pose. A bone a mesh is bound to is where its bind puts it. One with no bind of its own (a CC rig's upper arm or forearm, moved
            // only through its twist bones) is placed from its nearest bound descendant, keeping their offset from the frozen pose: that offset
            // is the same in both poses (a twist bone only rolls, the wrist sits at the end of the forearm), while its own local rotation is not
            // (the arm hangs in one pose and is out in the other - composing it under the parent's bind put the elbow under the shoulder).
            // A bone with no bound descendant either follows its parent's bind with its frozen local transform.
            {
                std::vector<xmath::fmat4> Frozen = Mats;                            // the frozen world matrices from the loop above
                std::vector<int>          iBound(Bones.size(), -1);                 // nearest bound descendant (or itself)
                for (int i = static_cast<int>(Bones.size()) - 1; i >= 0; --i)     // children come after their parents: walk up
                {
                    if (!Bones[i].m_InvBindPose.isIdentity()) iBound[i] = i;
                    if (const int p = Bones[i].m_iParent; p >= 0 && iBound[p] != p && iBound[i] >= 0 && (iBound[p] < 0 || iBound[i] < iBound[p])) iBound[p] = iBound[i];
                }
                for (std::size_t i = 0; i < Bones.size(); ++i)
                {
                    auto& W = m_WorldBind[i];
                    W.m_bRealBindData = !Bones[i].m_InvBindPose.isIdentity();
                    if (W.m_bRealBindData)          Mats[i] = Bones[i].m_InvBindPose.Inverse();
                    else if (const int d = iBound[i]; d >= 0)
                                                    Mats[i] = Bones[d].m_InvBindPose.Inverse() * Frozen[d].Inverse() * Frozen[i];
                    else
                    {
                        const xmath::fmat4 Local = Rests[i].m_RestPose.toMatrix();
                        Mats[i] = Bones[i].m_iParent < 0 ? Local : Mats[Bones[i].m_iParent] * Local;
                    }
                    W.m_Position = Mats[i].ExtractPosition(); W.m_Right = Mats[i].Right(); W.m_Up = Mats[i].Up();
                }
            }

            if (!Bones.empty()) Center /= float(Bones.size());
            float Radius = 0.5f;
            for (auto& W : m_WorldFrozen) Radius = std::max(Radius, (W.m_Position - Center).Length());
            for (auto& W : m_WorldBind)   Radius = std::max(Radius, (W.m_Position - Center).Length());
            m_Center = Center;
            m_Radius = Radius;
            m_Scene.m_bReframe = true;
        }

        //----------------------------------------------------------------------------------------
        // Names and selection
        //----------------------------------------------------------------------------------------

        float ViewScale() const noexcept { return m_Settings.m_bNormalizeSize && m_Radius > 1.0e-6f ? 1.0f / m_Radius : 1.0f; }

        std::string DisplayName(const xskeleton::skeleton& Skeleton, int iBone) const noexcept
        {
            const auto Hash = Skeleton.getBoneNames()[iBone].m_NameHash;
            if (auto It = m_Names.find(Hash); It != m_Names.end()) return It->second;
            return std::format("0x{:08X}", Hash);
        }

        // What the compiler matches overrides against: the name as imported, with its "vbone_" tag
        std::string RawName(const xskeleton::skeleton& Skeleton, int iBone) const noexcept
        {
            if (auto It = m_RawNames.find(Skeleton.getBoneNames()[iBone].m_NameHash); It != m_RawNames.end()) return It->second;
            return DisplayName(Skeleton, iBone);
        }

        std::string EffectiveName(const xskeleton::skeleton& Skeleton, int iBone) noexcept
        {
            if (auto* pDesc = Desc())
                if (const auto* pOverride = FindBoneOverride(pDesc->m_RootBone, RawName(Skeleton, iBone)); pOverride && !pOverride->m_Rename.empty()) return pOverride->m_Rename;
            return DisplayName(Skeleton, iBone);
        }

        // A plain click replaces the selection; Ctrl toggles one bone; Shift selects the bones between the anchor and this one, by index
        // (bones are stored parent before child, so the index tracks the hierarchy closely enough to be a sensible range)
        void Select(int iBone, bool bCtrl, bool bShift) noexcept
        {
            if (bShift && m_iAnchor != -1)
            {
                if (!bCtrl) m_Selected.clear();
                for (int i = std::min(m_iAnchor, iBone); i <= std::max(m_iAnchor, iBone); ++i) m_Selected.insert(i);
            }
            else if (bCtrl)
            {
                if (!m_Selected.erase(iBone)) m_Selected.insert(iBone);
                m_iAnchor = iBone;
            }
            else
            {
                m_Selected = { iBone };
                m_iAnchor  = iBone;
            }
        }

        void ClearSelectionUnlessModifier(bool bCtrl, bool bShift) noexcept
        {
            if (bCtrl || bShift) return;
            m_Selected.clear();
            m_iAnchor = -1;
        }

        // The bones an edit of this one applies to: the whole selection when it is part of a multi-bone selection, this one otherwise
        std::string EditTargets(const xskeleton::skeleton& Skeleton, int iBone, const std::string& Raw) const noexcept
        {
            std::string Text;
            if (iBone != -1 && m_Selected.count(iBone) && m_Selected.size() > 1) { for (int i : m_Selected) Text += RawName(Skeleton, i) + "\n"; }
            else Text = Raw;
            return xeditor::Quote(Text);
        }

        //----------------------------------------------------------------------------------------
        // Joint limits
        //----------------------------------------------------------------------------------------

        // the compiled skeleton as the joint resolution needs it: compiled names, parents, rest
        bool JointRig(std::vector<std::string>& Names, std::vector<std::int32_t>& Parents, xanimation::pose& Rest) noexcept
        {
            auto* pSkeleton = Skeleton();
            if (!pSkeleton) return false;
            const auto Bones = pSkeleton->getBones(); const auto Rests = pSkeleton->getBoneRests();
            const int N = int(Bones.size());
            Names.resize(N); Parents.resize(N); Rest.resize(N);
            for (int i = 0; i < N; ++i)
            {
                Names[i] = EffectiveName(*pSkeleton, i); Parents[i] = Bones[i].m_iParent;
                Rest.m_Position[i] = Rests[i].m_RestPose.m_Position; Rest.m_Rotation[i] = Rests[i].m_RestPose.m_Rotation.NormalizeSafeCopy(); Rest.m_Scale[i] = Rests[i].m_RestPose.m_Scale;
            }
            return true;
        }

        std::string DescriptorSnapshot() noexcept
        {
            xproperty::settings::context C;
            return xproperty::ui::undo::SnapshotToString(*m_Document.m_pDescriptor->getProperties(), m_Document.m_pDescriptor.get(), C);
        }
        void ApplyDescriptorSnapshot(const std::string& S) noexcept
        {
            xproperty::settings::context C;
            xproperty::ui::undo::ApplySnapshotFromString(*m_Document.m_pDescriptor->getProperties(), m_Document.m_pDescriptor.get(), S, C);
        }

        // An edit of the joint overrides as ONE undo step: Edit changes the descriptor (or returns an error, and nothing changes); an edit
        // that changes nothing leaves no undo entry. Review states follow the rule: a reviewed joint whose geometry changed needs review.
        std::string CommitJointEdit(const std::string& Label, const std::function<std::string(desc&)>& Edit) noexcept
        {
            auto* pDesc = Desc();
            if (!pDesc) return "nothing loaded";
            const auto Before = xskeleton_joints::JointState(*pDesc);
            if (auto Err = Edit(*pDesc); !Err.empty()) { xskeleton_joints::ApplyJointState(*pDesc, Before); return Err; }
            for (auto& O : pDesc->m_JointOverrides)
                if (O.m_Review != xskeleton_desc::joint_review::PENDING && !O.m_ReviewedHash.empty())
                    O.m_Review = xskeleton_joints::ReviewHash(O) == O.m_ReviewedHash ? xskeleton_desc::joint_review::REVIEWED : xskeleton_desc::joint_review::NEEDS_REVIEW;
            const auto After = xskeleton_joints::JointState(*pDesc);
            if (After == Before) return "no change";
            xskeleton_joints::ApplyJointState(*pDesc, Before);
            if (auto Err = m_Undo.Execute(std::format("JointEdit -Label {} -Before {} -After {}", xeditor::Quote(Label), xeditor::Quote(Before), xeditor::Quote(After))); !Err.empty()) return Err;
            m_Document.m_bDirty = true;
            return {};
        }

        static std::vector<float> Floats(const std::string& S)
        {
            std::vector<float> V; std::size_t i = 0;
            while (i < S.size())
            {
                while (i < S.size() && (S[i] == ' ' || S[i] == ',')) ++i;
                if (i >= S.size()) break;
                float f = 0; auto R = std::from_chars(S.data() + i, S.data() + S.size(), f);
                if (R.ec != std::errc{}) return {};
                V.push_back(f); i = std::size_t(R.ptr - S.data());
            }
            return V;
        }
        static bool AxisWord(std::string_view W, xskeleton_desc::joint_axis& A)
        {
            if (xstrtool::findI(W, "free") == 0) { A = xskeleton_desc::joint_axis::FREE; return true; }
            if (xstrtool::findI(W, "lim") == 0)  { A = xskeleton_desc::joint_axis::LIMITED; return true; }
            if (xstrtool::findI(W, "lock") == 0) { A = xskeleton_desc::joint_axis::LOCKED; return true; }
            return false;
        }

        // the mirrored override of Src's limit for bone Dst (frame aware); empty when there is no mirror bone
        std::string MirrorInto(desc& D, const xskeleton_joints::resolution& R, std::span<const std::string> Names, std::span<const std::int32_t> Parents, const xanimation::pose& Rest
                              , int Src, bool bForce, std::string& Note)
        {
            std::vector<std::string_view> Views(Names.begin(), Names.end());
            const int Dst = xanimation::FindMirrorBone(Views, Src);
            if (Dst < 0 || Dst == Src) return std::format("'{}' has no bone on the other side (a centre bone, or an unknown name pattern)", Names[Src]);
            const int N = int(Names.size());
            std::vector<xmath::fquat> W(N);
            for (int i = 0; i < N; ++i) W[i] = Parents[i] < 0 ? Rest.m_Rotation[i] : W[Parents[i]] * Rest.m_Rotation[i];
            const float Fd = xanimation::MeasureFacingDeg(xanimation::skeleton_view{ Parents }, Views, Rest) * xskeleton_joints::deg_v;
            const xmath::fvec3 Lateral(std::cos(Fd), 0, -std::sin(Fd));
            xanimation::frame_confidence C;
            const auto DstFrame = R.m_Limits[Dst].m_Type != xanimation::joint_type::FREE ? R.m_Limits[Dst].m_Frame : xanimation::SuggestFrame(Parents, Rest, Dst, C);
            const auto DstLong = W[Dst] * (DstFrame * xmath::fvec3(1, 0, 0));
            float Asym = 0;
            const auto M = xanimation::Mirror(R.m_Limits[Src], W[Src], W[Dst], Rest.m_Rotation[Dst], Lateral, &DstLong, &Asym);
            xskeleton_desc::joint_override O;
            if (const auto* pOld = D.findJointOverride(Names[Dst])) O = *pOld;
            O.m_BoneName = Names[Dst];
            const bool bSameFrame = std::abs(xmath::fquat::Dot(M.m_Frame, DstFrame)) > std::cos(0.25f * xskeleton_joints::deg_v);
            xskeleton_joints::SetOverrideFields(O, M, !bSameFrame);
            if (const auto* pOld = D.findJointOverride(Names[Dst]); pOld && pOld->m_Review == xskeleton_desc::joint_review::REVIEWED && !bForce
                && xskeleton_joints::ReviewHash(O) != pOld->m_ReviewedHash)
                return std::format("'{}' is reviewed and differs from the mirror of '{}' (an asymmetric pair): -Force 1 overwrites it", Names[Dst], Names[Src]);
            if (const auto* pSrcO = D.findJointOverride(Names[Src])) O.m_Source = pSrcO->m_Source;
            O.m_Provenance = std::format("mirrored from {}", Names[Src]);
            if (auto* pOld = D.findJointOverride(Names[Dst])) *pOld = O; else D.m_JointOverrides.push_back(O);
            if (Asym > 5.0f * xskeleton_joints::deg_v) Note += std::format(" (the rest poses differ by {:.1f} degrees: check '{}')", Asym / xskeleton_joints::deg_v, Names[Dst]);
            return {};
        }

        static const char* JointCmdName(joint_cmd::kind K) noexcept
        {
            switch (K) { case joint_cmd::kind::info: return "JointsInfo"; case joint_cmd::kind::set: return "JointSet"; case joint_cmd::kind::reset: return "JointReset";
                         case joint_cmd::kind::mirror: return "JointMirror"; case joint_cmd::kind::review: return "JointReview"; case joint_cmd::kind::view: return "JointsView";
                         case joint_cmd::kind::analyze: return "JointAnalyze"; case joint_cmd::kind::apply: return "JointApplyProposal"; default: return "JointLimits"; }
        }

        std::string RunJoint(joint_cmd& Cmd) noexcept
        {
            using namespace xeditor::cmd_util;
            auto* pDesc = Desc();
            if (!pDesc) return std::format("{}: nothing loaded", JointCmdName(Cmd.m_Kind));
            std::vector<std::string> Names; std::vector<std::int32_t> Parents; xanimation::pose Rest;
            if (Cmd.m_Kind == joint_cmd::kind::view)
            {
                std::string On, Reset; GetArg(Cmd.m_Parser, Cmd.m_hA, On); GetArg(Cmd.m_Parser, Cmd.m_hB, Reset);
                if (!On.empty()) { m_bJointsMode = On == "1" || On == "true"; if (m_bJointsMode) m_Settings.m_Pose = pose_mode::FROZEN; else { m_bTestPose = m_bPulling = false; m_PullNote.clear(); } }
                if (Reset == "1" || Reset == "true") { m_bTestPose = false; m_PullNote.clear(); }
                const int b = SelectedJoint();
                std::string Filter, Inc, Fr, FrA, Chk, Asym, Said;
                GetArg(Cmd.m_Parser, Cmd.m_hC, Filter); GetArg(Cmd.m_Parser, Cmd.m_hD, Inc); GetArg(Cmd.m_Parser, Cmd.m_hE, Fr); GetArg(Cmd.m_Parser, Cmd.m_hF, FrA); GetArg(Cmd.m_Parser, Cmd.m_hG, Chk); GetArg(Cmd.m_Parser, Cmd.m_hMirror, Asym);
                UpdateJoints();
                std::vector<std::string_view> Vw(m_JNames.begin(), m_JNames.end());
                const int Oth = b >= 0 && b < int(m_JNames.size()) ? xanimation::FindMirrorBone(Vw, b) : -1;
                const bool bSymV = m_bJointSymmetry && Oth >= 0 && Oth != b;
                if (!Filter.empty()) m_JFilter = xstrtool::findI(Filter, "chain") == 0 ? joint_filter::CHAIN : xstrtool::findI(Filter, "att") == 0 ? joint_filter::ATTENTION : xstrtool::findI(Filter, "all") == 0 ? joint_filter::ALL : joint_filter::SELECTED;
                if (!Inc.empty() && b >= 0)
                {
                    if (Inc == "1") { m_bJInclude = true; m_bJIncludeBall = false; m_JIncludeBone = b; m_bTestPose = false; m_PullNote.clear(); }
                    else if (xstrtool::findI(Inc, "ball") == 0) m_bJIncludeBall = true;
                    else if (xstrtool::findI(Inc, "acc") == 0) Said = IncludeAccept(bSymV);
                    else { m_bJInclude = m_bJIncludeBall = false; m_bTestPose = false; m_PullNote.clear(); m_JointKey.clear(); }
                }
                if (!Fr.empty() && b >= 0 && b < int(m_JNames.size()))
                {
                    const auto& Lb = m_JointRes.m_Limits[b];
                    if (xstrtool::findI(Fr, "conf") == 0) Said = JointCommand(std::format("JointSet -Bone {} -Frame \"{} {} {} {}\"{}", xeditor::Quote(m_JNames[b]), Lb.m_Frame.m_X, Lb.m_Frame.m_Y, Lb.m_Frame.m_Z, Lb.m_Frame.m_W, bSymV ? " -Mirror 1" : ""));
                    else if (xstrtool::findI(Fr, "clip") == 0) { xmath::fquat Fq; if (FrameFromClips(b, Fq)) { m_bJFramePending = true; m_JFrameBone = b; m_JFrameNew = Fq; Said = "frame pending (snapped to the clips' bend axis)"; } else Said = "snap to clips: no package evidence moves this joint"; }
                    else if (xstrtool::findI(Fr, "snap") == 0) { xmath::fquat Fq; if (FrameFromPoseBend(b, Fq)) { m_bJFramePending = true; m_JFrameBone = b; m_JFrameNew = Fq; Said = "frame pending (snapped to the pose's bend)"; } else Said = "snap: the joint is not bent in the test pose"; }
                    else { char* pEnd = nullptr; const float Roll = std::strtof(Fr.c_str(), &pEnd); m_bJFramePending = true; m_JFrameBone = b; m_JFrameRoll = Roll; m_JFrameNew = (Lb.m_Frame * xanimation::details::AxisAngle(xanimation::details::AxX, Roll * xskeleton_joints::deg_v)).NormalizeCopy(); Said = "frame pending (rolled)"; }
                }
                if (!FrA.empty()) { if (xstrtool::findI(FrA, "rot") == 0) Said = ApplyFrame(false, bSymV); else if (xstrtool::findI(FrA, "keep") == 0) Said = ApplyFrame(true, bSymV); else { m_bJFramePending = false; m_JFrameRoll = 0; } }
                if (Chk == "1" && b >= 0) { m_JCheckT = 0; m_JCheckBone = b; }
                if (xstrtool::findI(Chk, "rev") == 0 && m_JCheckT == -2 && m_JCheckBone >= 0) { Said = JointCommand(std::format("JointReview -Bone {}{} -By User", xeditor::Quote(m_JNames[m_JCheckBone]), bSymV ? " -Mirror 1" : "")); m_JCheckT = -1; }
                if (!Asym.empty() && m_bJAsymPopup) Said = ResolveAsym(xstrtool::findI(Asym, "over") == 0 ? 0 : xstrtool::findI(Asym, "this") == 0 ? 1 : 2);
                std::string Hs;
                for (const auto& H : m_JHandles) Hs += std::format("{}{} {:.0f} {:.0f}", Hs.empty() ? "" : ", ", JHTag(H.m_Kind), H.m_Screen.x, H.m_Screen.y);
                static constexpr const char* FiltN[] = { "selected", "chain", "attention", "all" };
                return std::format("JointsView: {}; selected {}; handle {}; test pose {}; last pull: {}; handles: {}; filter {}; attention {}; include {}; frame pending {}; check {}; asym popup {}{}{}"
                    , m_bJointsMode ? "on" : "off", b >= 0 && b < int(m_JNames.size()) ? m_JNames[b] : "(none)"
                    , m_PullEff >= 0 ? std::format("{:.0f} {:.0f}", m_HandleScreen.x, m_HandleScreen.y) : "(none)", m_bTestPose ? "yes" : "no", m_PullNote.empty() ? "(none)" : m_PullNote
                    , Hs.empty() ? "(none)" : Hs, FiltN[int(m_JFilter)], JointAttention().size()
                    , !m_bJInclude ? std::string("off") : m_bJIncludeValid ? std::format("on (proposal {}; off-model {})", xskeleton_joints::Describe(m_JIncludeProposal), m_JIncludeRep.m_nOffModel) : std::string("on (no pose yet)")
                    , m_bJFramePending ? "yes" : "no", m_JCheckT == -1 ? std::string("off") : m_JCheckT == -2 ? std::string("asking") : std::format("{:.1f} s", m_JCheckT)
                    , m_bJAsymPopup ? "yes" : "no", Said.empty() ? "" : "; said: ", Said);
            }
            if (Cmd.m_Kind != joint_cmd::kind::mode && !JointRig(Names, Parents, Rest)) return std::format("{}: no compiled skeleton (compile it first)", JointCmdName(Cmd.m_Kind));
            if (Cmd.m_Kind == joint_cmd::kind::analyze)
            {
                std::string Pk, Inc, Worst; GetArg(Cmd.m_Parser, Cmd.m_hA, Pk); GetArg(Cmd.m_Parser, Cmd.m_hB, Inc); GetArg(Cmd.m_Parser, Cmd.m_hBone, Worst);
                if (!Worst.empty()) return "JointAnalyze: " + FitShowWorst(Worst);
                FitLoadSources();
                if (!Pk.empty()) for (auto& S : m_FitSources) S.m_bUse = xstrtool::findI(Pk, S.m_Package) != std::string::npos;
                m_bFitIncludeReviewed = Inc == "1" || Inc == "true";
                return "JointAnalyze: " + FitAnalyze();
            }
            if (Cmd.m_Kind == joint_cmd::kind::apply)
            {
                std::string Bs; GetArg(Cmd.m_Parser, Cmd.m_hA, Bs);
                if (!Bs.empty()) for (auto& Rw : m_FitRows) Rw.m_bSelected = Rw.m_Bone >= 0 && Rw.m_Bone < int(m_JNames.size()) && (" " + Bs + " ").find(" " + m_JNames[Rw.m_Bone] + " ") != std::string::npos;
                return "JointApplyProposal: " + FitApply();
            }
            auto R = xskeleton_joints::Resolve(*pDesc, Names, Parents, Rest);
            std::string BoneName; GetArg(Cmd.m_Parser, Cmd.m_hBone, BoneName);
            int iBone = -1;
            for (int i = 0; i < int(Names.size()); ++i) if (Names[i] == BoneName) iBone = i;
            if (!BoneName.empty() && iBone < 0) return std::format("{}: no bone '{}'", JointCmdName(Cmd.m_Kind), BoneName);
            std::string MirrorArg, ForceArg; GetArg(Cmd.m_Parser, Cmd.m_hMirror, MirrorArg); GetArg(Cmd.m_Parser, Cmd.m_hForce, ForceArg);
            const bool bMirror = MirrorArg == "1" || MirrorArg == "true", bForce = ForceArg == "1" || ForceArg == "true";

            switch (Cmd.m_Kind)
            {
            case joint_cmd::kind::info:
            {
                std::string T = std::format("Joint limits {} (profile {}): {} overrides, {} preset ({} frames need confirmation), {} unspecified\n"
                    , pDesc->m_JointLimits == xskeleton_desc::joint_limits_mode::ON ? "ON" : "OFF (a preview: compiled skeletons have no limits)"
                    , xskeleton_joints::ProfileName(xskeleton_joints::EffectiveProfile(*pDesc)), R.m_nOverride, R.m_nPreset, R.m_nNeedFrame, R.m_nUnspecified);
                for (int i = 0; i < int(Names.size()); ++i)
                    if (iBone < 0 || i == iBone) T += std::format("{}: {} | {} | {}\n", i, Names[i], R.m_Source[i], xskeleton_joints::Describe(R.m_Limits[i]));
                for (const auto& Wn : R.m_Warnings) T += "warning: " + Wn + "\n";
                for (const auto& E : R.m_Errors) T += "error: " + E + "\n";
                return T;
            }
            case joint_cmd::kind::mode:
            {
                std::string Mode, Profile; GetArg(Cmd.m_Parser, Cmd.m_hA, Mode); GetArg(Cmd.m_Parser, Cmd.m_hB, Profile);
                if (!Profile.empty())
                {
                    xskeleton_desc::joint_profile P;
                    if (xstrtool::findI(Profile, "v2") != std::string::npos) P = xskeleton_desc::joint_profile::HUMANOID_V2;
                    else if (xstrtool::findI(Profile, "v1") != std::string::npos) P = xskeleton_desc::joint_profile::HUMANOID_V1;
                    else if (xstrtool::findI(Profile, "none") == 0) P = xskeleton_desc::joint_profile::NONE;
                    else return "JointLimits: -Profile \"Humanoid v1\", \"Humanoid v2\" or None";
                    if (!JointRig(Names, Parents, Rest)) return "JointLimits: no compiled skeleton (compile it first)";
                    // how many joints the change moves: the resolutions before and after, joint by joint
                    const auto Old = pDesc->m_JointProfile;
                    const auto A = xskeleton_joints::Resolve(*pDesc, Names, Parents, Rest);
                    pDesc->m_JointProfile = P; const auto B = xskeleton_joints::Resolve(*pDesc, Names, Parents, Rest); pDesc->m_JointProfile = Old;
                    int nChanged = 0;
                    for (std::size_t i = 0; i < A.m_Limits.size(); ++i) nChanged += xskeleton_joints::Describe(A.m_Limits[i]) != xskeleton_joints::Describe(B.m_Limits[i]);
                    const auto Err = CommitJointEdit(std::format("Joint profile {}", xskeleton_joints::ProfileName(P)), [&](desc& D) { D.m_JointProfile = P; return std::string{}; });
                    if (!Err.empty()) return "JointLimits: " + Err;
                    if (Mode.empty()) return std::format("JointLimits: profile {} ({} joints change)", xskeleton_joints::ProfileName(P), nChanged);
                }
                if (Mode.empty()) return "JointLimits: -Mode On|Off and/or -Profile name";
                const bool bOn = xstrtool::findI(Mode, "on") == 0;
                if (!bOn && xstrtool::findI(Mode, "off") != 0) return "JointLimits: -Mode On or Off";
                const auto Err = CommitJointEdit(bOn ? "Joint limits on" : "Joint limits off", [&](desc& D) { if (bOn && D.m_JointLimits != xskeleton_desc::joint_limits_mode::ON) D.m_JointProfile = xskeleton_joints::newest_profile_v;      // pinned on activation
                                                                                                     D.m_JointLimits = bOn ? xskeleton_desc::joint_limits_mode::ON : xskeleton_desc::joint_limits_mode::OFF; return std::string{}; });
                return Err.empty() ? std::format("JointLimits: {} (compile to apply)", bOn ? "on" : "off") : "JointLimits: " + Err;
            }
            case joint_cmd::kind::set:
            {
                std::string Geo, Hinge, Swing, Twist, Axes, Frame, Source;
                GetArg(Cmd.m_Parser, Cmd.m_hA, Geo); GetArg(Cmd.m_Parser, Cmd.m_hB, Hinge); GetArg(Cmd.m_Parser, Cmd.m_hC, Swing); GetArg(Cmd.m_Parser, Cmd.m_hD, Twist);
                GetArg(Cmd.m_Parser, Cmd.m_hE, Axes); GetArg(Cmd.m_Parser, Cmd.m_hF, Frame); GetArg(Cmd.m_Parser, Cmd.m_hG, Source); std::string Relaxed, Tone; GetArg(Cmd.m_Parser, Cmd.m_hH, Relaxed); GetArg(Cmd.m_Parser, Cmd.m_hI, Tone);
                std::string Note;
                const auto Err = CommitJointEdit(std::format("Joint limit {}", BoneName), [&](desc& D) -> std::string
                {
                    xskeleton_desc::joint_override O;
                    if (const auto* pOld = D.findJointOverride(BoneName)) O = *pOld;
                    else { O.m_BoneName = BoneName; xskeleton_joints::SetOverrideFields(O, R.m_Limits[iBone], false); if (O.m_Geometry == xskeleton_desc::joint_geometry::FREE && Geo.empty()) O.m_Geometry = xskeleton_desc::joint_geometry::BALL;
                           if (R.m_Limits[iBone].m_Type == xanimation::joint_type::FREE) { O.m_SwingRadius = { 45, 45 }; O.m_TwistHalf = 45; } }
                    if (!Geo.empty())
                    {
                        if (xstrtool::findI(Geo, "free") == 0) O.m_Geometry = xskeleton_desc::joint_geometry::FREE;
                        else if (xstrtool::findI(Geo, "fixed") == 0) O.m_Geometry = xskeleton_desc::joint_geometry::FIXED;
                        else if (xstrtool::findI(Geo, "hinge") == 0) O.m_Geometry = xskeleton_desc::joint_geometry::HINGE;
                        else if (xstrtool::findI(Geo, "ball") == 0) O.m_Geometry = xskeleton_desc::joint_geometry::BALL;
                        else return std::format("-Geometry '{}': Free, Fixed, Hinge or Ball", Geo);
                    }
                    if (!Hinge.empty()) { auto V = Floats(Hinge); if (V.size() != 2) return "-Hinge: \"min max\""; O.m_HingeMin = V[0]; O.m_HingeMax = V[1]; }
                    if (!Swing.empty()) { auto V = Floats(Swing); if (V.size() != 4) return "-Swing: \"centre_y centre_z radius_y radius_z\""; O.m_SwingCentre = { V[0], V[1] }; O.m_SwingRadius = { V[2], V[3] }; }
                    if (!Twist.empty()) { auto V = Floats(Twist); if (V.size() != 2) return "-Twist: \"centre half\""; O.m_TwistCentre = V[0]; O.m_TwistHalf = V[1]; }
                    if (!Axes.empty())
                    {
                        std::vector<std::string> Words; { std::string W; for (char c : Axes + " ") { if (c == ' ' || c == ',') { if (!W.empty()) Words.push_back(W); W.clear(); } else W += c; } }
                        if (Words.size() != 3 || !AxisWord(Words[0], O.m_SwingY) || !AxisWord(Words[1], O.m_SwingZ) || !AxisWord(Words[2], O.m_Twist)) return "-Axes: \"swing_y swing_z twist\", each Free, Limited or Locked";
                    }
                    if (!Frame.empty())
                    {
                        if (xstrtool::findI(Frame, "sugg") == 0) O.m_bFrameSet = false;
                        else { auto V = Floats(Frame); if (V.size() != 4) return "-Frame: \"x y z w\" or Suggested"; O.m_bFrameSet = true; O.m_Frame = xmath::fquat(V[0], V[1], V[2], V[3]).NormalizeSafeCopy(); }
                    }
                    if (!Relaxed.empty()) { auto V = Floats(Relaxed); if (V.size() != 3) return "-Relaxed: \"swing_y swing_z twist\""; O.m_Relaxed = { V[0], V[1], V[2] }; }
                    if (!Tone.empty()) { auto V = Floats(Tone); if (V.size() != 1 || V[0] < 0) return "-Tone: one number >= 0"; O.m_Tone = V[0]; }
                    if (!Source.empty()) O.m_Source = xstrtool::findI(Source, "fit") == 0 ? xskeleton_desc::joint_source::FITTED : xstrtool::findI(Source, "pre") == 0 ? xskeleton_desc::joint_source::PRESET : xskeleton_desc::joint_source::CUSTOM;
                    // validated before it is accepted (atomic: a bad field changes nothing)
                    if (const char* pBad = xanimation::Validate(xskeleton_joints::LimitFromOverride(O, Rest.m_Rotation[iBone], R.m_Limits[iBone].m_Frame)); pBad) return pBad;
                    if (auto* pOld = D.findJointOverride(BoneName)) *pOld = O; else D.m_JointOverrides.push_back(O);
                    if (bMirror)
                    {
                        const auto R2 = xskeleton_joints::Resolve(D, Names, Parents, Rest);
                        if (auto E = MirrorInto(D, R2, Names, Parents, Rest, iBone, bForce, Note); !E.empty()) return E;
                    }
                    return {};
                });
                if (!Err.empty()) return std::format("JointSet: {}", Err);
                const auto R3 = xskeleton_joints::Resolve(*pDesc, Names, Parents, Rest);
                return std::format("JointSet: {} = {}{}", BoneName, xskeleton_joints::Describe(R3.m_Limits[iBone]), Note);
            }
            case joint_cmd::kind::reset:
            {
                std::vector<std::string_view> Views(Names.begin(), Names.end());
                const int Other = bMirror ? xanimation::FindMirrorBone(Views, iBone) : -1;
                const auto Err = CommitJointEdit(std::format("Joint limit reset {}", BoneName), [&](desc& D) -> std::string
                {
                    auto& V = D.m_JointOverrides;
                    V.erase(std::remove_if(V.begin(), V.end(), [&](const auto& O) { return O.m_BoneName == BoneName || (Other >= 0 && O.m_BoneName == Names[Other]); }), V.end());
                    return {};
                });
                if (!Err.empty()) return std::format("JointReset: {}", Err);
                const auto R3 = xskeleton_joints::Resolve(*pDesc, Names, Parents, Rest);
                return std::format("JointReset: {} = {} ({})", BoneName, xskeleton_joints::Describe(R3.m_Limits[iBone]), R3.m_Source[iBone]);
            }
            case joint_cmd::kind::mirror:
            {
                std::string Note;
                const auto Err = CommitJointEdit(std::format("Joint limit mirror {}", BoneName), [&](desc& D) { return MirrorInto(D, R, Names, Parents, Rest, iBone, bForce, Note); });
                return Err.empty() ? std::format("JointMirror: {} mirrored to the other side{}", BoneName, Note) : std::format("JointMirror: {}", Err);
            }
            case joint_cmd::kind::review:
            {
                std::vector<std::string_view> Views(Names.begin(), Names.end());
                const int Other = bMirror ? xanimation::FindMirrorBone(Views, iBone) : -1;
                std::string By; GetArg(Cmd.m_Parser, Cmd.m_hG, By);
                const bool ByUser = xstrtool::findI(By, "user") == 0;
                const auto Err = CommitJointEdit(std::format("Joint limit reviewed {}", BoneName), [&](desc& D) -> std::string
                {
                    for (int b : { iBone, Other })
                    {
                        if (b < 0) continue;
                        xskeleton_desc::joint_override* pO = D.findJointOverride(Names[b]);
                        if (!pO)    // reviewing a preset joint keeps what was reviewed: its values are copied in
                        {
                            xskeleton_desc::joint_override O; O.m_BoneName = Names[b]; O.m_Source = xskeleton_desc::joint_source::PRESET;
                            xskeleton_joints::SetOverrideFields(O, R.m_Limits[b], false);
                            O.m_Provenance = std::format("{} ({})", xskeleton_joints::ProfileName(D.m_JointProfile), R.m_Source[b]);
                            D.m_JointOverrides.push_back(O); pO = &D.m_JointOverrides.back();
                        }
                        pO->m_Review = xskeleton_desc::joint_review::REVIEWED;
                        pO->m_ReviewedHash = xskeleton_joints::ReviewHash(*pO);
                        const char* pBy = ByUser ? "reviewed in the editor" : "reviewed from the console";
                        if (pO->m_Provenance.find(pBy) == std::string::npos) pO->m_Provenance += pO->m_Provenance.empty() ? std::string(pBy) : std::string("; ") + pBy;
                    }
                    return {};
                });
                return Err.empty() ? std::format("JointReview: {}{} reviewed", BoneName, Other >= 0 ? " and " + Names[Other] : "") : std::format("JointReview: {}", Err);
            }
            }
            return {};
        }

        std::string RunQuery(query_cmd& Cmd) noexcept
        {
            auto* pSkeleton = Skeleton();
            switch (Cmd.m_Kind)
            {
            case query_cmd::kind::list_bones:
            {
                if (!pSkeleton) return "ListBones: no compiled skeleton (compile it first)";
                std::string Text;
                const auto Bones = pSkeleton->getBones();
                for (int i = 0; i < int(Bones.size()); ++i)
                {
                    const auto Raw = RawName(*pSkeleton, i);
                    Text += std::format("{}: {} (raw {}) parent {}{}{}{}", i, EffectiveName(*pSkeleton, i), Raw, int(Bones[i].m_iParent)
                        , Bones[i].m_Flags.m_bVirtual ? " virtual" : "", Bones[i].m_Flags.m_bSocket ? " socket" : "", m_Selected.count(i) ? " selected" : "");
                    Text += std::format(" LOD{}", GetBoneLODLevel(*pSkeleton, i));
                    if (auto* pDesc = Desc()) if (const auto* pOverride = FindBoneOverride(pDesc->m_RootBone, Raw); pOverride && pOverride->m_bDeleteBone) Text += " (marked for deletion)";
                    Text += '\n';
                }
                return Text;
            }
            case query_cmd::kind::select_bones:
            {
                if (!pSkeleton) return "SelectBones: no compiled skeleton";
                std::string Text, Add;
                if (!xeditor::cmd_util::GetArg(Cmd.m_Parser, Cmd.m_hBones, Text)) return "SelectBones: bad arguments";
                const bool bAdd = xeditor::cmd_util::GetArg(Cmd.m_Parser, Cmd.m_hAdd, Add) && Add == "true";
                std::set<int> Found;
                for (auto& Name : SplitLines(Text))
                {
                    int iFound = -1;
                    for (int i = 0; i < int(pSkeleton->getBones().size()) && iFound < 0; ++i) if (RawName(*pSkeleton, i) == Name || DisplayName(*pSkeleton, i) == Name) iFound = i;
                    if (iFound < 0) return std::format("SelectBones: no compiled bone '{}'", Name);
                    Found.insert(iFound);
                }
                if (!bAdd) m_Selected.clear();
                m_Selected.insert(Found.begin(), Found.end());
                m_iAnchor = Found.empty() ? -1 : *Found.begin();
                return std::format("SelectBones: {} selected", m_Selected.size());
            }
            case query_cmd::kind::clear_selection:
                m_Selected.clear(); m_iAnchor = -1;
                return "ClearSelection: done";
            case query_cmd::kind::list_layers:
            {
                auto* pDesc = Desc();
                if (!pDesc) return "ListLayers: nothing loaded";
                std::string Text;
                for (auto& Group : pDesc->m_MaskGroups) Text += std::format("{}: {} bone(s)\n", Group.m_Name, Group.m_Entries.size());
                return Text.empty() ? "(no mask layers)" : Text;
            }
            }
            return {};
        }

        //----------------------------------------------------------------------------------------
        // The bone hierarchy: every imported bone, edited through commands
        //----------------------------------------------------------------------------------------

        static void CenterNextCheckbox() noexcept
        {
            const float Width = ImGui::GetContentRegionAvail().x, Box = ImGui::GetFrameHeight();
            if (Width > Box) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (Width - Box) * 0.5f);
        }

        void RenderBoneNode(const xskeleton::skeleton& Skeleton, const std::unordered_map<std::string, int>& Compiled, const xskeleton_desc::details::bone& Node, int ParentLOD, int iLayer, std::string& Pending) noexcept
        {
            auto& D = *Desc();
            const std::string& Raw = Node.m_Name;
            const auto It = Compiled.find(Raw);
            const int iBone = It == Compiled.end() ? -1 : It->second;
            const auto* pOverride = FindBoneOverride(D.m_RootBone, Raw);
            const std::string Name = pOverride && !pOverride->m_Rename.empty() ? pOverride->m_Rename : StripVBoneTag(Raw);

            const bool bSelected = iBone != -1 && m_Selected.count(iBone), bHovered = iBone != -1 && iBone == m_iHovered, bKids = !Node.m_Children.empty();
            const bool bDelete = pOverride && pOverride->m_bDeleteBone, bVirtual = pOverride && pOverride->m_Type == xskeleton_desc::bone_type::VIRTUAL, bExpose = pOverride && pOverride->m_bExpose;
            const bool bRenaming = m_RenamingBone == Raw;

            ImGui::TableNextRow();
            if (bHovered) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_HeaderHovered));
            ImGui::TableSetColumnIndex(0);

            ImGuiTreeNodeFlags Flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_DefaultOpen;
            if (bSelected) Flags |= ImGuiTreeNodeFlags_Selected;
            if (!bKids)    Flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

            // A source node the compiler folded into the joints below it: real in the file, not in the runtime skeleton
            const bool bFolded = iBone == -1 && !bDelete && !Compiled.empty();
            if (bDelete)       ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 90, 90, 255));
            else if (bVirtual) ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 180, 84, 255));
            else if (bFolded)  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
            std::string Label = Name;
            if (pOverride && pOverride->m_LODLevel > 0) Label += std::format("  [LOD{}]", pOverride->m_LODLevel);
            if (iBone != -1) if (const int Eff = GetBoneLODLevel(Skeleton, iBone); Eff != (pOverride ? pOverride->m_LODLevel : 0)) Label += std::format("  (compiled: LOD{})", Eff);
            if (bDelete) Label += "  (pending delete)";
            if (bFolded) Label += "  (source only - folded at compile)";
            if (m_iScrollTo >= 0 && iBone >= 0 && iBone < int(m_ScrollPath.size()) && m_ScrollPath[iBone]) ImGui::SetNextItemOpen(true);
            const bool bOpen = ImGui::TreeNodeEx(Raw.c_str(), Flags, "%s", bRenaming ? "" : Label.c_str());
            if (iBone >= 0 && iBone == m_iScrollTo) { if (!ImGui::IsItemVisible()) ImGui::SetScrollHereY(0.5f); m_iScrollTo = -1; }
            if (bDelete || bVirtual || bFolded) ImGui::PopStyleColor();

            if (ImGui::IsItemHovered()) { m_iHovered = iBone; m_bTreeHovered = true; }

            // The bone's intent: its icon at the end of the name cell (dimmed while only suggested), spelled out on hover
            {
                const auto OvIntent = pOverride ? pOverride->m_Intent : xskeleton_desc::bone_intent::AUTO;
                const auto Intent   = xskeleton_desc::ResolveIntent(OvIntent, Raw);
                if (Intent != xskeleton::intent::NONE)
                {
                    const bool   bSuggested = OvIntent == xskeleton_desc::bone_intent::AUTO;
                    const float  Size       = ImGui::GetTextLineHeight();
                    const ImVec2 Min(ImGui::GetItemRectMax().x - Size - 4.0f, ImGui::GetItemRectMin().y + (ImGui::GetItemRectSize().y - Size) * 0.5f);
                    xskeleton_editor::DrawIntentIcon(*ImGui::GetWindowDrawList(), Min, Size, Intent, bSuggested);
                    if (ImGui::IsMouseHoveringRect(Min, ImVec2(Min.x + Size, Min.y + Size)))
                        ImGui::SetTooltip("%s%s", xskeleton::intent_help_v[static_cast<int>(Intent)], bSuggested ? "\n(suggested from the name - right click to accept or change)" : "");
                }
            }

            if (bRenaming)
            {
                ImGui::SameLine();
                static char Buffer[128];
                if (m_bRenameStarted) { std::snprintf(Buffer, sizeof(Buffer), "%s", m_RenameBuffer.c_str()); ImGui::SetKeyboardFocusHere(); m_bRenameStarted = false; }
                ImGui::PushID(Raw.c_str());
                ImGui::SetNextItemWidth(-FLT_MIN);
                const bool bEnter = ImGui::InputText("##rename", Buffer, sizeof(Buffer), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
                if (bEnter || ImGui::IsItemDeactivatedAfterEdit())
                {
                    Pending = std::format("SetBone -Bones {} -Rename {}", xeditor::Quote(Raw), xeditor::Quote(std::string(Buffer)));
                    m_RenamingBone.clear();
                }
                else if (ImGui::IsItemDeactivated()) m_RenamingBone.clear();
                ImGui::PopID();
            }
            else
            {
                if (ImGui::IsItemClicked() && iBone != -1) Select(iBone, ImGui::GetIO().KeyCtrl, ImGui::GetIO().KeyShift);
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) { m_RenamingBone = Raw; m_bRenameStarted = true; m_RenameBuffer = Name; }
            }

            ImGui::PushID(Raw.c_str());
            if (ImGui::BeginPopupContextItem("BoneRowContextMenu"))
            {
                if (iBone != -1 && !bSelected) Select(iBone, false, false);
                const auto Targets = EditTargets(Skeleton, iBone, Raw);
                if (ImGui::MenuItem("Rename")) { m_RenamingBone = Raw; m_bRenameStarted = true; m_RenameBuffer = Name; }
                if (ImGui::MenuItem(bVirtual ? "Mark as Normal Bone" : "Mark as Virtual Bone")) Pending = std::format("SetBone -Bones {} -Virtual {}", Targets, bVirtual ? "false" : "true");
                if (ImGui::MenuItem(bDelete  ? "Unmark Deletion"     : "Mark for Deletion"))    Pending = std::format("SetBone -Bones {} -Delete {}",  Targets, bDelete  ? "false" : "true");
                if (ImGui::MenuItem(bExpose  ? "Unexpose Socket"     : "Expose as Socket"))     Pending = std::format("SetBone -Bones {} -Expose {}",  Targets, bExpose  ? "false" : "true");
                if (ImGui::BeginMenu("Intent"))
                {
                    using bi = xskeleton_desc::bone_intent;
                    const auto Cur       = pOverride ? pOverride->m_Intent : bi::AUTO;
                    const auto Suggested = xskeleton::SuggestIntent(Raw);
                    const std::string AutoLabel = std::format("Auto (suggested: {})", xskeleton::intent_names_v[static_cast<int>(Suggested)]);
                    static constexpr std::pair<bi, const char*> Items[] { {bi::NONE, "none"}, {bi::MOTION, "motion"}, {bi::FACING, "facing"}, {bi::LOOK_AT, "lookat"}, {bi::IK_TARGET, "iktarget"} };
                    if (ImGui::MenuItem(AutoLabel.c_str(), nullptr, Cur == bi::AUTO)) Pending = std::format("SetBone -Bones {} -Intent auto", Targets);
                    ImGui::Separator();
                    for (auto& [Value, pArg] : Items)
                    {
                        const auto Resolved = xskeleton_desc::ResolveIntent(Value, Raw);
                        if (ImGui::MenuItem(xskeleton::intent_names_v[static_cast<int>(Resolved)], nullptr, Cur == Value)) Pending = std::format("SetBone -Bones {} -Intent {}", Targets, pArg);
                        if (ImGui::IsItemHovered() && Resolved != xskeleton::intent::NONE) ImGui::SetTooltip("%s", xskeleton::intent_help_v[static_cast<int>(Resolved)]);
                    }
                    ImGui::EndMenu();
                }
                if (ImGui::MenuItem("Match Children's LOD to This Bone")) Pending = std::format("MatchLOD -Bones {}", Targets);
                if (iLayer >= 0 && ImGui::MenuItem("Match Children's Active Layer Weight to This Bone"))
                    Pending = std::format("SetMaskWeight -Layer {} -Bones {} -Weight {} -Children true", xeditor::Quote(D.m_MaskGroups[iLayer].m_Name), Targets, GetMaskWeight(D, iLayer, Raw));
                ImGui::EndPopup();
            }
            ImGui::PopID();

            auto Check = [&](int Column, const char* pId, bool bValue, const char* pCommand, const char* pOption)
            {
                ImGui::TableSetColumnIndex(Column);
                ImGui::PushID(Raw.c_str());
                CenterNextCheckbox();
                bool V = bValue;
                if (ImGui::Checkbox(pId, &V)) Pending = std::format("SetBone -Bones {} -{} {}", EditTargets(Skeleton, iBone, Raw), pOption, V ? "true" : "false");
                ImGui::PopID();
                (void)pCommand;
            };
            Check(1, "##expose",  bExpose,  "SetBone", "Expose");
            Check(2, "##virtual", bVirtual, "SetBone", "Virtual");
            Check(3, "##delete",  bDelete,  "SetBone", "Delete");

            int Effective = pOverride ? pOverride->m_LODLevel : 0;
            ImGui::TableSetColumnIndex(4);
            {
                int Cur = Effective;
                ImGui::PushID(Raw.c_str());
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::InputInt("##lod", &Cur, 1, 1))
                {
                    Cur = std::max(ParentLOD, Cur);         // a value below the parent's is what the compiler overrides
                    Pending = std::format("SetBone -Bones {} -LOD {}", EditTargets(Skeleton, iBone, Raw), Cur);
                    Effective = Cur;
                }
                ImGui::PopID();
            }
            if (iLayer >= 0)
            {
                ImGui::TableSetColumnIndex(5);
                float Weight = GetMaskWeight(D, iLayer, Raw);
                ImGui::PushID(Raw.c_str());
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::InputFloat("##mask", &Weight, 0.0f, 0.0f, "%.2f"))
                    Pending = std::format("SetMaskWeight -Layer {} -Bones {} -Weight {}", xeditor::Quote(D.m_MaskGroups[iLayer].m_Name), EditTargets(Skeleton, iBone, Raw), std::clamp(Weight, 0.0f, 1.0f));
                ImGui::PopID();
            }

            if (bKids && bOpen)
            {
                for (auto& Child : Node.m_Children) RenderBoneNode(Skeleton, Compiled, Child, std::max(ParentLOD, Effective), iLayer, Pending);
                ImGui::TreePop();
            }
        }

        void RenderHierarchy() noexcept
        {
            auto* pDesc = Desc();
            auto* pSkeleton = Skeleton();
            if (!pDesc) return;
            if (!pSkeleton) { ImGui::TextWrapped("%s", m_ErrorMessage.empty() ? "Nothing to show." : m_ErrorMessage.c_str()); return; }
            if (m_Details.m_RootBone.m_Name.empty()) { ImGui::TextDisabled("No bone information yet: compile the skeleton."); return; }

            ImGui::Text("%d bones", int(pSkeleton->getBones().size()));
            const int iLayer = pDesc->findMaskGroup(m_Settings.m_ActiveLayer);
            if (ImGui::BeginCombo("Layer", m_Settings.m_ActiveLayer.empty() ? "None" : m_Settings.m_ActiveLayer.c_str()))
            {
                if (ImGui::Selectable("None", m_Settings.m_ActiveLayer.empty())) m_Settings.m_ActiveLayer.clear();
                for (auto& G : pDesc->m_MaskGroups) if (ImGui::Selectable(G.m_Name.c_str(), G.m_Name == m_Settings.m_ActiveLayer)) m_Settings.m_ActiveLayer = G.m_Name;
                ImGui::EndCombo();
            }
            ImGui::Separator();

            std::unordered_map<std::string, int> Compiled;
            for (int i = 0; i < int(pSkeleton->getBones().size()); ++i) Compiled[RawName(*pSkeleton, i)] = i;

            std::string Pending;
            // A bone hovered or newly selected in the 3D view (not in the tree itself) is brought into the tree's view
            if (pSkeleton)
            {
                int iTarget = -1;
                if (!m_bTreeHovered && m_iHovered >= 0 && m_iHovered != m_iLastHover3D) iTarget = m_iHovered;
                if (m_Selected != m_LastSelected && !m_Selected.empty() && !m_bTreeHovered) iTarget = *m_Selected.rbegin();
                m_iLastHover3D = m_bTreeHovered ? -1 : m_iHovered;
                m_LastSelected = m_Selected;
                if (iTarget >= 0 && iTarget < int(pSkeleton->getBones().size()))
                {
                    m_iScrollTo = iTarget;
                    m_ScrollPath.assign(pSkeleton->getBones().size(), false);
                    for (int p = pSkeleton->getBones()[iTarget].m_iParent; p >= 0; p = pSkeleton->getBones()[p].m_iParent) m_ScrollPath[p] = true;
                }
            }

            m_bTreeHovered = false;
            const bool bHovering = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
            const int  Hovered = m_iHovered;
            if (ImGui::BeginTable("##Bones", iLayer >= 0 ? 6 : 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV))
            {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Bone", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Sock", ImGuiTableColumnFlags_WidthFixed, 34.0f);
                ImGui::TableSetupColumn("Virt", ImGuiTableColumnFlags_WidthFixed, 34.0f);
                ImGui::TableSetupColumn("Del",  ImGuiTableColumnFlags_WidthFixed, 34.0f);
                ImGui::TableSetupColumn("LOD",  ImGuiTableColumnFlags_WidthFixed, 80.0f);
                if (iLayer >= 0) ImGui::TableSetupColumn("Weight", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                ImGui::TableHeadersRow();
                RenderBoneNode(*pSkeleton, Compiled, m_Details.m_RootBone, 0, iLayer, Pending);
                ImGui::EndTable();
            }
            (void)bHovering; (void)Hovered;

            if (!Pending.empty()) xeditor::Run(m_Undo, Pending);
        }

        //----------------------------------------------------------------------------------------
        // The 3D view
        //----------------------------------------------------------------------------------------

        void RenderViewport() noexcept
        {
            m_CameraToolbar.Render(ImGui::GetContentRegionAvail(), m_Scene.m_State, [this] { RenderViewportBody(); }
                                  , [this](bool bHorizontal, float ButtonHeight)
                                    {
                                        xeditor::scale_reference::Button(bHorizontal, ButtonHeight);
                                        xeditor::transform_gizmo::Buttons(m_Gizmo, bHorizontal, ButtonHeight);
                                    });
        }

        void RenderViewportBody() noexcept
        {
            auto* pHost   = xeditor::host::current();
            auto* pWindow = pHost ? pHost->find<xgpu::window>() : nullptr;
            const ImVec2 Avail = ImGui::GetContentRegionAvail();
            const ImVec2 Min   = ImGui::GetCursorScreenPos();
            xeditor::transform_gizmo::layer GizmoLayer(m_Gizmo, *ImGui::GetWindowDrawList());     // the gizmo over all of it
            ImGui::GetWindowDrawList()->AddRectFilled(Min, ImVec2(Min.x + Avail.x, Min.y + Avail.y), IM_COL32(115, 115, 115, 255));

            // The gizmo (W E R) on the descriptor's PreTransform, before the view's button: while it is hovered a Dummy stands in for the button
            // (the camera and the keys still work, a click is the gizmo's, not a pick), and only its drag keeps the camera out
            bool bGizmo = false;
            if (auto* pDesc = Desc(); pDesc && m_bReady && pWindow && Skeleton())
            {
                const float S = ViewScale();
                bGizmo = xeditor::transform_gizmo::Run(m_Gizmo, *this, GizmoLayer, m_Scene.m_View, Min, Avail, xeditor::transform_gizmo::TargetOf(pDesc->m_PreTransform)
                                                     , xmath::fmat4::fromScale(xmath::fvec3(S, S, S)));
            }
            if (bGizmo) ImGui::Dummy(Avail);
            else        ImGui::InvisibleButton("##SkeletonViewport", Avail, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            const bool bHovered = ImGui::IsItemHovered() && !bGizmo;
            if (!xeditor::transform_gizmo::Dragging() && !m_bPulling && !(m_bJointsMode && (m_bPullHover || m_JHandleHover >= 0 || m_JDrag >= 0)))
            {
                xeditor::PreviewGestures();
                m_Scene.HandleInput();
            }

            auto* pSkeleton = Skeleton();
            if (!m_bReady || !pWindow) { ImGui::SetCursorScreenPos(Min); ImGui::TextDisabled("The 3D view needs a GPU device (open from the editor)."); return; }
            if (!pSkeleton) { ImGui::SetCursorScreenPos(Min); ImGui::TextWrapped("%s", m_ErrorMessage.empty() ? "Nothing to show." : m_ErrorMessage.c_str()); return; }

            // "Resize skeleton to 1m" scales the view only, never the asset
            if (m_Settings.m_bNormalizeSize != m_bLastNormalize) { m_bLastNormalize = m_Settings.m_bNormalizeSize; m_Scene.m_bReframe = true; }
            const float Scale = ViewScale();
            m_Scene.m_Radius = m_Radius * Scale;
            m_Scene.m_Center = m_Center * Scale;
            const auto& Src = m_Settings.m_Pose == pose_mode::BIND ? m_WorldBind : m_WorldFrozen;
            m_Scaled = Src;
            // The PreTransform being edited (the gizmo, the inspector), before it is compiled: the compiled bones moved from the one they were compiled with
            if (auto* pDesc = Desc())
            {
                // (the pose moves from the corrections it was compiled with to the ones it has now)
                namespace tg = xeditor::transform_gizmo;
                // (the bind pose is moved by the PreTransform alone; the frozen pose by both: the PostTransform is for the frozen pose and the animations)
                const bool  bBind = m_Settings.m_Pose == pose_mode::BIND;
                const auto  Same = m_Compiled == tg::PreOf(pDesc->m_PreTransform) && (bBind || m_CompiledPost == tg::PreOf(pDesc->m_PostTransform));
                const auto  Was  = tg::Matrix(m_Compiled) * (bBind ? xmath::fmat4::fromIdentity() : tg::Matrix(m_CompiledPost));
                const auto  Now  = tg::Matrix(tg::PreOf(pDesc->m_PreTransform)) * (bBind ? xmath::fmat4::fromIdentity() : tg::Matrix(tg::PreOf(pDesc->m_PostTransform)));
                const xmath::fmat4 Pending = Same ? xmath::fmat4::fromIdentity() : Now * Was.Inverse();
                if (!Same)
                    for (auto& W : m_Scaled)
                    {
                        const xmath::fvec3 P = Pending * W.m_Position;
                        const auto Dir = [&](const xmath::fvec3& V) { const xmath::fvec3 D = Pending * (W.m_Position + V) - P; return D.Length() > 1.0e-12f ? D.NormalizeCopy() * V.Length() : V; };
                        W.m_Right    = Dir(W.m_Right);
                        W.m_Up       = Dir(W.m_Up);
                        W.m_Position = P;
                    }
            }
            for (auto& W : m_Scaled) W.m_Position = W.m_Position * Scale;
            if (m_bJointsMode) ApplyJointPose(Scale);
            m_Scene.UpdateView(Min, Avail.x, Avail.y);

            // Hover and click: a ray from the mouse against the bones' solid shapes. Matches the original
            // E23 example exactly: RAW mouse position (not pre-subtracted by Min), against a view whose
            // viewport is the panel's own ABSOLUTE screen rect (set in UpdateView above) - RayFromScreen
            // uses Viewport.Min/Max internally to find its own center, so it needs the SAME coordinate
            // space the mouse position is already in.
            const ImVec2 Mouse = ImGui::GetIO().MousePos;
            const bool bOrbiting = ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle);
            if (bHovered && !bOrbiting)
            {
                const auto Dir = m_Scene.m_View.RayFromScreen(Mouse.x, Mouse.y);
                float T = 0;
                int iHit = -1;
                PickWedge(*pSkeleton, m_Scaled, m_Scene.m_View.getPosition(), Dir, m_Scene.m_Radius, iHit, T);
                m_iHovered = iHit;
                if (m_bJointsMode && m_JHandleHover >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) StartHandleDrag(Mouse);   // a limit handle wins
                else if (m_bJointsMode && m_bPullHover && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) StartPull();     // the handle wins over a pick
                else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                {
                    if (iHit != -1) Select(iHit, ImGui::GetIO().KeyCtrl, ImGui::GetIO().KeyShift);
                    else            ClearSelectionUnlessModifier(ImGui::GetIO().KeyCtrl, ImGui::GetIO().KeyShift);
                }
            }
            else if (!m_bTreeHovered) m_iHovered = -1;

            auto Style = m_Scene.Style();
            const int iLayer = m_Settings.m_ColorMode == render_color_mode::ACTIVE_LAYER_RENDER && !m_Settings.m_ActiveLayer.empty() ? pSkeleton->findMaskGroupIndex(xstrtool::CRC32(m_Settings.m_ActiveLayer)) : -1;
            BuildWedgeGeometry    (*pSkeleton, m_Scaled, m_IsTwist, Style, m_Selected, m_Scene.m_Radius, m_Settings.m_ColorMode, iLayer, m_iHovered, m_Outline);
            BuildWedgeFillGeometry(*pSkeleton, m_Scaled, m_IsTwist, Style, m_Selected, m_Scene.m_Radius, m_Settings.m_ColorMode, iLayer, m_iHovered, m_Fill);
            {   // the intents as the descriptor has them now (no compile needed to see a change)
                const auto nBones = pSkeleton->getBones().size();
                std::vector<std::uint8_t> Intents(nBones, 0), Suggested(nBones, 0);
                if (auto* pDesc = Desc())
                    for (std::size_t i = 0; i < nBones; ++i)
                    {
                        const auto  Raw = RawName(*pSkeleton, static_cast<int>(i));
                        const auto* pOv = FindBoneOverride(pDesc->m_RootBone, Raw);
                        const auto  Ov  = pOv ? pOv->m_Intent : xskeleton_desc::bone_intent::AUTO;
                        Intents[i]   = static_cast<std::uint8_t>(xskeleton_desc::ResolveIntent(Ov, Raw));
                        Suggested[i] = Ov == xskeleton_desc::bone_intent::AUTO;
                    }
                BuildIntentGlyphFills(m_Scaled, Intents, Suggested, Style, m_Selected, m_iHovered, m_Scene.m_Radius, m_Fill);
                BuildIntentGlyphs(m_Scaled, Intents, Suggested, Style, m_Selected, m_iHovered, m_Scene.m_Radius, m_Outline);
            }
            m_Scene.SetLines(m_Outline);
            m_Scene.SetFill(m_Fill);
            m_Scene.RenderShadow(*pWindow);

            xgpu::tools::imgui::AddCustomRenderCallback([this](xgpu::cmd_buffer& CmdBuffer, const ImVec2&, const ImVec2&)
            {
                if (m_bOpen) m_Scene.DrawBones(CmdBuffer);
            });
            if (!m_Scaled.empty())
            {   // the human scale reference: 1.8 m beside the skeleton as drawn (the PreTransform being edited), scaled with the view
                xmath::fvec3 BoxMin = m_Scaled[0].m_Position, BoxMax = BoxMin;
                for (const auto& W : m_Scaled) { BoxMin = BoxMin.Min(W.m_Position); BoxMax = BoxMax.Max(W.m_Position); }
                xeditor::scale_reference::DrawBesideBox(*ImGui::GetWindowDrawList(), m_Scene.m_View.getW2C(), m_Scene.m_View.getPosition(), Min, Avail, BoxMin, BoxMax, Scale);
            }
            m_Scene.DrawOverlay();

            RenderLabels(*pSkeleton, Min, Avail, Mouse);
            if (m_bJointsMode) DrawJointOverlay(Min, Avail, Scale, Mouse);
        }

        //----------------------------------------------------------------------------------------
        // Joints mode (J2, thin slice): the selected joint's allowed range drawn on the bone, a test pull (body_ik inside the limits) in a
        // sandbox pose that is never saved, and the Joints panel whose edits go through the Joint* commands only.
        //----------------------------------------------------------------------------------------

        int SelectedJoint() const noexcept { return m_Selected.size() == 1 ? *m_Selected.begin() : -1; }

        // the resolution, redone when the joint section or the skeleton changed
        bool UpdateJoints() noexcept
        {
            auto* pDesc = Desc(); auto* pSkeleton = Skeleton();
            if (!pDesc || !pSkeleton) return false;
            const auto Key = std::format("{}|{}", static_cast<const void*>(pSkeleton), xskeleton_joints::JointState(*pDesc));
            if (Key == m_JointKey) return !m_JNames.empty();
            m_JointKey = Key;
            if (!JointRig(m_JNames, m_JParents, m_JRest)) return false;
            m_JointRes = xskeleton_joints::Resolve(*pDesc, m_JNames, m_JParents, m_JRest);
            m_Ik.Setup(m_JParents, m_JointRes.m_Limits);
            if (m_bTestPose && m_TestPose.count() != m_JRest.count()) m_bTestPose = false;
            return true;
        }

        // the sandbox pose replaces the frozen pose in the view (positions moved by their model-space change, axes turned)
        void ApplyJointPose(float Scale) noexcept
        {
            if (!UpdateJoints() || m_Scaled.size() != m_JNames.size()) return;
            std::vector<xmath::fquat> Rr; std::vector<xmath::fvec3> Pr;
            xanimation::details::WorldRP(xanimation::skeleton_view{ m_JParents }, m_JRest, Rr, Pr);
            m_FrozenView.resize(m_Scaled.size());
            for (std::size_t i = 0; i < m_Scaled.size(); ++i) m_FrozenView[i] = m_Scaled[i].m_Position;
            if (!m_bTestPose) { m_CurR = Rr; return; }
            std::vector<xmath::fvec3> Pt;
            xanimation::details::WorldRP(xanimation::skeleton_view{ m_JParents }, m_TestPose, m_CurR, Pt);
            for (std::size_t i = 0; i < m_Scaled.size(); ++i)
            {
                const auto D = (m_CurR[i] * Rr[i].ConjugateCopy()).NormalizeCopy();
                m_Scaled[i].m_Position = m_Scaled[i].m_Position + (Pt[i] - Pr[i]) * Scale;
                m_Scaled[i].m_Right = D * m_Scaled[i].m_Right;
                m_Scaled[i].m_Up    = D * m_Scaled[i].m_Up;
            }
        }

        bool ProjectView(const xmath::fvec3& P, const ImVec2& Min, const ImVec2& Size, ImVec2& Out) const noexcept
        {
            const xmath::fvec4 Clip = m_Scene.m_View.getW2C() * xmath::fvec4(P, 1.0f);
            if (Clip.m_W <= 1.0e-4f) return false;
            Out = ImVec2(Min.x + (Clip.m_X / Clip.m_W * 0.5f + 0.5f) * Size.x, Min.y + (Clip.m_Y / Clip.m_W * 0.5f + 0.5f) * Size.y);
            return true;
        }

        int FirstChild(int b) const noexcept { for (int c = b + 1; c < int(m_JParents.size()); ++c) if (m_JParents[c] == b) return c; return -1; }

        void StartPull() noexcept
        {
            const int b = SelectedJoint();
            if (b < 0 || m_PullEff < 0) return;
            m_bPulling  = true;
            m_PullBone  = b;
            // Include this pose: only that joint's limit is lifted, every other joint keeps its own
            if (m_bJInclude && m_JIncludeBone >= 0 && m_JIncludeBone < int(m_JointRes.m_Limits.size()))
            {
                auto Lim = m_JointRes.m_Limits;
                auto& J  = Lim[m_JIncludeBone];
                if (J.m_Type == xanimation::joint_type::HINGE && !m_bJIncludeBall) { J.m_HingeMin = -3.14159265f; J.m_HingeMax = 3.14159265f; }  // still a hinge: the pose explores its axis
                else J = xanimation::joint_limit{};                                                                                            // a ball (or "make it a ball"): free
                m_Ik.Setup(m_JParents, Lim);
            }
            else m_Ik.Setup(m_JParents, m_JointRes.m_Limits);
            m_PullBase  = m_bTestPose ? m_TestPose : m_JRest;     // a pull solves from the pose it started on (the result does not depend on how often the mouse moved)
            m_PullPlane = m_PullTargetView = m_Scaled[m_PullEff].m_Position;
        }

        // why a pull stopped short, in words
        std::string PullReason(const xanimation::ik_task_report& R, const std::vector<std::uint8_t>& W) const
        {
            using namespace xanimation;
            if (R.m_Status == ik_status::REACHED) return "reached";
            if (R.m_Reasons & ik_reason::LIMIT)
            {
                std::string S;      // every joint of the chain that is on a limit, nearest the end first
                for (int b = int(W.size()) - 1; b >= 0; --b)
                {
                    if (!W[b]) continue;
                    const auto E = Evaluate(m_JointRes.m_Limits[b], m_TestPose.m_Rotation[b], 2e-3f);
                    if (!E.m_Faces) continue;
                    const char* pFace = (E.m_Faces & joint_face::HINGE_HIGH) ? "hinge upper limit" : (E.m_Faces & joint_face::HINGE_LOW) ? "hinge lower limit"
                                      : (E.m_Faces & joint_face::SWING) ? "swing limit" : (E.m_Faces & joint_face::TWIST) ? "twist limit" : "limit";
                    S += std::format("{}{} at its {}", S.empty() ? "" : ", ", m_JNames[b], pFace);
                }
                return S.empty() ? "a joint limit" : S;
            }
            if (R.m_Reasons & ik_reason::REACH)  return "beyond reach of the chain";
            if (R.m_Reasons & ik_reason::LOCKED) return "the chain is 3 bones: the rest of the body is held";
            if (R.m_Reasons & ik_reason::BUDGET) return "the solver ran out of iterations";
            return "missed";
        }

        //----------------------------------------------------------------------------------------
        // J3: the limit's own handles (drag the cone's edges and centre, the twist and hinge arcs' ends), Include this pose, the joint
        // frame, the guided check, display filters. Every edit is one JointSet (one undo step, console parity); Esc cancels a drag.
        //----------------------------------------------------------------------------------------
        enum jh : int { JH_RY_P, JH_RY_N, JH_RZ_P, JH_RZ_N, JH_CENTRE, JH_TW_LO, JH_TW_HI, JH_H_MIN, JH_H_MAX, JH_COUNT };
        static const char* JHName(int k) noexcept
        {
            static constexpr const char* N[] = { "swing y radius", "swing y radius", "swing z radius", "swing z radius", "swing centre", "twist low end", "twist high end", "hinge min", "hinge max" };
            return k >= 0 && k < JH_COUNT ? N[k] : "?";
        }
        static const char* JHTag(int k) noexcept
        {
            static constexpr const char* N[] = { "RY+", "RY-", "RZ+", "RZ-", "C", "TW-", "TW+", "H-", "H+" };
            return k >= 0 && k < JH_COUNT ? N[k] : "?";
        }

        struct joint_draw_ctx { xmath::fvec3 P0, C, Dir, Ref; xmath::fquat Ap = xmath::fquat::fromIdentity(); float Len = 0; };

        joint_draw_ctx JointCtx(int b, const xanimation::joint_limit& L, const xmath::fquat& Local) const noexcept
        {
            using namespace xanimation;
            joint_draw_ctx K;
            const int c = FirstChild(b);
            K.P0  = m_Scaled[b].m_Position;
            K.Len = std::max(c >= 0 ? (m_Scaled[c].m_Position - K.P0).Length() : 0.0f, 0.03f * m_Scene.m_Radius);
            const auto ParentR = m_JParents[b] >= 0 ? m_CurR[m_JParents[b]] : xmath::fquat::fromIdentity();
            K.Ap = (ParentR * L.m_Rest * L.m_Frame).NormalizeCopy();
            float Sy = 0, Sz = 0, T = 0; bool Sing = false;
            if (L.m_Type == joint_type::BALL) details::SplitBall(JointRotation(L, Local), Sy, Sz, T, Sing);
            const auto Sw = K.Ap * details::ExpV(xmath::fvec3(0, Sy, Sz));
            K.Dir = Sw * xmath::fvec3(1, 0, 0); K.Ref = Sw * xmath::fvec3(0, 1, 0); K.C = K.P0 + K.Dir * (K.Len * 0.6f);
            return K;
        }
        static xmath::fvec3 ConePoint(const joint_draw_ctx& K, float Sy, float Sz) noexcept { return K.P0 + (K.Ap * (xanimation::details::MakeBall(Sy, Sz, 0) * xmath::fvec3(1, 0, 0))) * (K.Len * 0.9f); }
        static xmath::fvec3 TwistPoint(const joint_draw_ctx& K, float a, float r = 0.18f) noexcept { return K.C + (xanimation::details::AxisAngle(K.Dir, a) * K.Ref) * (K.Len * r); }
        static xmath::fvec3 HingePoint(const joint_draw_ctx& K, float a) noexcept { return K.P0 + (K.Ap * (xanimation::details::AxisAngle(xmath::fvec3(0, 0, 1), a) * xmath::fvec3(1, 0, 0))) * (K.Len * 0.9f); }

        static bool HandleUsed(const xanimation::joint_limit& L, int k) noexcept
        {
            using namespace xanimation;
            if (L.m_Type == joint_type::HINGE) return k == JH_H_MIN || k == JH_H_MAX;
            if (L.m_Type != joint_type::BALL) return false;
            if (k == JH_RY_P || k == JH_RY_N) return L.m_SwingY == axis_motion::LIMITED;
            if (k == JH_RZ_P || k == JH_RZ_N) return L.m_SwingZ == axis_motion::LIMITED;
            if (k == JH_CENTRE) return L.m_SwingY != axis_motion::FREE || L.m_SwingZ != axis_motion::FREE;
            if (k == JH_TW_LO || k == JH_TW_HI) return L.m_Twist == axis_motion::LIMITED && L.m_TwistHalf < 3.14f;
            return false;
        }
        static xmath::fvec3 HandlePoint(const joint_draw_ctx& K, const xanimation::joint_limit& L, int k) noexcept
        {
            switch (k)
            {
            case JH_RY_P:   return ConePoint(K, L.m_SwingCy + L.m_SwingRy, L.m_SwingCz);
            case JH_RY_N:   return ConePoint(K, L.m_SwingCy - L.m_SwingRy, L.m_SwingCz);
            case JH_RZ_P:   return ConePoint(K, L.m_SwingCy, L.m_SwingCz + L.m_SwingRz);
            case JH_RZ_N:   return ConePoint(K, L.m_SwingCy, L.m_SwingCz - L.m_SwingRz);
            case JH_CENTRE: return ConePoint(K, L.m_SwingCy, L.m_SwingCz);
            case JH_TW_LO:  return TwistPoint(K, L.m_TwistCentre - L.m_TwistHalf, 0.22f);
            case JH_TW_HI:  return TwistPoint(K, L.m_TwistCentre + L.m_TwistHalf, 0.22f);
            case JH_H_MIN:  return HingePoint(K, L.m_HingeMin);
            default:        return HingePoint(K, L.m_HingeMax);
            }
        }
        // a handle's parameters (radians, or a scale of both radii with Shift)
        static int HandleParams(const xanimation::joint_limit& L, int k, bool bBoth, float p[2]) noexcept
        {
            switch (k)
            {
            case JH_RY_P: case JH_RY_N: p[0] = bBoth ? 1.0f : L.m_SwingRy; return 1;
            case JH_RZ_P: case JH_RZ_N: p[0] = bBoth ? 1.0f : L.m_SwingRz; return 1;
            case JH_CENTRE: p[0] = L.m_SwingCy; p[1] = L.m_SwingCz; return 2;
            case JH_TW_LO:  p[0] = L.m_TwistCentre - L.m_TwistHalf; return 1;
            case JH_TW_HI:  p[0] = L.m_TwistCentre + L.m_TwistHalf; return 1;
            case JH_H_MIN:  p[0] = L.m_HingeMin; return 1;
            default:        p[0] = L.m_HingeMax; return 1;
            }
        }
        static xanimation::joint_limit WithParams(const xanimation::joint_limit& L0, int k, bool bBoth, const float p[2]) noexcept
        {
            constexpr float Pi = 3.14159265f, MinR = 0.5f * Pi / 180.0f, MaxR = 179.0f * Pi / 180.0f;
            auto L = L0;
            switch (k)
            {
            case JH_RY_P: case JH_RY_N: case JH_RZ_P: case JH_RZ_N:
                if (bBoth) { const float s = std::max(p[0], 0.01f); L.m_SwingRy = std::clamp(L0.m_SwingRy * s, MinR, MaxR); L.m_SwingRz = std::clamp(L0.m_SwingRz * s, MinR, MaxR); }
                else if (k <= JH_RY_N) L.m_SwingRy = std::clamp(p[0], MinR, MaxR);
                else                   L.m_SwingRz = std::clamp(p[0], MinR, MaxR);
                break;
            case JH_CENTRE: L.m_SwingCy = std::clamp(p[0], -Pi * 0.9f, Pi * 0.9f); L.m_SwingCz = std::clamp(p[1], -Pi * 0.9f, Pi * 0.9f); break;
            case JH_TW_LO: case JH_TW_HI:
            {
                float Lo = L0.m_TwistCentre - L0.m_TwistHalf, Hi = L0.m_TwistCentre + L0.m_TwistHalf;
                if (k == JH_TW_LO) Lo = std::min(p[0], Hi); else Hi = std::max(p[0], Lo);
                L.m_TwistCentre = 0.5f * (Lo + Hi); L.m_TwistHalf = std::min(Pi, 0.5f * (Hi - Lo));
                break;
            }
            case JH_H_MIN: L.m_HingeMin = std::clamp(p[0], -Pi, L0.m_HingeMax); break;
            default:       L.m_HingeMax = std::clamp(p[0], L0.m_HingeMin, Pi); break;
            }
            return L;
        }

        void StartHandleDrag(const ImVec2& Mouse) noexcept
        {
            const int b = SelectedJoint();
            if (b < 0 || m_JHandleHover < 0 || b >= int(m_JointRes.m_Limits.size())) return;
            for (const auto& H : m_JHandles) if (H.m_Kind == m_JHandleHover) m_JDragHandle0 = H.m_Screen;
            m_JDrag = m_JHandleHover; m_JDragShift = ImGui::GetIO().KeyShift;
            m_JDragStart = m_JPreview = m_JointRes.m_Limits[b];
            m_JDragMouse0 = Mouse;
        }

        // the handle follows the mouse: its parameters solved so the handle's screen point lands under the mouse (Gauss-Newton from the drag's start)
        void DragHandle(const joint_draw_ctx& K, const ImVec2& Min, const ImVec2& Size, const ImVec2& Mouse) noexcept
        {
            const bool bBoth = m_JDragShift && m_JDrag <= JH_RZ_N;
            float p[2] = {}, h0[2] = {};
            const int n = HandleParams(m_JDragStart, m_JDrag, bBoth, p);
            h0[0] = p[0]; h0[1] = p[1];
            const ImVec2 Target(m_JDragHandle0.x + Mouse.x - m_JDragMouse0.x, m_JDragHandle0.y + Mouse.y - m_JDragMouse0.y);
            auto Screen = [&](const float q[2], ImVec2& S) { return ProjectView(HandlePoint(K, WithParams(m_JDragStart, m_JDrag, bBoth, q), m_JDrag), Min, Size, S); };
            for (int It = 0; It < 8; ++It)
            {
                ImVec2 S; if (!Screen(p, S)) break;
                const float ex = Target.x - S.x, ey = Target.y - S.y;
                if (ex * ex + ey * ey < 0.25f) break;
                float J[2][2] = {};
                for (int d = 0; d < n; ++d)
                {
                    float q[2] = { p[0], p[1] }; constexpr float h = 1e-3f; q[d] += h;
                    ImVec2 S2; if (!Screen(q, S2)) return;
                    J[0][d] = (S2.x - S.x) / h; J[1][d] = (S2.y - S.y) / h;
                }
                if (n == 1)
                {
                    const float jj = J[0][0] * J[0][0] + J[1][0] * J[1][0];
                    if (jj < 1e-6f) break;
                    p[0] += (J[0][0] * ex + J[1][0] * ey) / (jj + 1.0f);
                }
                else
                {
                    const float a = J[0][0], bb = J[0][1], c = J[1][0], d = J[1][1];
                    const float A11 = a * a + c * c + 1.0f, A12 = a * bb + c * d, A22 = bb * bb + d * d + 1.0f, g1 = a * ex + c * ey, g2 = bb * ex + d * ey;
                    const float Det = A11 * A22 - A12 * A12;
                    if (std::abs(Det) < 1e-9f) break;
                    p[0] += (A22 * g1 - A12 * g2) / Det; p[1] += (A11 * g2 - A12 * g1) / Det;
                }
                if (!bBoth) HandleParams(WithParams(m_JDragStart, m_JDrag, false, p), m_JDrag, false, p);     // keep inside the clamps
            }
            m_JPreview = WithParams(m_JDragStart, m_JDrag, bBoth, p);
            float q0[2], q1[2];
            HandleParams(m_JDragStart, m_JDrag, false, q0); HandleParams(m_JPreview, m_JDrag, false, q1);
            constexpr float D = 57.29578f;
            m_JDragTip = n == 2 ? std::format("{}: {:.1f}, {:.1f} deg (delta {:+.1f}, {:+.1f})", JHName(m_JDrag), q1[0] * D, q1[1] * D, (q1[0] - q0[0]) * D, (q1[1] - q0[1]) * D)
                       : bBoth ? std::format("both swing radii: {:.1f}, {:.1f} deg (x{:.2f})", m_JPreview.m_SwingRy * D, m_JPreview.m_SwingRz * D, p[0])
                       : std::format("{}: {:.1f} deg (delta {:+.1f})", JHName(m_JDrag), q1[0] * D, (q1[0] - q0[0]) * D);
            (void)h0;
        }

        // the JointSet arguments that reproduce a limit's ranges (degrees)
        static std::string JointSetArgs(const xanimation::joint_limit& L)
        {
            constexpr float D = 57.29578f;
            using namespace xanimation;
            if (L.m_Type == joint_type::HINGE) return std::format("-Geometry Hinge -Hinge \"{} {}\"", L.m_HingeMin * D, L.m_HingeMax * D);
            if (L.m_Type == joint_type::BALL)
            {
                static constexpr const char* Ax[] = { "Free", "Limited", "Locked" };
                return std::format("-Geometry Ball -Swing \"{} {} {} {}\" -Twist \"{} {}\" -Axes \"{} {} {}\"", L.m_SwingCy * D, L.m_SwingCz * D, L.m_SwingRy * D, L.m_SwingRz * D
                                  , L.m_TwistCentre * D, L.m_TwistHalf * D, Ax[int(L.m_SwingY)], Ax[int(L.m_SwingZ)], Ax[int(L.m_Twist)]);
            }
            return L.m_Type == joint_type::FIXED ? "-Geometry Fixed" : "-Geometry Free";
        }

        // one JointSet for the joint (with its mirror when symmetry is on); a reviewed, different other side asks first
        std::string CommitJointLimit(const std::string& Name, const std::string& Args, bool bSym) noexcept
        {
            const auto R = JointCommand(std::format("JointSet -Bone {} {}{}", xeditor::Quote(Name), Args, bSym ? " -Mirror 1" : ""));
            if (R.find("asymmetric pair") != std::string::npos)
            {
                m_bJAsymPopup = true; m_JAsymMsg = R;
                m_JAsymName = std::format("JointSet -Bone {} {} -Mirror 1 -Force 1", xeditor::Quote(Name), Args);       // overwrite the other side
                m_JAsymArgs = std::format("JointSet -Bone {} {}", xeditor::Quote(Name), Args);                          // this side only
            }
            return R;
        }
        std::string MirrorJoint(const std::string& Name) noexcept
        {
            const auto R = JointCommand(std::format("JointMirror -Bone {}", xeditor::Quote(Name)));
            if (R.find("asymmetric pair") != std::string::npos)
            { m_bJAsymPopup = true; m_JAsymMsg = R; m_JAsymName = std::format("JointMirror -Bone {} -Force 1", xeditor::Quote(Name)); m_JAsymArgs.clear(); }
            return R;
        }
        // the asymmetric pair's choice: 0 overwrite the other side, 1 this side only, 2 cancel (the commands were kept when it was asked)
        std::string ResolveAsym(int Choice) noexcept
        {
            std::string R = "cancelled: nothing changed";
            if (Choice == 0) R = JointCommand(m_JAsymName);
            if (Choice == 1 && !m_JAsymArgs.empty()) R = JointCommand(m_JAsymArgs);
            m_bJAsymPopup = false; m_JAsymMsg.clear(); m_JointMsg = R;
            return R;
        }

        // the mirror of a limit on the other side's bone (what a symmetric edit will give it), for its ghost
        bool MirrorPreview(int Src, const xanimation::joint_limit& L, int& Dst, xanimation::joint_limit& Out) const
        {
            std::vector<std::string_view> Views(m_JNames.begin(), m_JNames.end());
            Dst = xanimation::FindMirrorBone(Views, Src);
            if (Dst < 0 || Dst == Src) return false;
            const int N = int(m_JNames.size());
            std::vector<xmath::fquat> W(N);
            for (int i = 0; i < N; ++i) W[i] = m_JParents[i] < 0 ? m_JRest.m_Rotation[i] : W[m_JParents[i]] * m_JRest.m_Rotation[i];
            const float Fd = xanimation::MeasureFacingDeg(xanimation::skeleton_view{ m_JParents }, Views, m_JRest) * xskeleton_joints::deg_v;
            Out = xanimation::Mirror(L, W[Src], W[Dst], m_JRest.m_Rotation[Dst], xmath::fvec3(std::cos(Fd), 0, -std::sin(Fd)));
            return true;
        }

        // the joints needing attention: a frame to confirm, a review to redo, outside its limit in the pose shown, an error
        struct joint_attention { int m_Bone; std::string m_Why; bool m_bFrame; };
        std::vector<joint_attention> JointAttention() const
        {
            std::vector<joint_attention> A;
            auto* pDesc = const_cast<session*>(this)->Desc();
            for (int b = 0; b < int(m_JNames.size()); ++b)
            {
                if (m_JointRes.m_Source[b].find("frame needs confirmation") != std::string::npos) A.push_back({ b, "frame to confirm", true });
                if (pDesc) if (const auto* pO = pDesc->findJointOverride(m_JNames[b]); pO && pO->m_Review == xskeleton_desc::joint_review::NEEDS_REVIEW) A.push_back({ b, "changed since its review", false });
                if (m_bTestPose && b < m_TestPose.count() && !xanimation::Evaluate(m_JointRes.m_Limits[b], m_TestPose.m_Rotation[b], 2e-3f).m_bInside) A.push_back({ b, "outside its limit in this pose", false });
            }
            return A;
        }

        // a frame change: the new frame (bone frame) of the selected joint, rotating the envelope with it or keeping it where it is
        std::string ApplyFrame(bool bKeepEnvelope, bool bSym) noexcept
        {
            const int b = m_JFrameBone;
            if (b < 0 || b >= int(m_JNames.size())) return "no frame change pending";
            const auto& L = m_JointRes.m_Limits[b];
            const auto& F = m_JFrameNew;
            std::string Args = std::format("-Frame \"{} {} {} {}\"", F.m_X, F.m_Y, F.m_Z, F.m_W);
            if (bKeepEnvelope && (L.m_Type == xanimation::joint_type::BALL || L.m_Type == xanimation::joint_type::HINGE))
            {
                auto Keep = xanimation::FitFromSamples(L.m_Type, L.m_Rest, F, xanimation::LimitBoundarySamples(L), 0.0f);
                Keep.m_SwingY = L.m_SwingY; Keep.m_SwingZ = L.m_SwingZ; Keep.m_Twist = L.m_Twist;
                Args += " " + JointSetArgs(Keep);
            }
            m_bJFramePending = false; m_JFrameRoll = 0;
            return CommitJointLimit(m_JNames[b], Args, bSym);
        }
        //----------------------------------------------------------------------------------------
        // J4 Analyze clips: the packages' evidence (JointEvidence.bin written by their compile), a widen-only proposal per joint, applied
        // only by the user, as one undo step, and refused when the joints or the skeleton changed since the analysis.
        //----------------------------------------------------------------------------------------
        std::string FitStateKey() noexcept
        {
            auto* pDesc = Desc();
            std::string K = pDesc ? xskeleton_joints::JointState(*pDesc) : std::string{};
            for (const auto& N : m_JNames) { K += '|'; K += N; }
            return K;
        }

        // every package compiled against this skeleton: the AnimPackage logs next to this skeleton's log folder
        void FitLoadSources() noexcept
        {
            m_FitSources.clear();
            if (!UpdateJoints()) return;
            const std::filesystem::path LogsRoot = std::filesystem::path(m_Document.m_LogPath).parent_path().parent_path().parent_path().parent_path();
            const std::uint64_t Me = m_Document.m_Guid.m_Instance.m_Value;
            std::error_code Ec;
            for (std::filesystem::recursive_directory_iterator It(LogsRoot / L"AnimPackage", Ec), End; !Ec && It != End; It.increment(Ec))
            {
                if (It->path().filename() != L"JointEvidence.bin") continue;
                jfit_source S;
                S.m_Path = It->path().wstring();
                if (!xskeleton_joint_evidence::Read(S.m_Path, S.m_E) || S.m_E.m_Skeleton != Me) continue;
                S.m_Package = xstrtool::To(It->path().parent_path().stem().wstring());
                S.m_bStale = S.m_E.m_Bones != m_JNames;            // the skeleton's bones changed since that package compiled
                S.m_bUse = !S.m_bStale;
                m_FitSources.push_back(std::move(S));
            }
            std::sort(m_FitSources.begin(), m_FitSources.end(), [](const auto& A, const auto& B) { return A.m_Package < B.m_Package; });
        }

        std::string FitAnalyze() noexcept
        {
            using namespace xanimation;
            m_FitRows.clear(); m_FitFrames = 0; m_FitFree = 0; m_bFitOpen = true;
            if (!UpdateJoints()) return "no compiled skeleton";
            auto* pDesc = Desc();
            int nUsed = 0, nStale = 0;
            for (const auto& S : m_FitSources) { nUsed += S.m_bUse && !S.m_bStale; nStale += S.m_bStale; for (const auto& C : S.m_E.m_Clips) if (S.m_bUse && !S.m_bStale) m_FitFrames += C.m_nFrames; }
            if (!nUsed) return m_FitSources.empty() ? "no package on this skeleton has evidence yet (compile a package that uses it)"
                                                    : std::format("no usable package ({} stale: their skeleton bones differ - recompile them)", nStale);
            const int N = int(m_JNames.size());
            std::vector<xmath::fquat> Samples; std::vector<std::array<int, 3>> Where;
            for (int b = 0; b < N; ++b)
            {
                const auto& L = m_JointRes.m_Limits[b];
                if (L.m_Type == joint_type::FREE) { ++m_FitFree; continue; }
                Samples.clear(); Where.clear();
                for (int s = 0; s < int(m_FitSources.size()); ++s)
                {
                    const auto& S = m_FitSources[s];
                    if (!S.m_bUse || S.m_bStale) continue;
                    for (int c = 0; c < int(S.m_E.m_Clips.size()); ++c)
                        for (int f = 0; f < S.m_E.m_Clips[c].m_nFrames; ++f) { Samples.push_back(S.m_E.m_Clips[c].m_Rotations[std::size_t(f) * N + b]); Where.push_back({ s, c, f }); }
                }
                jfit_row Rw; Rw.m_Bone = b;
                Rw.m_New = ProposeExpansion(L, Samples, 2.0f * xskeleton_joints::deg_v, &Rw.m_Rep, 3.0f * xskeleton_joints::deg_v);     // real clips wander off a hinge a little
                if (!Rw.m_Rep.m_nOutside && !Rw.m_Rep.m_nOffModel && !Rw.m_Rep.m_nSingular) continue;
                // coverage: distinct poses (5 degree cells of the joint's swing and twist), not raw frames
                std::set<std::tuple<int, int, int>> Cells;
                for (const auto& Q : Samples)
                {
                    float Sy, Sz, Tw; bool Sg; details::SplitBall(JointRotation(L, Q), Sy, Sz, Tw, Sg);
                    const float C = 1.0f / (5.0f * xskeleton_joints::deg_v);
                    Cells.insert({ int(std::floor(Sy * C)), int(std::floor(Sz * C)), int(std::floor(Tw * C)) });
                }
                Rw.m_nCoverage = int(Cells.size());
                if (Rw.m_Rep.m_iWorst >= 0) { Rw.m_iSrc = Where[Rw.m_Rep.m_iWorst][0]; Rw.m_iClip = Where[Rw.m_Rep.m_iWorst][1]; Rw.m_iFrame = Where[Rw.m_Rep.m_iWorst][2]; }
                const auto* pO = pDesc->findJointOverride(m_JNames[b]);
                Rw.m_bProtected = pO && pO->m_Review == xskeleton_desc::joint_review::REVIEWED && !m_bFitIncludeReviewed;
                Rw.m_bChanges  = xskeleton_joints::Describe(Rw.m_New) != xskeleton_joints::Describe(L);
                Rw.m_bSelected = !Rw.m_bProtected && Rw.m_bChanges;
                m_FitRows.push_back(Rw);
            }
            m_FitKey = FitStateKey();
            std::string T = std::format("{} package(s), {} frames; {} joint(s) would change, {} free joints (nothing to widen){}\n", nUsed, m_FitFrames
                , std::count_if(m_FitRows.begin(), m_FitRows.end(), [](const auto& R) { return R.m_bChanges; }), m_FitFree, nStale ? std::format("; {} stale package(s) skipped", nStale) : "");
            for (const auto& Rw : m_FitRows)
                T += std::format("  {}: {} -> {} | outside {} of {} (worst {:.1f} deg) | off-model {} | coverage {} poses{}{}\n", m_JNames[Rw.m_Bone]
                    , xskeleton_joints::Describe(m_JointRes.m_Limits[Rw.m_Bone]), xskeleton_joints::Describe(Rw.m_New), Rw.m_Rep.m_nOutside, Rw.m_Rep.m_nSamples
                    , Rw.m_Rep.m_WorstExcess / xskeleton_joints::deg_v, Rw.m_Rep.m_nOffModel, Rw.m_nCoverage, Rw.m_bProtected ? " | reviewed: protected" : "", !Rw.m_bChanges ? " | no change (only off-model samples: check the frame)" : Rw.m_bSelected ? " | selected" : "");
            return T;
        }

        std::string FitApply() noexcept
        {
            if (m_FitRows.empty()) return "nothing to apply (Analyze clips first)";
            if (FitStateKey() != m_FitKey) return "the joints or the skeleton changed since the analysis: analyze again (stale proposal refused)";
            std::string Pk;
            for (const auto& S : m_FitSources) if (S.m_bUse && !S.m_bStale) Pk += (Pk.empty() ? "" : " ") + S.m_Package;
            int n = 0;
            const auto Err = CommitJointEdit("Joint limits fitted from clips", [&](desc& D) -> std::string
            {
                for (const auto& Rw : m_FitRows)
                {
                    if (!Rw.m_bSelected || !Rw.m_bChanges) continue;
                    if (Rw.m_bProtected) return std::format("'{}' is reviewed: analyze with IncludeReviewed to change it", m_JNames[Rw.m_Bone]);
                    xskeleton_desc::joint_override O;
                    const bool bOld = D.findJointOverride(m_JNames[Rw.m_Bone]) != nullptr;
                    if (bOld) O = *D.findJointOverride(m_JNames[Rw.m_Bone]); else O.m_BoneName = m_JNames[Rw.m_Bone];
                    xskeleton_joints::SetOverrideFields(O, Rw.m_New, O.m_bFrameSet);
                    if (O.m_bFrameSet) O.m_Frame = Rw.m_New.m_Frame;
                    O.m_Source = xskeleton_desc::joint_source::FITTED;
                    O.m_Provenance = std::format("fitted from clips: packages {} ({} frames)", Pk, m_FitFrames);
                    if (bOld) *D.findJointOverride(m_JNames[Rw.m_Bone]) = O; else D.m_JointOverrides.push_back(O);
                    ++n;
                }
                return n ? std::string{} : std::string("no selected row changes anything");
            });
            if (!Err.empty()) return Err;
            m_FitRows.clear(); m_JointKey.clear();
            return std::format("{} joint(s) widened to what the clips do (one undo step)", n);
        }

        // the worst frame of a proposal row becomes the test pose (never saved)
        std::string FitShowWorst(const std::string& Bone) noexcept
        {
            for (const auto& Rw : m_FitRows)
            {
                if (Rw.m_Bone < 0 || m_JNames[Rw.m_Bone] != Bone || Rw.m_iSrc < 0) continue;
                const auto& C = m_FitSources[Rw.m_iSrc].m_E.m_Clips[Rw.m_iClip];
                m_TestPose = m_JRest;
                for (int i = 0; i < int(m_JNames.size()); ++i) m_TestPose.m_Rotation[i] = C.m_Rotations[std::size_t(Rw.m_iFrame) * m_JNames.size() + i];
                m_bTestPose = true; m_bJointsMode = true; m_Selected = { Rw.m_Bone };
                return std::format("test pose = '{}' frame {} (package {}), the worst for {}", C.m_Name, Rw.m_iFrame, m_FitSources[Rw.m_iSrc].m_Package, Bone);
            }
            return std::format("no proposal row for '{}' (Analyze clips first)", Bone);
        }

        // the hinge axis (z) snapped to the axis the clips bend this joint about (the principal axis of its rotations from rest)
        bool FrameFromClips(int b, xmath::fquat& Out)
        {
            if (m_FitSources.empty()) FitLoadSources();
            const int N = int(m_JNames.size());
            if (b < 0 || b >= N) return false;
            double M[3][3] = {}; double n = 0;
            std::vector<xmath::fvec3> V;
            // a hinge bends about an axis across the bone: the twist about the long axis (x) is taken out before the principal axis
            const auto Xb = (m_JointRes.m_Limits[b].m_Frame * xmath::fvec3(1, 0, 0)).NormalizeSafeCopy();
            for (const auto& S : m_FitSources)
            {
                if (!S.m_bUse || S.m_bStale) continue;
                for (const auto& C : S.m_E.m_Clips)
                    for (int f = 0; f < C.m_nFrames; ++f)
                    {
                        auto v = xanimation::details::LogQ(xanimation::details::Canon(m_JRest.m_Rotation[b].ConjugateCopy() * C.m_Rotations[std::size_t(f) * N + b]));
                        v = v - Xb * v.Dot(Xb);
                        V.push_back(v);
                        const double a[3] = { v.m_X, v.m_Y, v.m_Z };
                        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) M[i][j] += a[i] * a[j];
                        n += v.Length();
                    }
            }
            if (V.empty() || n / V.size() < 2.0 * xskeleton_joints::deg_v) return false;     // the clips barely move it: nothing to say
            double a[3] = { 0, 0, 1 };
            for (int it = 0; it < 50; ++it)
            {
                double r[3] = {};
                for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r[i] += M[i][j] * a[j];
                const double l = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]); if (l < 1e-12) return false;
                for (int i = 0; i < 3; ++i) a[i] = r[i] / l;
            }
            xmath::fvec3 Ax{ static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]) };
            double Sgn = 0; for (const auto& v : V) Sgn += v.Dot(Ax);
            if (Sgn < 0) Ax = Ax * -1.0f;                                       // bending the way the clips mostly bend is +z
            const auto& L = m_JointRes.m_Limits[b];
            const auto X = (L.m_Frame * xmath::fvec3(1, 0, 0)).NormalizeSafeCopy();
            auto Z = Ax - X * Ax.Dot(X);
            if (Z.LengthSq() < 1e-6f) return false;
            Z = Z.NormalizeCopy();
            const auto Y = Z.Cross(X).NormalizeCopy();
            Out = xanimation::QuatFromBasis(X, Y, X.Cross(Y).NormalizeCopy()).NormalizeCopy();
            return true;
        }

        // the hinge axis snapped to the bend the test pose shows at this joint (between the parent's segment and the bone's)
        bool FrameFromPoseBend(int b, xmath::fquat& Out) const
        {
            if (!m_bTestPose || b < 0 || m_JParents[b] < 0) return false;
            const int c = FirstChild(b);
            if (c < 0) return false;
            std::vector<xmath::fquat> R; std::vector<xmath::fvec3> P;
            xanimation::details::WorldRP(xanimation::skeleton_view{ m_JParents }, m_TestPose, R, P);
            const auto d1 = (P[b] - P[m_JParents[b]]).NormalizeSafeCopy(), d2 = (P[c] - P[b]).NormalizeSafeCopy();
            const auto n = d1.Cross(d2);
            if (n.Length() < std::sin(3.0f * xskeleton_joints::deg_v)) return false;          // not bent enough to say which way
            const auto& L = m_JointRes.m_Limits[b];
            const auto Xw = (R[b] * L.m_Frame) * xmath::fvec3(1, 0, 0);
            auto Zw = n - Xw * n.Dot(Xw);
            if (Zw.LengthSq() < 1e-8f) return false;
            Zw = Zw.NormalizeCopy();
            const auto Yw = Zw.Cross(Xw).NormalizeCopy();
            Out = (R[b].ConjugateCopy() * xanimation::QuatFromBasis(Xw, Yw, Xw.Cross(Yw).NormalizeCopy())).NormalizeCopy();
            return true;
        }

        // Include this pose: the proposal for the selected joint from the pose it is in now (its limit lifted while pulling)
        void UpdateIncludeProposal() noexcept
        {
            using namespace xanimation;
            m_bJIncludeValid = false;
            const int b = m_JIncludeBone;
            if (!m_bJInclude || b < 0 || b >= int(m_JNames.size()) || !m_bTestPose) return;
            const auto& L = m_JointRes.m_Limits[b];
            const auto  Q = m_TestPose.m_Rotation[b];
            constexpr float Margin = 2.0f * 3.14159265f / 180.0f;
            if (m_bJIncludeBall && L.m_Type == joint_type::HINGE)
            {
                auto S = LimitBoundarySamples(L, 16); S.push_back(Q);
                m_JIncludeProposal = FitFromSamples(joint_type::BALL, L.m_Rest, L.m_Frame, S, Margin, &m_JIncludeRep);
            }
            else if (L.m_Type == joint_type::FREE)
            {   // nothing yet: a ball around rest and this pose
                const xmath::fquat Rs[2] = { L.m_Rest, Q };
                frame_confidence C;
                m_JIncludeProposal = FitFromSamples(joint_type::BALL, L.m_Rest, SuggestFrame(m_JParents, m_JRest, b, C), std::span<const xmath::fquat>(Rs, 2), Margin, &m_JIncludeRep);
            }
            else m_JIncludeProposal = ProposeExpansion(L, std::span<const xmath::fquat>(&Q, 1), Margin, &m_JIncludeRep);
            m_bJIncludeValid = true;
        }
        std::string IncludeAccept(bool bSym) noexcept
        {
            UpdateIncludeProposal();
            if (!m_bJIncludeValid) return "Include this pose: pull the chain first";
            const auto R = CommitJointLimit(m_JNames[m_JIncludeBone], JointSetArgs(m_JIncludeProposal), bSym);
            m_bJInclude = m_bJIncludeBall = false; m_JointKey.clear();
            return R;
        }

        // the guided check of the selected joint: bend, extend, twist (parent held), then the other side; asks at the end
        void StepCheck() noexcept
        {
            using namespace xanimation;
            if (m_JCheckT < 0 || m_JCheckBone < 0 || m_JCheckBone >= int(m_JNames.size())) return;
            m_JCheckT += std::min(ImGui::GetIO().DeltaTime, 0.1f);
            const auto& L = m_JointRes.m_Limits[m_JCheckBone];
            const int nAxes = L.m_Type == joint_type::HINGE ? 1 : L.m_Type == joint_type::BALL ? 3 : 0;
            constexpr float Phase = 1.6f;
            const int   Ph = int(m_JCheckT / Phase);
            const float s  = std::sin((m_JCheckT - Ph * Phase) / Phase * 6.2831853f);
            m_TestPose = m_JRest; m_bTestPose = true;
            if (Ph < nAxes)
            {
                xmath::fquat J;
                if (L.m_Type == joint_type::HINGE) { const float m = 0.5f * (L.m_HingeMin + L.m_HingeMax), h = 0.5f * (L.m_HingeMax - L.m_HingeMin); J = details::AxisAngle(details::AxZ, m + h * s); m_JCheckLabel = "bend and extend through the hinge range"; }
                else if (Ph == 0) { J = details::MakeBall(L.m_SwingCy + (L.m_SwingY == axis_motion::LIMITED ? L.m_SwingRy : 0.0f) * s, L.m_SwingCz, L.m_TwistCentre); m_JCheckLabel = "swing along y"; }
                else if (Ph == 1) { J = details::MakeBall(L.m_SwingCy, L.m_SwingCz + (L.m_SwingZ == axis_motion::LIMITED ? L.m_SwingRz : 0.0f) * s, L.m_TwistCentre); m_JCheckLabel = "swing along z"; }
                else { J = details::MakeBall(L.m_SwingCy, L.m_SwingCz, L.m_TwistCentre + (L.m_Twist == axis_motion::LIMITED ? std::min(L.m_TwistHalf, 3.0f) : 0.0f) * s); m_JCheckLabel = "twist"; }
                m_TestPose.m_Rotation[m_JCheckBone] = LocalRotation(L, J);
                m_JCheckLabel = std::format("Checking {} ({}/{}): {}", m_JNames[m_JCheckBone], Ph + 1, nAxes + 1, m_JCheckLabel);
            }
            else if (Ph == nAxes) m_JCheckLabel = std::format("Checking {} ({}/{}): the other side (ghost) should mirror it", m_JNames[m_JCheckBone], nAxes + 1, nAxes + 1);
            else { m_JCheckT = -2; m_bTestPose = false; m_JCheckLabel = std::format("Did {} look right? Mark it reviewed in the Joints panel.", m_JNames[m_JCheckBone]); }
        }

        // a limit's shape: a ball's swing ring (spokes) and twist arc, a hinge's arc; dashed while not reviewed
        void DrawLimitShape(ImDrawList* pDraw, const ImVec2& Min, const ImVec2& Size, const joint_draw_ctx& K, const xanimation::joint_limit& L, const xanimation::joint_eval& Eval
                           , ImU32 Base, float Thick, bool bDashed, bool bSpokes) const noexcept
        {
            using namespace xanimation;
            const ImU32 Hot = IM_COL32(255, 80, 60, 255);
            auto Line = [&](const xmath::fvec3& A, const xmath::fvec3& B, ImU32 Col, float T) { ImVec2 a, bb; if (ProjectView(A, Min, Size, a) && ProjectView(B, Min, Size, bb)) pDraw->AddLine(a, bb, Col, T); };
            auto Poly = [&](const std::vector<xmath::fvec3>& Pts, ImU32 Col, bool bLoop)
            {
                const int n = int(Pts.size());
                for (int i = 0; i + 1 < n + (bLoop ? 1 : 0); ++i) if (!bDashed || (i & 1) == 0) Line(Pts[i], Pts[(i + 1) % n], Col, Thick);
            };
            if (L.m_Type == joint_type::BALL)
            {
                std::vector<xmath::fvec3> Ring;
                for (int k = 0; k < 48; ++k) { float Sy, Sz; SwingBoundary(L, k * 6.2831853f / 48, Sy, Sz); Ring.push_back(ConePoint(K, Sy, Sz)); }
                const ImU32 Col = (Eval.m_Faces & joint_face::SWING) ? Hot : Base;
                Poly(Ring, Col, true);
                if (bSpokes) for (int k = 0; k < 48; k += 12) Line(K.P0, Ring[k], Col, 1.0f);
                if (L.m_Twist == axis_motion::LIMITED && L.m_TwistHalf < 3.14f)
                {
                    std::vector<xmath::fvec3> Arc;
                    for (int k = 0; k <= 16; ++k) Arc.push_back(TwistPoint(K, L.m_TwistCentre - L.m_TwistHalf + 2 * L.m_TwistHalf * k / 16));
                    Poly(Arc, (Eval.m_Faces & joint_face::TWIST) ? Hot : Base, false);
                }
            }
            else if (L.m_Type == joint_type::HINGE)
            {
                std::vector<xmath::fvec3> Arc;
                for (int k = 0; k <= 32; ++k) Arc.push_back(HingePoint(K, L.m_HingeMin + (L.m_HingeMax - L.m_HingeMin) * k / 32));
                Poly(Arc, Base, false);
                Line(K.P0, Arc.front(), (Eval.m_Faces & joint_face::HINGE_LOW) ? Hot : Base, std::max(1.0f, Thick - 0.5f));
                Line(K.P0, Arc.back(),  (Eval.m_Faces & joint_face::HINGE_HIGH) ? Hot : Base, std::max(1.0f, Thick - 0.5f));
            }
        }

        // a joint's state as a tag (not colour alone): R reviewed, * changed since review, ? frame to confirm, ! outside its limit
        std::string JointTag(int b, const xanimation::joint_eval& Eval) const
        {
            auto* pDesc = const_cast<session*>(this)->Desc();
            std::string T;
            if (pDesc) if (const auto* pO = pDesc->findJointOverride(m_JNames[b]); pO)
            {
                if (pO->m_Review == xskeleton_desc::joint_review::REVIEWED) T += "R";
                if (pO->m_Review == xskeleton_desc::joint_review::NEEDS_REVIEW) T += "*";
            }
            if (m_JointRes.m_Source[b].find("frame needs confirmation") != std::string::npos) T += "?";
            if (!Eval.m_bInside) T += "!";
            return T;
        }

        void DrawJointOverlay(const ImVec2& Min, const ImVec2& Size, float Scale, const ImVec2& Mouse) noexcept
        {
            using namespace xanimation;
            auto* pDraw = ImGui::GetWindowDrawList();
            if (!UpdateJoints() || m_Scaled.size() != m_JNames.size() || m_CurR.size() != m_JNames.size()) return;
            StepCheck();
            auto Text = [&](ImVec2 P, ImU32 Col, const std::string& S)       // text on a dark backing: readable over the floor and the bones
            {
                const ImVec2 Sz = ImGui::CalcTextSize(S.c_str());
                pDraw->AddRectFilled(ImVec2(P.x - 3, P.y - 2), ImVec2(P.x + Sz.x + 3, P.y + Sz.y + 2), IM_COL32(20, 20, 20, 190), 3.0f);
                pDraw->AddText(P, Col, S.c_str());
            };
            const auto Status = std::format("Joints - limits {} - {}", Desc()->m_JointLimits == xskeleton_desc::joint_limits_mode::ON ? "on" : "off (preview)"
                                           , m_bJInclude ? "INCLUDE THIS POSE: pull the chain; only this joint's limit is lifted (Include / Cancel in the panel)"
                                           : m_bTestPose ? "preview pose, not saved (Reset pose in the Joints panel)" : "drag a handle to edit, the orange one to test");
            Text(ImVec2(Min.x + 8, Min.y + 8), IM_COL32(255, 230, 150, 255), Status);
            if (m_JCheckT != -1 && !m_JCheckLabel.empty()) Text(ImVec2(Min.x + 8, Min.y + 30), IM_COL32(170, 230, 255, 255), m_JCheckLabel);

            const int b = SelectedJoint();
            auto* pDesc = Desc();
            auto Reviewed = [&](int j) { const auto* pO = pDesc->findJointOverride(m_JNames[j]); return pO && pO->m_Review == xskeleton_desc::joint_review::REVIEWED; };
            auto LocalOf  = [&](int j) { return m_bTestPose && j < m_TestPose.count() ? m_TestPose.m_Rotation[j] : m_JRest.m_Rotation[j]; };

            // the filter: other joints drawn thin, no handles (Selected: only the selected joint)
            if (m_JFilter != joint_filter::SELECTED)
            {
                std::vector<std::uint8_t> Show(m_JNames.size(), 0);
                if (m_JFilter == joint_filter::ALL) std::fill(Show.begin(), Show.end(), std::uint8_t(1));
                else if (m_JFilter == joint_filter::ATTENTION) for (const auto& A : JointAttention()) Show[A.m_Bone] = 1;
                else if (b >= 0)
                {
                    for (int a = m_JParents[b]; a >= 0; a = m_JParents[a]) Show[a] = 1;
                    for (int a = FirstChild(b); a >= 0; a = FirstChild(a)) Show[a] = 1;
                }
                int nDrawn = 0;
                for (int j = 0; j < int(m_JNames.size()) && nDrawn < 200; ++j)
                {
                    if (!Show[j] || j == b || m_JointRes.m_Limits[j].m_Type == joint_type::FREE) continue;
                    const auto& L = m_JointRes.m_Limits[j];
                    const auto  Ev = Evaluate(L, LocalOf(j), 2e-3f);
                    DrawLimitShape(pDraw, Min, Size, JointCtx(j, L, LocalOf(j)), L, Ev, IM_COL32(150, 170, 200, 170), 1.0f, !Reviewed(j), false);
                    if (const auto Tag = JointTag(j, Ev); !Tag.empty()) { ImVec2 S; if (ProjectView(m_Scaled[j].m_Position, Min, Size, S)) Text(ImVec2(S.x + 6, S.y - 16), IM_COL32(255, 220, 120, 255), Tag); }
                    ++nDrawn;
                }
            }

            m_bPullHover = false; m_JHandleHover = -1; m_JHandles.clear();
            if (b < 0) { m_PullEff = -1; return; }
            const auto  Local = LocalOf(b);
            const bool  bReviewed = Reviewed(b);
            const ImU32 Base = bReviewed ? IM_COL32(120, 230, 140, 255) : IM_COL32(120, 190, 255, 255);

            // the limit shown: the drag's preview, the Include proposal (over the old one, dimmed), a pending frame, or the resolved limit
            if (m_bJInclude && m_JIncludeBone == b) UpdateIncludeProposal();
            const auto& Resolved = m_JointRes.m_Limits[b];
            xanimation::joint_limit L = m_JDrag >= 0 ? m_JPreview : (m_bJInclude && m_bJIncludeValid && m_JIncludeBone == b) ? m_JIncludeProposal : Resolved;
            const auto  K    = JointCtx(b, L, Local);
            const auto  Eval = Evaluate(L, Local, 2e-3f);
            if (L.m_Type != Resolved.m_Type || m_JDrag >= 0 || (m_bJInclude && m_JIncludeBone == b))
                DrawLimitShape(pDraw, Min, Size, JointCtx(b, Resolved, Local), Resolved, Evaluate(Resolved, Local, 2e-3f), IM_COL32(160, 160, 160, 150), 1.0f, true, false);   // the old one
            DrawLimitShape(pDraw, Min, Size, K, L, Eval, m_bJInclude ? IM_COL32(255, 210, 90, 255) : Base, 2.0f, !bReviewed && m_JDrag < 0, true);
            if (m_bJFramePending && m_JFrameBone == b)
            {   // the pending frame: the envelope rotated with it (solid orange) and kept where it is (dotted cyan)
                auto Rot = Resolved; Rot.m_Frame = m_JFrameNew;
                DrawLimitShape(pDraw, Min, Size, JointCtx(b, Rot, Local), Rot, Evaluate(Rot, Local, 2e-3f), IM_COL32(255, 150, 60, 230), 1.5f, false, false);
                auto Keep = FitFromSamples(Resolved.m_Type == joint_type::HINGE ? joint_type::HINGE : joint_type::BALL, Resolved.m_Rest, m_JFrameNew, LimitBoundarySamples(Resolved), 0.0f);
                DrawLimitShape(pDraw, Min, Size, JointCtx(b, Keep, Local), Keep, Evaluate(Keep, Local, 2e-3f), IM_COL32(90, 220, 255, 230), 1.5f, true, false);
            }

            // the frame's axes: x the long axis (red), y (green), z the hinge axis (blue)
            {
                const auto& Fq = (m_bJFramePending && m_JFrameBone == b) ? m_JFrameNew : L.m_Frame;
                const auto ParentR = m_JParents[b] >= 0 ? m_CurR[m_JParents[b]] : xmath::fquat::fromIdentity();
                const auto Ap = (ParentR * L.m_Rest * Fq).NormalizeCopy();
                const xmath::fvec3 Ax[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
                const ImU32 Cols[3] = { IM_COL32(230, 70, 70, 220), IM_COL32(80, 220, 80, 220), IM_COL32(80, 130, 255, 220) };
                for (int i = 0; i < 3; ++i) { ImVec2 a, e; if (ProjectView(K.P0, Min, Size, a) && ProjectView(K.P0 + (Ap * Ax[i]) * (K.Len * 0.35f), Min, Size, e)) pDraw->AddLine(a, e, Cols[i], 2.0f); }
            }

            // the other side's ghost while symmetry would change it (a handle drag, Include, the check's last step)
            std::vector<std::string_view> Views(m_JNames.begin(), m_JNames.end());
            const int Other = FindMirrorBone(Views, b);
            const bool bSym = m_bJointSymmetry && Other >= 0 && Other != b;
            if (bSym && (m_JDrag >= 0 || m_bJInclude || m_JCheckT >= 0))
            {
                int Dst = -1; xanimation::joint_limit M;
                if (MirrorPreview(b, L, Dst, M) && Dst >= 0 && Dst < int(m_Scaled.size()))
                {
                    DrawLimitShape(pDraw, Min, Size, JointCtx(Dst, M, LocalOf(Dst)), M, Evaluate(M, LocalOf(Dst), 2e-3f), IM_COL32(220, 160, 255, 200), 1.5f, true, false);
                    ImVec2 S; if (ProjectView(m_Scaled[Dst].m_Position, Min, Size, S)) Text(ImVec2(S.x + 8, S.y + 8), IM_COL32(220, 180, 255, 255), std::format("{} (mirrored)", m_JNames[Dst]));
                }
            }

            // where the bone points now
            {
                const auto Now = (m_CurR[b] * L.m_Frame) * xmath::fvec3(1, 0, 0);
                ImVec2 a, e; if (ProjectView(K.P0, Min, Size, a) && ProjectView(K.P0 + Now * K.Len, Min, Size, e)) pDraw->AddLine(a, e, Eval.m_bInside ? IM_COL32(255, 220, 80, 255) : IM_COL32(255, 80, 60, 255), 2.5f);
            }

            // the handles (not while pulling, checking or including)
            const bool bHandles = !m_bPulling && m_JCheckT < 0 && !m_bJInclude && !(m_bJFramePending && m_JFrameBone == b);
            if (bHandles)
                for (int k = 0; k < JH_COUNT; ++k)
                {
                    if (!HandleUsed(L, k)) continue;
                    ImVec2 S; if (!ProjectView(HandlePoint(K, L, k), Min, Size, S)) continue;
                    m_JHandles.push_back({ k, S });
                    const float dx = Mouse.x - S.x, dy = Mouse.y - S.y;
                    const bool bHot = m_JDrag == k || (m_JDrag < 0 && dx * dx + dy * dy < 64.0f);
                    if (bHot && m_JDrag < 0) m_JHandleHover = k;
                    const float R = bHot ? 6.0f : 4.5f;
                    if (k == JH_CENTRE) pDraw->AddRectFilled(ImVec2(S.x - R, S.y - R), ImVec2(S.x + R, S.y + R), bHot ? IM_COL32(255, 255, 255, 255) : IM_COL32(240, 240, 240, 220));
                    else pDraw->AddCircleFilled(S, R, bHot ? IM_COL32(255, 255, 255, 255) : IM_COL32(240, 240, 240, 220));
                    pDraw->AddCircle(S, R, IM_COL32(20, 20, 20, 255), 0, 1.5f);
                    if (bHot && m_JDrag < 0) Text(ImVec2(S.x + 10, S.y - 18), IM_COL32(255, 255, 255, 255), std::format("{}{}", JHName(k), k <= JH_RZ_N ? " (Shift: both)" : ""));
                }

            // the handle drag: preview while held, Esc cancels, release commits one JointSet
            if (m_JDrag >= 0)
            {
                if (ImGui::IsKeyPressed(ImGuiKey_Escape)) { m_JDrag = -1; m_JDragTip.clear(); m_JointMsg = "drag cancelled: nothing changed"; }
                else if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
                {
                    DragHandle(JointCtx(b, m_JDragStart, Local), Min, Size, Mouse);
                    Text(ImVec2(Mouse.x + 16, Mouse.y + 12), IM_COL32(255, 255, 255, 255), m_JDragTip + (bSym ? std::format("\nalso {} (mirrored)", m_JNames[Other]) : ""));
                }
                else
                {
                    float q0[2], q1[2];
                    const int n = HandleParams(m_JDragStart, m_JDrag, false, q0); HandleParams(m_JPreview, m_JDrag, false, q1);
                    const bool bChanged = std::abs(q1[0] - q0[0]) > 1e-5f || (n == 2 && std::abs(q1[1] - q0[1]) > 1e-5f) || (m_JDragShift && std::abs(m_JPreview.m_SwingRz - m_JDragStart.m_SwingRz) > 1e-5f);
                    m_JDrag = -1;
                    if (bChanged) CommitJointLimit(m_JNames[b], JointSetArgs(m_JPreview), bSym);
                }
            }

            // the asymmetric pair: never overwritten without a choice
            if (m_bJAsymPopup) ImGui::OpenPopup("Asymmetric pair##joints");
            if (ImGui::BeginPopupModal("Asymmetric pair##joints", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                if (!m_bJAsymPopup) ImGui::CloseCurrentPopup();         // answered elsewhere (the JointsView command): the modal must not stay and block the view
                ImGui::TextWrapped("%s", m_JAsymMsg.c_str());
                if (ImGui::Button("Overwrite the other side")) { ResolveAsym(0); ImGui::CloseCurrentPopup(); }
                if (!m_JAsymArgs.empty()) { ImGui::SameLine(); if (ImGui::Button("This side only")) { ResolveAsym(1); ImGui::CloseCurrentPopup(); } }
                ImGui::SameLine(); if (ImGui::Button("Cancel")) { ResolveAsym(2); ImGui::CloseCurrentPopup(); }
                ImGui::EndPopup();
            }

            {   // the label: what is allowed, where it came from, its state
                ImVec2 S;
                if (ProjectView(K.P0, Min, Size, S))
                {
                    const auto Tag = JointTag(b, Eval);
                    auto Txt = std::format("{}  {}{}\n{}{}", m_JNames[b], xskeleton_joints::Describe(L), Tag.empty() ? "" : "  [" + Tag + "]", m_JointRes.m_Source[b]
                                          , Eval.m_bInside ? "" : std::format("  - outside by {:.1f} deg", Eval.m_Excess * 57.29578f));
                    if (m_bJInclude && m_bJIncludeValid && m_JIncludeBone == b)
                        Txt += std::format("\nInclude: {}{}", xskeleton_joints::Describe(m_JIncludeProposal)
                                          , m_JIncludeRep.m_nOffModel ? "\nthis pose needs another axis: 'Make it a ball' in the panel" : "");
                    Text(ImVec2(S.x + 10, S.y + 10), IM_COL32(255, 255, 255, 240), Txt);
                }
            }

            // the test pull: the handle at the end of the selected bone (its first child)
            const int c = FirstChild(b);
            m_PullEff = c;
            if (c < 0 || m_JCheckT >= 0) return;
            ImVec2 Hs;
            if (!ProjectView(m_Scaled[c].m_Position, Min, Size, Hs)) return;
            m_bPullHover = !m_bPulling && m_JHandleHover < 0 && m_JDrag < 0 && handles::Near(Mouse, Hs);
            m_HandleScreen = Hs;
            handles::Draw(*pDraw, Hs, handles::kind::pull, m_bPullHover, m_bPulling);       // the same handle as the animation editor's Pose tool

            if (m_bPulling)
            {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
                {
                    // the mouse on the plane through the handle, facing the camera
                    const auto Cam = m_Scene.m_View.getPosition();
                    const auto Dir = m_Scene.m_View.RayFromScreen(Mouse.x, Mouse.y);
                    const auto N   = (m_PullPlane - Cam).NormalizeSafeCopy();
                    const float Den = Dir.Dot(N);
                    if (std::abs(Den) > 1e-6f) m_PullTargetView = Cam + Dir * ((m_PullPlane - Cam).Dot(N) / Den);
                    // model space: the frozen view position of the effector plus the move, unscaled
                    std::vector<xmath::fquat> Rr; std::vector<xmath::fvec3> Pr;
                    details::WorldRP(skeleton_view{ m_JParents }, m_JRest, Rr, Pr);
                    ik_task T; T.m_Bone = m_PullEff; T.m_bPosition = true;
                    T.m_Position = Pr[m_PullEff] + (m_PullTargetView - m_FrozenView[m_PullEff]) * (1.0f / std::max(Scale, 1e-6f));
                    std::vector<std::uint8_t> W(m_JNames.size(), 0);
                    for (int k = 0, a = m_PullBone; k < 3 && a >= 0 && m_JParents[a] >= 0; ++k, a = m_JParents[a]) W[a] = 1;     // the bone and two above it; the root never
                    m_Ik.Solve(m_PullBase, W, { &T, 1 }, { &m_PullRep, 1 }, m_TestPose);
                    m_bTestPose = true;
                    m_PullNote = std::format("{} {}", m_PullRep.m_Status == ik_status::REACHED ? "reached" : std::format("missed by {:.1f} cm:", m_PullRep.m_PosMiss * 100.0f), m_PullRep.m_Status == ik_status::REACHED ? "" : PullReason(m_PullRep, W));
                }
                else { m_bPulling = false; m_Ik.Setup(m_JParents, m_JointRes.m_Limits); }
            }
            if (m_bTestPose && !m_PullNote.empty())
            {
                ImVec2 G;
                if (ProjectView(m_PullTargetView, Min, Size, G))
                {
                    pDraw->AddCircle(G, 9.0f, IM_COL32(255, 255, 255, 200), 0, 2.0f);                // the target asked for (a ghost)
                    if (m_PullRep.m_Status != ik_status::REACHED) pDraw->AddLine(Hs, G, IM_COL32(255, 80, 60, 220), 1.5f);   // the gap
                    Text(ImVec2(G.x + 12, G.y - 8), m_PullRep.m_Status == ik_status::REACHED ? IM_COL32(150, 255, 150, 255) : IM_COL32(255, 160, 140, 255), m_PullNote);
                }
            }
        }

        std::string JointCommand(const std::string& Cmd) noexcept
        {
            m_JointMsg = m_Undo.Query(Cmd);
            m_JointKey.clear();
            return m_JointMsg;
        }

        void RenderJointsPanel() noexcept
        {
            using namespace xanimation;
            if (ImGui::Checkbox("Joints mode", &m_bJointsMode))
            {
                if (m_bJointsMode) m_Settings.m_Pose = pose_mode::FROZEN;       // limits are measured from the rest (frozen) pose
                else { m_bTestPose = m_bPulling = m_bJInclude = m_bJFramePending = false; m_JCheckT = -1; m_PullNote.clear(); }
            }
            if (ImGui::IsItemHovered()) xeditor::hint::Text("Shows the selected joint's allowed range on the bone: drag its handles to change it, pull the orange handle to test it. The test pose is never saved.");
            auto* pDesc = Desc();
            if (!pDesc || !UpdateJoints()) { ImGui::TextDisabled("Compile the skeleton to see its joints."); return; }

            const bool bOn = pDesc->m_JointLimits == xskeleton_desc::joint_limits_mode::ON;
            int nReviewed = 0;
            for (const auto& O : pDesc->m_JointOverrides) nReviewed += O.m_Review == xskeleton_desc::joint_review::REVIEWED;
            ImGui::Text("Limits: %s", bOn ? "On" : "Off (what you see is a preview)");
            ImGui::SameLine();
            if (ImGui::SmallButton(bOn ? "Turn off" : "Turn on")) JointCommand(std::format("JointLimits -Mode {}", bOn ? "Off" : "On"));
            {
                static constexpr const char* Profiles[] = { "None", "Humanoid v1", "Humanoid v2" };
                int Pf = int(xskeleton_joints::EffectiveProfile(*pDesc));
                ImGui::SetNextItemWidth(140);
                ImGui::BeginDisabled(!bOn); const bool bPick = ImGui::Combo("Profile", &Pf, Profiles, 3); ImGui::EndDisabled();
                if (bPick && Pf != int(pDesc->m_JointProfile)) JointCommand(std::format("JointLimits -Profile \"{}\"", Profiles[Pf]));
                if (ImGui::IsItemHovered()) xeditor::hint::Text(bOn ? "The preset the joints without an override take, pinned when limits were turned on: a newer preset never changes this skeleton by itself. Changing it says how many joints change (one undo step)." : "Limits are off: the preview follows the newest preset. Turning limits on pins it.");
            }
            ImGui::Text("Preset %d   Overrides %d   Reviewed %d", m_JointRes.m_nPreset, m_JointRes.m_nOverride, nReviewed);
            static constexpr const char* Filters[] = { "Selected", "Chain", "Needs attention", "All" };
            int F = int(m_JFilter);
            ImGui::SetNextItemWidth(160);
            if (ImGui::Combo("Show", &F, Filters, 4)) m_JFilter = joint_filter(F);
            if (ImGui::IsItemHovered()) xeditor::hint::Text("Which joints draw their range: the selected one only, its chain, the ones that need attention, or all. Tags: R reviewed, * changed since its review, ? frame to confirm, ! outside its limit.");

            // Analyze clips (J4): a proposal from what the packages on this skeleton do - nothing changes until Apply
            if (ImGui::Button("Analyze clips...")) { FitLoadSources(); m_FitRows.clear(); m_bFitOpen = true; }
            if (ImGui::IsItemHovered()) xeditor::hint::Text("Looks at what the animation packages on this skeleton do with each joint and proposes wider limits where the clips go further. It only proposes: nothing changes until you Apply.");
            if (m_bFitOpen && ImGui::CollapsingHeader("Analyze clips###fit", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (m_FitSources.empty()) ImGui::TextDisabled("No package on this skeleton has evidence yet: compile a package that uses it.");
                for (std::size_t s = 0; s < m_FitSources.size(); ++s)
                {
                    auto& S = m_FitSources[s];
                    int nF = 0; std::string Clips; for (const auto& C : S.m_E.m_Clips) { nF += C.m_nFrames; Clips += (Clips.empty() ? "" : ", ") + C.m_Name; }
                    ImGui::BeginDisabled(S.m_bStale);
                    ImGui::Checkbox(std::format("{}: {} clip(s), {} frames{}###src{}", S.m_Package, S.m_E.m_Clips.size(), nF, S.m_bStale ? " (stale: recompile it)" : "", s).c_str(), &S.m_bUse);
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered()) xeditor::hint::Text(Clips.c_str());
                }
                ImGui::Checkbox("Include reviewed joints", &m_bFitIncludeReviewed);
                if (ImGui::IsItemHovered()) xeditor::hint::Text("Reviewed joints are protected by default: the clips may be wrong (a bad retarget) and widening would hide it.");
                ImGui::SameLine();
                if (ImGui::Button("Analyze")) m_JointMsg = FitAnalyze();
                if (!m_FitRows.empty())
                {
                    ImGui::Text("%d frames; %d free joint(s) have nothing to widen", m_FitFrames, m_FitFree);
                    // one block per joint (the panel is narrow: a table squeezed its text to one letter a line)
                    for (std::size_t r = 0; r < m_FitRows.size(); ++r)
                    {
                        auto& Rw = m_FitRows[r];
                        ImGui::PushID(int(r));
                        ImGui::Separator();
                        ImGui::BeginDisabled(Rw.m_bProtected || !Rw.m_bChanges); ImGui::Checkbox("##sel", &Rw.m_bSelected); ImGui::EndDisabled();
                        ImGui::SameLine(); if (ImGui::Selectable(m_JNames[Rw.m_Bone].c_str(), SelectedJoint() == Rw.m_Bone, 0, ImVec2(ImGui::GetContentRegionAvail().x - 60, 0))) m_Selected = { Rw.m_Bone };
                        if (Rw.m_iSrc >= 0) { ImGui::SameLine(); if (ImGui::SmallButton("Worst")) m_JointMsg = FitShowWorst(m_JNames[Rw.m_Bone]); if (ImGui::IsItemHovered()) xeditor::hint::Text("Shows the clip frame furthest outside the current limit as the test pose."); }
                        ImGui::TextWrapped("now:  %s", xskeleton_joints::Describe(m_JointRes.m_Limits[Rw.m_Bone]).c_str());
                        if (Rw.m_bChanges) ImGui::TextWrapped("to:   %s", xskeleton_joints::Describe(Rw.m_New).c_str());
                        else ImGui::TextDisabled("no change: only samples the %s cannot hold - check its frame (Snap to clips)", m_JointRes.m_Limits[Rw.m_Bone].m_Type == joint_type::HINGE ? "hinge" : "limit");
                        ImGui::TextDisabled("outside %d of %d (worst %.1f deg), %d distinct poses%s", Rw.m_Rep.m_nOutside, Rw.m_Rep.m_nSamples, Rw.m_Rep.m_WorstExcess / xskeleton_joints::deg_v, Rw.m_nCoverage, Rw.m_bProtected ? ", reviewed: protected" : "");
                        if (Rw.m_Rep.m_nOffModel) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "%d sample(s) the %s cannot hold (never promoted)", Rw.m_Rep.m_nOffModel, m_JointRes.m_Limits[Rw.m_Bone].m_Type == joint_type::HINGE ? "hinge" : "limit");
                        ImGui::PopID();
                    }
                    ImGui::Separator();
                    if (ImGui::Button("Apply selected")) m_JointMsg = FitApply();
                    if (ImGui::IsItemHovered()) xeditor::hint::Text("The selected joints take the proposed limits (marked Fitted, with the packages they came from) as one undo step.");
                }
            }

            // the joints that need attention: click to select; a frame can be confirmed from here
            const auto Att = JointAttention();
            if (!m_JointRes.m_Errors.empty()) for (const auto& E : m_JointRes.m_Errors) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "%s", E.c_str());
            if (!Att.empty() && ImGui::CollapsingHeader(std::format("Needs attention ({})###att", Att.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                for (std::size_t i = 0; i < Att.size(); ++i)
                {
                    ImGui::PushID(int(i));
                    if (ImGui::Selectable(std::format("{}  - {}", m_JNames[Att[i].m_Bone], Att[i].m_Why).c_str(), SelectedJoint() == Att[i].m_Bone, 0, ImVec2(ImGui::GetContentRegionAvail().x - 70, 0)))
                    { m_Selected = { Att[i].m_Bone }; m_iScrollTo = Att[i].m_Bone; }
                    if (Att[i].m_bFrame)
                    {
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Confirm"))
                        {
                            const auto& Fq = m_JointRes.m_Limits[Att[i].m_Bone].m_Frame;
                            JointCommand(std::format("JointSet -Bone {} -Frame \"{} {} {} {}\"", xeditor::Quote(m_JNames[Att[i].m_Bone]), Fq.m_X, Fq.m_Y, Fq.m_Z, Fq.m_W));
                        }
                        if (ImGui::IsItemHovered()) xeditor::hint::Text("The frame drawn (x red along the bone, z blue the hinge axis) is right: keep it as is.");
                    }
                    ImGui::PopID();
                }
            }
            if (m_bTestPose)
            {
                ImGui::TextColored(ImVec4(1, 0.85f, 0.5f, 1), "Preview pose - not saved");
                ImGui::SameLine();
                if (ImGui::SmallButton("Reset pose")) { m_bTestPose = false; m_PullNote.clear(); }
            }
            ImGui::Separator();

            const int b = SelectedJoint();
            if (b < 0 || b >= int(m_JNames.size())) { ImGui::TextDisabled("Select one bone to see and edit its joint."); return; }
            const auto& L = m_JointRes.m_Limits[b];
            const auto& Name = m_JNames[b];
            std::vector<std::string_view> Views(m_JNames.begin(), m_JNames.end());
            const int Other = FindMirrorBone(Views, b);
            ImGui::Text("%s", Name.c_str());
            if (Other >= 0 && Other != b) { ImGui::SameLine(); ImGui::TextDisabled("<-> %s", m_JNames[Other].c_str()); ImGui::SameLine(); ImGui::Checkbox("Symmetry", &m_bJointSymmetry); }
            const bool bSym = m_bJointSymmetry && Other >= 0 && Other != b;
            const std::string Mir = bSym ? " -Mirror 1" : "";
            const xskeleton_desc::joint_override* pO = pDesc->findJointOverride(Name);
            ImGui::TextDisabled("Source: %s", m_JointRes.m_Source[b].c_str());
            ImGui::TextDisabled("Review: %s", !pO ? "not reviewed (resolved from the profile)" : pO->m_Review == xskeleton_desc::joint_review::REVIEWED ? "reviewed"
                                              : pO->m_Review == xskeleton_desc::joint_review::NEEDS_REVIEW ? "needs review (changed since)" : "pending");

            static constexpr const char* Geos[] = { "Free", "Fixed", "Hinge", "Ball" };
            int Geo = int(L.m_Type);
            if (ImGui::Combo("Geometry", &Geo, Geos, 4) && Geo != int(L.m_Type)) JointCommand(std::format("JointSet -Bone {} -Geometry {}{}", xeditor::Quote(Name), Geos[Geo], Mir));

            // the numbers (degrees), kept while one is edited and sent as one JointSet when the edit ends
            constexpr float D = 57.29578f;
            if (m_JEditBone != b || !ImGui::IsAnyItemActive())
            {
                m_JEditBone = b;
                m_JEdit[0] = L.m_HingeMin * D; m_JEdit[1] = L.m_HingeMax * D;
                m_JEdit[2] = L.m_SwingCy * D; m_JEdit[3] = L.m_SwingCz * D; m_JEdit[4] = L.m_SwingRy * D; m_JEdit[5] = L.m_SwingRz * D;
                m_JEdit[6] = L.m_TwistCentre * D; m_JEdit[7] = L.m_TwistHalf * D;
            }
            if (L.m_Type == joint_type::HINGE)
            {
                ImGui::DragFloat2("Range (min, max)", &m_JEdit[0], 0.5f, -180, 180, "%.1f");
                if (ImGui::IsItemDeactivatedAfterEdit()) JointCommand(std::format("JointSet -Bone {} -Hinge \"{} {}\"{}", xeditor::Quote(Name), m_JEdit[0], m_JEdit[1], Mir));
            }
            if (L.m_Type == joint_type::BALL)
            {
                ImGui::DragFloat2("Swing centre (y, z)", &m_JEdit[2], 0.5f, -180, 180, "%.1f");
                const bool b1 = ImGui::IsItemDeactivatedAfterEdit();
                ImGui::DragFloat2("Swing radius (y, z)", &m_JEdit[4], 0.5f, 0, 179, "%.1f");
                const bool b2 = ImGui::IsItemDeactivatedAfterEdit();
                if (b1 || b2) JointCommand(std::format("JointSet -Bone {} -Swing \"{} {} {} {}\"{}", xeditor::Quote(Name), m_JEdit[2], m_JEdit[3], m_JEdit[4], m_JEdit[5], Mir));
                ImGui::DragFloat2("Twist (centre, half)", &m_JEdit[6], 0.5f, -180, 180, "%.1f");
                if (ImGui::IsItemDeactivatedAfterEdit()) JointCommand(std::format("JointSet -Bone {} -Twist \"{} {}\"{}", xeditor::Quote(Name), m_JEdit[6], m_JEdit[7], Mir));
                static constexpr const char* Axes[] = { "Free", "Limited", "Locked" };
                int A[3] = { int(L.m_SwingY), int(L.m_SwingZ), int(L.m_Twist) };
                bool bAx = false;
                ImGui::SetNextItemWidth(90); bAx |= ImGui::Combo("##sy", &A[0], Axes, 3); ImGui::SameLine();
                ImGui::SetNextItemWidth(90); bAx |= ImGui::Combo("##sz", &A[1], Axes, 3); ImGui::SameLine();
                ImGui::SetNextItemWidth(90); bAx |= ImGui::Combo("Swing y, z, twist", &A[2], Axes, 3);
                if (bAx) JointCommand(std::format("JointSet -Bone {} -Axes \"{} {} {}\"{}", xeditor::Quote(Name), Axes[A[0]], Axes[A[1]], Axes[A[2]], Mir));
            }

            // Include this pose
            ImGui::Separator();
            if (!m_bJInclude || m_JIncludeBone != b)
            {
                if (ImGui::Button("Include a pose...")) { m_bJInclude = true; m_bJIncludeBall = false; m_JIncludeBone = b; m_bTestPose = false; m_PullNote.clear(); }
                if (ImGui::IsItemHovered()) xeditor::hint::Text("Pull the chain (orange handle) into the pose this joint should allow: only this joint's limit is lifted. The limit then grows just enough to include it; Include or Cancel.");
            }
            else
            {
                ImGui::TextColored(ImVec4(1, 0.85f, 0.4f, 1), "Include a pose: pull the orange handle");
                if (m_bJIncludeValid)
                {
                    ImGui::TextWrapped("Proposed: %s", xskeleton_joints::Describe(m_JIncludeProposal).c_str());
                    if (m_JIncludeRep.m_nOffModel && L.m_Type == joint_type::HINGE && !m_bJIncludeBall)
                    {
                        ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "This pose bends sideways: a hinge cannot hold it.");
                        if (ImGui::SmallButton("Make it a ball")) m_bJIncludeBall = true;
                        if (ImGui::IsItemHovered()) xeditor::hint::Text("A separate, explicit change: the joint becomes a ball (swing in two axes and twist) fitted to its hinge range and this pose.");
                    }
                }
                ImGui::BeginDisabled(!m_bJIncludeValid);
                if (ImGui::Button("Include")) IncludeAccept(bSym);
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button("Cancel##include")) { m_bJInclude = m_bJIncludeBall = false; m_bTestPose = false; m_PullNote.clear(); m_JointKey.clear(); }
            }

            // the frame
            ImGui::Separator();
            {
                const bool bNeedsFrame = m_JointRes.m_Source[b].find("frame needs confirmation") != std::string::npos;
                ImGui::Text("Frame: %s", bNeedsFrame ? "needs confirmation" : (pO && pO->m_bFrameSet) ? "confirmed" : "suggested");
                if (ImGui::IsItemHovered()) xeditor::hint::Text("x (red) runs along the bone, z (blue) is the axis a hinge bends about. If they look wrong, roll the frame or snap it to a bent test pose.");
                if (bNeedsFrame) { ImGui::SameLine(); if (ImGui::SmallButton("Confirm frame")) { const auto& Fq = L.m_Frame; JointCommand(std::format("JointSet -Bone {} -Frame \"{} {} {} {}\"{}", xeditor::Quote(Name), Fq.m_X, Fq.m_Y, Fq.m_Z, Fq.m_W, Mir)); } }
                ImGui::SetNextItemWidth(160);
                if (ImGui::DragFloat("Roll frame (deg)", &m_JFrameRoll, 0.5f, -180, 180, "%.1f"))
                { m_bJFramePending = true; m_JFrameBone = b; m_JFrameNew = (L.m_Frame * details::AxisAngle(details::AxX, m_JFrameRoll / D)).NormalizeCopy(); }
                ImGui::SameLine();
                if (ImGui::SmallButton("Snap to the pose's bend"))
                {
                    xmath::fquat Fq;
                    if (FrameFromPoseBend(b, Fq)) { m_bJFramePending = true; m_JFrameBone = b; m_JFrameNew = Fq; m_JointMsg.clear(); }
                    else m_JointMsg = "Snap to the pose's bend: pull the chain so this joint is clearly bent first (more than 3 degrees)";
                }
                if (ImGui::IsItemHovered()) xeditor::hint::Text("Bend this joint with a test pull, then snap: the hinge axis (z) becomes the axis the pose bends about.");
                ImGui::SameLine();
                if (ImGui::SmallButton("Snap to clips"))
                {
                    xmath::fquat Fq;
                    if (FrameFromClips(b, Fq)) { m_bJFramePending = true; m_JFrameBone = b; m_JFrameNew = Fq; m_JointMsg.clear(); }
                    else m_JointMsg = "Snap to clips: no package on this skeleton moves this joint enough to say its bend axis";
                }
                if (ImGui::IsItemHovered()) xeditor::hint::Text("The hinge axis (z) becomes the axis this joint bends about in the clips of the packages on this skeleton.");
                if (m_bJFramePending && m_JFrameBone == b)
                {
                    ImGui::TextWrapped("New frame: the orange outline turns the range with the frame, the dotted cyan one keeps the range where it is.");
                    if (ImGui::Button("Turn the range with it")) ApplyFrame(false, bSym);
                    ImGui::SameLine(); if (ImGui::Button("Keep the range")) ApplyFrame(true, bSym);
                    ImGui::SameLine(); if (ImGui::Button("Cancel##frame")) { m_bJFramePending = false; m_JFrameRoll = 0; }
                }
            }

            // review
            ImGui::Separator();
            if (m_JCheckT == -2 && m_JCheckBone == b)
            {
                ImGui::TextColored(ImVec4(0.6f, 0.9f, 1, 1), "Did it look right?");
                if (ImGui::Button("Yes - mark reviewed")) { JointCommand(std::format("JointReview -Bone {}{} -By User", xeditor::Quote(Name), Mir)); m_JCheckT = -1; m_JCheckLabel.clear(); }
                ImGui::SameLine(); if (ImGui::Button("Not yet")) { m_JCheckT = -1; m_JCheckLabel.clear(); }
            }
            else
            {
                if (ImGui::Button(m_JCheckT >= 0 ? "Checking..." : "Check this joint")) { if (m_JCheckT < 0) { m_JCheckT = 0; m_JCheckBone = b; } }
                if (ImGui::IsItemHovered()) xeditor::hint::Text("Moves the joint through its whole range (parent held): bend, extend, twist, then shows the other side; then asks.");
                ImGui::SameLine();
                if (ImGui::Button("Mark reviewed")) JointCommand(std::format("JointReview -Bone {}{} -By User", xeditor::Quote(Name), Mir));
                if (ImGui::IsItemHovered()) xeditor::hint::Text("You looked at this joint (bend it, twist it, check the other side): its values are recorded as reviewed. A later change asks for a review again.");
            }
            if (ImGui::Button("Reset to Auto")) JointCommand(std::format("JointReset -Bone {}{}", xeditor::Quote(Name), Mir));
            if (Other >= 0 && Other != b) { ImGui::SameLine(); if (ImGui::Button("Mirror to other side")) MirrorJoint(Name); }
            if (!m_JointMsg.empty()) ImGui::TextWrapped("%s", m_JointMsg.c_str());
        }

        // Names over the bones: the hovered and the selected ones, or all of them
        void RenderLabels(const xskeleton::skeleton& Skeleton, const ImVec2& Min, const ImVec2& Size, const ImVec2& Mouse) noexcept
        {
            auto* pDraw = ImGui::GetWindowDrawList();
            auto Project = [&](const xmath::fvec3& P, ImVec2& Out)
            {
                const xmath::fvec4 Clip = m_Scene.m_View.getW2C() * xmath::fvec4(P, 1.0f);
                if (Clip.m_W <= 1.0e-4f) return false;
                // NOT "1.0f - NDC.y": getV2CScales() (feeding into getW2C()'s own projection) already
                // flips Y for this engine's Vulkan convention so NDC comes out Y-up - flipping again here
                // double-inverts it, the same bug the original example's own getC2S() carries an explicit
                // comment warning against. Confirmed live against the original: this put high-world-Y
                // bones' labels at the bottom of the screen instead of the top.
                Out = ImVec2(Min.x + (Clip.m_X / Clip.m_W * 0.5f + 0.5f) * Size.x, Min.y + (Clip.m_Y / Clip.m_W * 0.5f + 0.5f) * Size.y);
                return true;
            };
            const auto Bones = Skeleton.getBones();
            const int nBones = std::min<int>(int(Bones.size()), 200);
            for (int i = 0; i < nBones; ++i)
            {
                const bool bShow = m_Settings.m_bAlwaysShowNames || m_Selected.count(i) || i == m_iHovered;
                if (!bShow) continue;
                // The bone's own wedge is drawn as the segment from ITS PARENT's joint to its own joint
                // (see BuildWedgeGeometry's A/B, xskeleton_editor_view.h:600-601) - labeling at either
                // endpoint (this bone's own position, or its parent's) reads as ambiguous about which
                // segment the name belongs to when several bones meet at a joint. The midpoint of that
                // same segment is unambiguous. A parentless (root) bone has no segment of its own (it
                // only gets a marker, not a wedge - see the "only parentless bones get a marker" comment
                // nearby) so it falls back to its own joint position.
                const int  iParent = Bones[i].m_iParent;
                const auto Anchor  = iParent < 0 ? m_Scaled[i].m_Position : (m_Scaled[iParent].m_Position + m_Scaled[i].m_Position) * 0.5f;
                ImVec2 Screen;
                if (!Project(Anchor, Screen)) continue;
                const auto Name = EffectiveName(Skeleton, i);
                const bool bStrong = m_Selected.count(i) || i == m_iHovered;
                pDraw->AddText(ImVec2(Screen.x + 6, Screen.y - 6), bStrong ? IM_COL32(255, 255, 255, 255) : IM_COL32(220, 220, 220, 150), Name.c_str());
            }
            (void)Mouse;
        }
    };

    inline const xeditor::auto_register_resource_editor g_Registration
    { xskeleton_desc::resource_type_guid_v
    , [](xresource::full_guid Guid, xresource_editor::library::guid LibraryGuid, xgpu::device* pDevice) -> std::unique_ptr<xeditor::resource_editor>
      { return std::make_unique<session>(Guid, LibraryGuid, pDevice); }
    };

    inline const xeditor::auto_register_thumbnail_renderer g_ThumbReg{ xrsc::skeleton_type_guid_v, []{ return std::make_unique<thumbnail_renderer>(); } };
}

#endif // XSKELETON_EDITOR_H
