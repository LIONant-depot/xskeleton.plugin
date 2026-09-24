#ifndef XSKELETON_EDITOR_H
#define XSKELETON_EDITOR_H
#pragma once

// The Skeleton editor: opens a skeleton resource from the asset browser in its own window. The bones are drawn as octahedral wedges (an outline
// and a translucent fill that casts a shadow on the ground); a click in the view, or in the bone hierarchy, selects bones. What the compiler does
// with each bone (a new name, virtual, deleted, exposed as a socket, its LOD, its weight in each mask layer) is edited in the hierarchy or with
// commands, all undoable. Hosts include this header and open editors through xeditor::open_resource_editors.
#include "source/Tools/Editor/xeditor_descriptor_editor.h"
#include "dependencies/xresource_pipeline_v2/source/editor/E10_Resources.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_scene.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_editor_runtime.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_thumbnail.h"
#include "plugins/xskeleton.plugin/source/xskeleton_descriptor.h"

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
        pose_mode           m_Pose              = pose_mode::FROZEN;
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
            Out = SplitLines(xeditor::Base64Decode(Text));
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
            : edit_cmd(System, Doc, Details, "SetBone", "Edits what the compiler does with bones (undoable). Usage: SetBone -Bones base64(one raw name per line) [-Rename base64] [-Virtual true|false] [-Delete true|false] [-Expose true|false] [-LOD level]")
        { RegisterArguments(); }

        void RegisterArguments() noexcept override
        {
            m_hBones   = m_Parser.addOption("Bones",   "Raw import names, one per line, base64",     true,  1);
            m_hRename  = m_Parser.addOption("Rename",  "The compiled name, base64 (empty: none)",     false, 1);
            m_hVirtual = m_Parser.addOption("Virtual", "true: a virtual bone, false: a normal one",   false, 1);
            m_hDelete  = m_Parser.addOption("Delete",  "true: leave the bone out of the skeleton",    false, 1);
            m_hExpose  = m_Parser.addOption("Expose",  "true: the bone is a socket",                  false, 1);
            m_hLOD     = m_Parser.addOption("LOD",     "The highest LOD the bone is active at",       false, 1);
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

            std::string Rename, Virtual, Delete, Expose, LOD;
            const bool bRename = xeditor::cmd_util::GetArg(m_Parser, m_hRename, Rename), bVirtual = xeditor::cmd_util::GetArg(m_Parser, m_hVirtual, Virtual);
            const bool bDelete = xeditor::cmd_util::GetArg(m_Parser, m_hDelete, Delete), bExpose = xeditor::cmd_util::GetArg(m_Parser, m_hExpose, Expose), bLOD = xeditor::cmd_util::GetArg(m_Parser, m_hLOD, LOD);
            if (!bRename && !bVirtual && !bDelete && !bExpose && !bLOD) return "SetBone: nothing to change";

            bool bV = false, bD = false, bE = false;
            int iLOD = 0;
            if ((bVirtual && !Bool(Virtual, bV)) || (bDelete && !Bool(Delete, bD)) || (bExpose && !Bool(Expose, bE))) return "SetBone: Virtual, Delete and Expose take true or false";
            if (bLOD && (std::from_chars(LOD.data(), LOD.data() + LOD.size(), iLOD).ec != std::errc() || iLOD < 0)) return "SetBone: LOD takes a level from 0";

            auto& D = Desc();
            for (auto& Name : Names)
            {
                auto& B = FindOrCreateBoneOverride(D, Name);
                if (bRename)  B.m_Rename      = xeditor::Base64Decode(Rename);
                if (bVirtual) B.m_Type        = bV ? xskeleton_desc::bone_type::VIRTUAL : xskeleton_desc::bone_type::NORMAL;
                if (bDelete)  B.m_bDeleteBone = bD;
                if (bExpose)  B.m_bExpose     = bE;
                if (bLOD)
                {
                    B.m_LODLevel = iLOD;
                    if (auto* pNode = FindDetailsBone(m_Details.m_RootBone, Name)) ClampDescendantLODs(D, *pNode, iLOD);
                }
            }
            m_Doc.m_bDirty = true;
            return {};
        }

        xcmdline::parser::handle m_hRename, m_hVirtual, m_hDelete, m_hExpose, m_hLOD;
    };

    // MatchLOD: every descendant of the named bones gets exactly the bone's own LOD
    struct match_lod_cmd : edit_cmd
    {
        match_lod_cmd(xundo::system& System, xeditor::descriptor_document& Doc, const xskeleton_desc::details& Details) noexcept
            : edit_cmd(System, Doc, Details, "MatchLOD", "Gives every descendant of the bones the bone's own LOD (undoable). Usage: MatchLOD -Bones base64(one raw name per line)")
        { RegisterArguments(); }
        void RegisterArguments() noexcept override { m_hBones = m_Parser.addOption("Bones", "Raw import names, one per line, base64", true, 1); }

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
            : edit_cmd(System, Doc, Details, "SetMaskWeight", "Sets the weight (0 to 1) of bones in a mask layer (undoable). Usage: SetMaskWeight -Layer base64(name) -Bones base64(one raw name per line) -Weight w [-Children true]")
        { RegisterArguments(); }
        void RegisterArguments() noexcept override
        {
            m_hLayer    = m_Parser.addOption("Layer",    "The mask layer's name, base64",             true,  1);
            m_hBones    = m_Parser.addOption("Bones",    "Raw import names, one per line, base64",     true,  1);
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
            Layer = xeditor::Base64Decode(Layer);
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
                    m_hBones = m_Parser.addOption("Bones", "Raw import names, one per line, base64", true, 1);
                    m_hAdd   = m_Parser.addOption("Add",   "true: keep the current selection and add to it", false, 1);
                }
            }
            std::string Query() noexcept override { return m_Session.RunQuery(*this); }
            xcmdline::parser::handle m_hBones, m_hAdd;
        };

        xskeleton_desc::details                                 m_Details;              // the imported hierarchy, from the compiler's log
        set_bone_cmd                                            m_SetBone;
        match_lod_cmd                                           m_MatchLOD;
        set_mask_weight_cmd                                     m_SetMaskWeight;
        xeditor::set_preview_cmd<view_settings>                 m_SetPreview;
        xeditor::list_preview_cmd<view_settings>                m_ListPreview;
        query_cmd                                               m_ListBones, m_SelectBones, m_ClearSelection, m_ListLayers;

        runtime                                                 m_Scene;
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
        int                                                     m_iAnchor = -1;
        int                                                     m_iHovered = -1;
        bool                                                    m_bTreeHovered = false;
        std::string                                             m_RenamingBone, m_RenameBuffer;
        bool                                                    m_bRenameStarted = false;

        std::vector<e19::draw_vert>                             m_Outline, m_Fill;

        session(xresource::full_guid Guid, e10::library::guid LibraryGuid, xgpu::device* pDevice) noexcept
            : descriptor_editor("Skeleton", Guid, LibraryGuid, pDevice)
            , m_SetBone(m_Undo, m_Document, m_Details), m_MatchLOD(m_Undo, m_Document, m_Details), m_SetMaskWeight(m_Undo, m_Document, m_Details)
            , m_SetPreview(m_Undo, m_Settings), m_ListPreview(m_Undo, m_Settings)
            , m_CameraCmds(m_Undo, m_Scene.Camera())
            , m_ListBones      (m_Undo, *this, query_cmd::kind::list_bones,      "ListBones",      "Every compiled bone: index, name, raw name, parent, flags, LOD and its override. Usage: ListBones")
            , m_SelectBones    (m_Undo, *this, query_cmd::kind::select_bones,    "SelectBones",    "Selects bones in the view. Usage: SelectBones -Bones base64(one raw name per line) [-Add true]")
            , m_ClearSelection (m_Undo, *this, query_cmd::kind::clear_selection, "ClearSelection", "Selects no bone. Usage: ClearSelection")
            , m_ListLayers     (m_Undo, *this, query_cmd::kind::list_layers,     "ListLayers",     "The mask layers and how many bones each weights. Usage: ListLayers")
        {
            m_SettingsInspector.BindObject(*xproperty::getObjectByType<view_settings>(), &m_Settings);
            m_bLastNormalize = m_Settings.m_bNormalizeSize;

            AddPanel("Skeleton View",      dock::left,   [this] { m_SettingsInspector.Show(); });
            AddPanel("Skeleton Viewport",  dock::center, [this] { RenderViewport(); }, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            AddPanel("Bone Hierarchy",     dock::right,  [this] { RenderHierarchy(); });
            AddPanel("Description",        dock::right,  [this] { m_DescriptorInspector.Show(); });

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

            auto* pDesc = Desc();
            if (!pDesc) { m_ErrorMessage = "The descriptor could not be read."; return; }

            m_Details = {};
            xtextfile::stream File;
            if (!File.Open(true, m_Document.m_LogPath + L"\\Details.txt", {}))
            {
                xproperty::settings::context Context;
                if (xproperty::sprop::serializer::Stream(File, m_Details, Context)) m_Details = {};
            }
            const auto Before = m_Document.Snapshot();
            pDesc->MergeWithDetails(m_Details);
            if (m_Document.Snapshot() != Before) m_Document.m_bDirty = true;
            RebuildNames();

            if (!std::filesystem::exists(m_Document.m_ResourcePath)) { m_ErrorMessage = "Not compiled yet: compile the skeleton to see it."; return; }
            m_Ref.m_Instance = m_Document.m_Guid.m_Instance;
            auto* pSkeleton = xresource::g_Mgr.getResource(m_Ref);
            if (!pSkeleton) { m_Ref.clear(); m_ErrorMessage = "The compiled skeleton could not be loaded."; return; }

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
            for (std::size_t i = 0; i < Bones.size(); ++i)
            {
                auto& W = m_WorldBind[i];
                if (!Bones[i].m_InvBindPose.isIdentity())
                {
                    Mats[i] = Bones[i].m_InvBindPose.Inverse();
                    W.m_bRealBindData = true;
                }
                else
                {
                    const xmath::fmat4 Local = Rests[i].m_RestPose.toMatrix();
                    Mats[i] = Bones[i].m_iParent < 0 ? Local : Mats[Bones[i].m_iParent] * Local;
                    W.m_bRealBindData = false;
                }
                W.m_Position = Mats[i].ExtractPosition(); W.m_Right = Mats[i].Right(); W.m_Up = Mats[i].Up();
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
            return xeditor::Base64Encode(Text);
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
                for (auto& Name : SplitLines(xeditor::Base64Decode(Text)))
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

            if (bDelete)       ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 90, 90, 255));
            else if (bVirtual) ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 180, 84, 255));
            std::string Label = Name;
            if (pOverride && pOverride->m_LODLevel > 0) Label += std::format("  [LOD{}]", pOverride->m_LODLevel);
            if (iBone != -1) if (const int Eff = GetBoneLODLevel(Skeleton, iBone); Eff != (pOverride ? pOverride->m_LODLevel : 0)) Label += std::format("  (compiled: LOD{})", Eff);
            if (bDelete) Label += "  (pending delete)";
            const bool bOpen = ImGui::TreeNodeEx(Raw.c_str(), Flags, "%s", bRenaming ? "" : Label.c_str());
            if (bDelete || bVirtual) ImGui::PopStyleColor();

            if (ImGui::IsItemHovered()) { m_iHovered = iBone; m_bTreeHovered = true; }

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
                    Pending = std::format("SetBone -Bones {} -Rename {}", xeditor::Base64Encode(Raw), xeditor::Base64Encode(std::string(Buffer)));
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
                if (ImGui::MenuItem("Match Children's LOD to This Bone")) Pending = std::format("MatchLOD -Bones {}", Targets);
                if (iLayer >= 0 && ImGui::MenuItem("Match Children's Active Layer Weight to This Bone"))
                    Pending = std::format("SetMaskWeight -Layer {} -Bones {} -Weight {} -Children true", xeditor::Base64Encode(D.m_MaskGroups[iLayer].m_Name), Targets, GetMaskWeight(D, iLayer, Raw));
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
                    Pending = std::format("SetMaskWeight -Layer {} -Bones {} -Weight {}", xeditor::Base64Encode(D.m_MaskGroups[iLayer].m_Name), EditTargets(Skeleton, iBone, Raw), std::clamp(Weight, 0.0f, 1.0f));
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
            auto* pHost   = xeditor::host::current();
            auto* pWindow = pHost ? pHost->find<xgpu::window>() : nullptr;
            const ImVec2 Avail = ImGui::GetContentRegionAvail();
            const ImVec2 Min   = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(Min, ImVec2(Min.x + Avail.x, Min.y + Avail.y), IM_COL32(115, 115, 115, 255));
            ImGui::InvisibleButton("##SkeletonViewport", Avail, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            const bool bHovered = ImGui::IsItemHovered();
            m_Scene.HandleInput();

            auto* pSkeleton = Skeleton();
            if (!m_bReady || !pWindow) { ImGui::SetCursorScreenPos(Min); ImGui::TextDisabled("The 3D view needs a GPU device (open from E29)."); return; }
            if (!pSkeleton) { ImGui::SetCursorScreenPos(Min); ImGui::TextWrapped("%s", m_ErrorMessage.empty() ? "Nothing to show." : m_ErrorMessage.c_str()); return; }

            // "Resize skeleton to 1m" scales the view only, never the asset
            if (m_Settings.m_bNormalizeSize != m_bLastNormalize) { m_bLastNormalize = m_Settings.m_bNormalizeSize; m_Scene.m_bReframe = true; }
            const float Scale = ViewScale();
            m_Scene.m_Radius = m_Radius * Scale;
            m_Scene.m_Center = m_Center * Scale;
            const auto& Src = m_Settings.m_Pose == pose_mode::BIND ? m_WorldBind : m_WorldFrozen;
            m_Scaled = Src;
            for (auto& W : m_Scaled) W.m_Position = W.m_Position * Scale;
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
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
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
            m_Scene.SetLines(m_Outline);
            m_Scene.SetFill(m_Fill);
            m_Scene.RenderShadow(*pWindow);

            xgpu::tools::imgui::AddCustomRenderCallback([this](xgpu::cmd_buffer& CmdBuffer, const ImVec2&, const ImVec2&)
            {
                if (m_bOpen) m_Scene.DrawBones(CmdBuffer);
            });

            RenderLabels(*pSkeleton, Min, Avail, Mouse);
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
    , [](xresource::full_guid Guid, e10::library::guid LibraryGuid, xgpu::device* pDevice) -> std::unique_ptr<xeditor::resource_editor>
      { return std::make_unique<session>(Guid, LibraryGuid, pDevice); }
    };

    inline const xeditor::auto_register_thumbnail_renderer g_ThumbReg{ xrsc::skeleton_type_guid_v, []{ return std::make_unique<thumbnail_renderer>(); } };
}

#endif // XSKELETON_EDITOR_H
