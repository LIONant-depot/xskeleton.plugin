#ifndef XSKELETON_BONE_TREE_H
#define XSKELETON_BONE_TREE_H
#pragma once
// The bone tree every editor shows (R57/R58: one shared piece, so improving it improves them all): the Skeleton editor's hierarchy rows,
// selection (click, Ctrl, Shift), hover, and the tree following a bone hovered or selected in the 3D view (its parents opened, scrolled to).
// An editor hands in the rows (a name, a parent row, the runtime bone or -1) and its selection; anything of its own on a row (icons,
// rename, menus, more columns) goes in the hooks. Inside an ImGui table the editor began (column 0 is the tree).
#include "dependencies/imgui/imgui.h"
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace xskeleton_editor
{
    struct bone_tree
    {
        struct row { std::string m_Name, m_Label; int m_iParent = -1, m_iBone = -1; ImU32 m_TextColor = 0; };     // Label empty: the name; TextColor 0: the style's

        struct hooks
        {
            std::function<void(int iBone, bool bCtrl, bool bShift)> m_Select;                  // a click on a row with a runtime bone
            std::function<void(int iRow, bool bOpen)>               m_AfterItem;               // right after the tree item (icons, rename, menus, other columns); may be empty
            std::function<bool(int iRow)>                           m_HideLabel;               // the label is drawn by the hook (a rename box); may be empty
        };

        // Call once per frame before Render: brings a bone hovered or newly selected in the 3D view (not in the tree) into view
        void Follow(const std::vector<row>& Rows, const std::set<int>& Selected, int Hovered) noexcept
        {
            int iTarget = -1;
            if (!m_bTreeHovered && Hovered >= 0 && Hovered != m_iLastHover3D) iTarget = Hovered;
            if (Selected != m_LastSelected && !Selected.empty() && !m_bTreeHovered) iTarget = *Selected.rbegin();
            m_iLastHover3D = m_bTreeHovered ? -1 : Hovered;
            m_LastSelected = Selected;
            if (iTarget < 0) return;
            m_iScrollTo = iTarget;
            m_OpenPath.assign(Rows.size(), false);
            int r = -1; for (int i = 0; i < int(Rows.size()); ++i) if (Rows[i].m_iBone == iTarget) { r = i; break; }
            for (int p = r >= 0 ? Rows[r].m_iParent : -1; p >= 0; p = Rows[p].m_iParent) m_OpenPath[p] = true;
        }

        // The rows from row 0's roots down. Hovered is set to the hovered row's bone (-1 when the tree hovers nothing) only while the tree is hovered.
        void Render(const std::vector<row>& Rows, const std::set<int>& Selected, int& Hovered, const hooks& H) noexcept
        {
            m_bTreeHovered = false;
            m_Children.assign(Rows.size(), {});
            m_Roots.clear();
            for (int i = 0; i < int(Rows.size()); ++i) (Rows[i].m_iParent >= 0 ? m_Children[Rows[i].m_iParent] : m_Roots).push_back(i);
            for (int r : m_Roots) Node(Rows, r, Selected, Hovered, H);
        }

        bool m_bTreeHovered = false;    // the mouse is over a row (last Render)

    private:
        void Node(const std::vector<row>& Rows, int r, const std::set<int>& Selected, int& Hovered, const hooks& H) noexcept
        {
            const auto& R = Rows[r];
            const bool bKids = !m_Children[r].empty(), bSelected = R.m_iBone >= 0 && Selected.count(R.m_iBone);
            ImGui::TableNextRow();
            if (R.m_iBone >= 0 && R.m_iBone == Hovered) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_HeaderHovered));
            ImGui::TableSetColumnIndex(0);
            ImGuiTreeNodeFlags Flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_DefaultOpen;
            if (bSelected) Flags |= ImGuiTreeNodeFlags_Selected;
            if (!bKids)    Flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
            if (R.m_TextColor) ImGui::PushStyleColor(ImGuiCol_Text, R.m_TextColor);
            if (m_iScrollTo >= 0 && r < int(m_OpenPath.size()) && m_OpenPath[r]) ImGui::SetNextItemOpen(true);
            const bool bHide = H.m_HideLabel && H.m_HideLabel(r);
            const bool bOpen = ImGui::TreeNodeEx(R.m_Name.c_str(), Flags, "%s", bHide ? "" : (R.m_Label.empty() ? R.m_Name : R.m_Label).c_str());
            if (R.m_iBone >= 0 && R.m_iBone == m_iScrollTo) { if (!ImGui::IsItemVisible()) ImGui::SetScrollHereY(0.5f); m_iScrollTo = -1; }
            if (R.m_TextColor) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) { Hovered = R.m_iBone; m_bTreeHovered = true; }
            if (!bHide && ImGui::IsItemClicked() && R.m_iBone >= 0 && H.m_Select) H.m_Select(R.m_iBone, ImGui::GetIO().KeyCtrl, ImGui::GetIO().KeyShift);
            if (H.m_AfterItem) H.m_AfterItem(r, bOpen);
            if (bKids && bOpen)
            {
                for (int c : m_Children[r]) Node(Rows, c, Selected, Hovered, H);
                ImGui::TreePop();
            }
        }

        int                             m_iScrollTo = -1, m_iLastHover3D = -1;
        std::vector<bool>               m_OpenPath;
        std::set<int>                   m_LastSelected;
        std::vector<std::vector<int>>   m_Children;
        std::vector<int>                m_Roots;
    };
}
#endif
