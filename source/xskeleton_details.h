#ifndef XSKELETON_DETAILS_H
#define XSKELETON_DETAILS_H
#pragma once

namespace xskeleton_desc
{
    // Read-only, editor-facing view of what was actually imported from the source asset (before the
    // user's edits in xskeleton_desc::descriptor - deletions, exposed sockets, LOD levels, masks, etc.
    // are layered on top of this). None of this survives into the compiled runtime skeleton.
    struct details
    {
        struct bone
        {
            std::string          m_Name      = {};
            std::vector<bone>    m_Children  = {};

            XPROPERTY_DEF
            ( "bone", bone
            , obj_member<"Name",        &bone::m_Name >
            , obj_member<"Children",    &bone::m_Children >
            )
        };

        int findBone(std::string_view Name) const noexcept
        {
            for (auto& E : m_BoneList)
                if (E == Name) return static_cast<int>(&E - m_BoneList.data());
            return -1;
        }

        std::vector<std::string>    m_BoneList  = {};    // Flat list of every imported bone name, in import order.
        int                         m_NumBones  = 0;
        bone                        m_RootBone  = {};    // The imported hierarchy, as a tree, for display.
        bool                        m_bCompactTree = false;  // the tree is too deep for xproperty's paths: m_RootBone is empty here, the tree follows as the "BoneTree" table

        // xproperty builds each property path in a 256 char buffer, so deep trees are kept out of it
        static constexpr int        compact_depth_v = 8;

        static int Depth(const bone& B) noexcept
        {
            int d = 0;
            for (auto& C : B.m_Children) d = std::max(d, Depth(C));
            return d + 1;
        }

        // Writes (after the xproperty stream) or reads the tree as a flat pre-order table: parent index, name
        xerr StreamCompactTree(xtextfile::stream& File, bone& Tree) noexcept
        {
            struct row { int m_Parent; std::string m_Name; };
            std::vector<row> Rows;
            if (!File.isReading())
            {
                const auto Flatten = [&](auto&& Self, const bone& B, int Parent) -> void
                {
                    const int Me = static_cast<int>(Rows.size());
                    Rows.push_back({ Parent, B.m_Name });
                    for (auto& C : B.m_Children) Self(Self, C, Me);
                };
                Flatten(Flatten, Tree, -1);
            }

            if (auto Err = File.Record("BoneTree"
                , [&](std::size_t& Count, xerr&) { if (File.isReading()) Rows.resize(Count); else Count = Rows.size(); }
                , [&](std::size_t i, xerr& Err)
                {
                    false
                    || (Err = File.Field("Parent", Rows[i].m_Parent))
                    || (Err = File.Field("Name",   Rows[i].m_Name));
                }); Err) return Err;

            if (File.isReading())
            {
                std::vector<bone> Bones(Rows.size());
                for (std::size_t i = 0; i < Rows.size(); ++i) Bones[i].m_Name = Rows[i].m_Name;
                for (int i = static_cast<int>(Rows.size()) - 1; i > 0; --i)    // pre-order: a child always comes after its parent
                    Bones[Rows[i].m_Parent].m_Children.insert(Bones[Rows[i].m_Parent].m_Children.begin(), std::move(Bones[i]));
                if (!Bones.empty()) Tree = std::move(Bones[0]);
            }
            return {};
        }

        XPROPERTY_DEF
        ( "details", details
        , obj_member<"RootBone",        &details::m_RootBone,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"BoneList",        &details::m_BoneList,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"NumBones",        &details::m_NumBones,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"CompactTree",     &details::m_bCompactTree, member_flags<flags::DONT_SHOW> >
        )
    };
    XPROPERTY_REG(details)
    XPROPERTY_REG2(bone_, details::bone)
}

#endif
