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

        XPROPERTY_DEF
        ( "details", details
        , obj_member<"RootBone",        &details::m_RootBone,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"BoneList",        &details::m_BoneList,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"NumBones",        &details::m_NumBones,   member_flags<flags::SHOW_READONLY> >
        )
    };
    XPROPERTY_REG(details)
    XPROPERTY_REG2(bone_, details::bone)
}

#endif
