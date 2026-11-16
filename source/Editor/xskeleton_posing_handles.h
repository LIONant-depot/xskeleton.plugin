#ifndef XSKELETON_POSING_HANDLES_H
#define XSKELETON_POSING_HANDLES_H
#pragma once
// The posing handles every editor draws the same way (the Skeleton Editor's Joints mode, the animation editor's Pose tool): what a handle
// looks like in each state, how near the mouse must be, the pin icon next to an effector. One look, one feel, one place to change it.
#include "dependencies/imgui/imgui.h"
#include <span>
#include <utility>

namespace xskeleton_editor::handles
{
    inline constexpr float pick_radius_v = 10.0f;

    enum class kind { effector, pinned, fk_tip, pull };

    inline bool Near(const ImVec2& A, const ImVec2& B, float R = pick_radius_v) noexcept
    {
        const float dx = A.x - B.x, dy = A.y - B.y;
        return dx * dx + dy * dy < R * R;
    }

    // the nearest of (id, screen) within the pick radius: its id, else -1
    inline int Nearest(std::span<const std::pair<int, ImVec2>> Items, const ImVec2& Mouse, float R = pick_radius_v) noexcept
    {
        int Best = -1; float BestD = R * R;
        for (const auto& [Id, S] : Items) { const float d = (S.x - Mouse.x) * (S.x - Mouse.x) + (S.y - Mouse.y) * (S.y - Mouse.y); if (d < BestD) { BestD = d; Best = Id; } }
        return Best;
    }

    inline void Draw(ImDrawList& DL, const ImVec2& S, kind K, bool bHot, bool bDrag) noexcept
    {
        constexpr ImU32 Edge = IM_COL32(20, 20, 20, 255);
        switch (K)
        {
        case kind::pinned:
        {
            const ImU32 C = bDrag ? IM_COL32(255, 200, 60, 255) : bHot ? IM_COL32(255, 235, 120, 255) : IM_COL32(230, 70, 70, 255);
            DL.AddRectFilled(ImVec2(S.x - 6, S.y - 6), ImVec2(S.x + 6, S.y + 6), C); DL.AddRect(ImVec2(S.x - 6, S.y - 6), ImVec2(S.x + 6, S.y + 6), Edge);
            break;
        }
        case kind::fk_tip:
            DL.AddQuadFilled(ImVec2(S.x, S.y - 7), ImVec2(S.x + 7, S.y), ImVec2(S.x, S.y + 7), ImVec2(S.x - 7, S.y), bHot || bDrag ? IM_COL32(120, 220, 255, 255) : IM_COL32(60, 170, 255, 255));
            break;
        case kind::pull:
        case kind::effector:
        {
            const ImU32 C = bDrag ? IM_COL32(255, 200, 60, 255) : bHot ? IM_COL32(255, 235, 120, 255) : IM_COL32(255, 150, 40, 255);
            const float R = (bHot || bDrag) ? 8.0f : 7.0f;
            DL.AddCircleFilled(S, R, C); DL.AddCircle(S, R, Edge);
            break;
        }
        }
    }

    // The pin icon of an effector handle, up and right of it: a small push pin, filled when pinned. Returns its clickable box.
    inline std::pair<ImVec2, ImVec2> PinIcon(ImDrawList& DL, const ImVec2& S, bool bPinned, bool bHot) noexcept
    {
        const ImVec2 C(S.x + 13, S.y - 13);
        const ImVec2 Min(C.x - 6, C.y - 7), Max(C.x + 6, C.y + 8);
        const ImU32 Col = bPinned ? IM_COL32(230, 70, 70, 255) : bHot ? IM_COL32(255, 255, 255, 255) : IM_COL32(200, 200, 200, 200);
        if (bHot) DL.AddRectFilled(Min, Max, IM_COL32(0, 0, 0, 120), 3.0f);
        DL.AddLine(ImVec2(C.x, C.y), ImVec2(C.x, C.y + 7), Col, 2.0f);                 // the needle
        if (bPinned) DL.AddCircleFilled(ImVec2(C.x, C.y - 2), 4.0f, Col);             // the head
        else         DL.AddCircle(ImVec2(C.x, C.y - 2), 4.0f, Col, 0, 1.5f);
        return { Min, Max };
    }
    inline bool In(const std::pair<ImVec2, ImVec2>& Box, const ImVec2& P) noexcept { return P.x >= Box.first.x && P.x <= Box.second.x && P.y >= Box.first.y && P.y <= Box.second.y; }
}
#endif
