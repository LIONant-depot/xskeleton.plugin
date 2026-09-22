#ifndef XSKELETON_EDITOR_VIEW_H
#define XSKELETON_EDITOR_VIEW_H
#pragma once

// How a skeleton is drawn in the skeleton, animation and skin editors: the world pose of each bone, the octahedral "wedge" bone gizmo (an
// outline, a translucent fill, a root sphere), colouring by type / LOD / mask weight / socket, and the geometry the bone picking draws and
// the ray tests it falls back on. CPU side only: the editors own the GPU objects that draw these vertices.
#include "plugins/xskeleton.plugin/source/xskeleton.h"
#include "source/Examples/E19_MaterialEditor/E19_mesh_manager.h"
#include "dependencies/imgui/imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>
#include <vector>

namespace xskeleton_editor
{
    //---------------------------------------------------------------------------
    // Per-bone world-space info. Two independent poses, both computed once at load time:
    //   FROZEN - forward kinematics through each bone's local m_RestPose. Always valid, since
    //            m_RestPose is populated regardless of skinning.
    //   BIND   - Bones[i].m_InvBindPose.Inverse(). Only meaningful for a bone the compiler actually
    //            saw mesh-skin data for; falls back to that same bone's FROZEN position otherwise
    //            (see ComputeBoneWorldsAndFraming) rather than collapsing to the origin.
    // A real mesh bound in a different configuration than its authored rest pose will show the two
    // arrays disagreeing per-bone - that's the point of keeping both, not a bug to reconcile away.
    //---------------------------------------------------------------------------

    struct bone_world
    {
        xmath::fvec3    m_Position      {};
        xmath::fvec3    m_Right         {};
        xmath::fvec3    m_Up            {};
        bool            m_bRealBindData {true}; // FROZEN: always true. BIND: false where propagated (see above).
    };

    // FROZEN: forward-kinematics pose from each bone's local rest transform - the pose the skeleton
    // resource was authored/animated in. BIND: the pose the mesh was originally skinned against;
    // bones with no real bind data (see the fill/outline dimming) had their position inferred by
    // propagating from the nearest ancestor that has one.
    enum class pose_mode { FROZEN, BIND };

    // What determines each bone's viewport/label color - a single choice rather than two
    // independent booleans, since NORMAL/LOD/mask-weight coloring are mutually exclusive by nature
    // (a bone only has one outline color on screen at a time).
    enum class render_color_mode : std::uint8_t { NORMAL_RENDER, LOD_RENDER, ACTIVE_LAYER_RENDER, EXPOSE_RENDER };
    static constexpr auto render_color_mode_v = std::array
    { xproperty::settings::enum_item("Normal",       render_color_mode::NORMAL_RENDER)
    , xproperty::settings::enum_item("LOD",          render_color_mode::LOD_RENDER)
    , xproperty::settings::enum_item("Active Layer", render_color_mode::ACTIVE_LAYER_RENDER)
    , xproperty::settings::enum_item("Exposed",      render_color_mode::EXPOSE_RENDER)
    };

    //---------------------------------------------------------------------------
    // Wedge geometry - the classic bone gizmo: wide "ring" near the joint, tapering to the child.
    // The ring's cross-section uses the BONE'S OWN world Right()/Up() axes (not the parent's, not
    // a camera-facing basis), matching a real 3D wedge rather than a billboard.
    //---------------------------------------------------------------------------

    struct wedge_shape
    {
        std::array<xmath::fvec3, 4> m_Ring;
    };

    inline bool ComputeWedgeShape(const xmath::fvec3& A, const xmath::fvec3& B, const xmath::fvec3& Right, const xmath::fvec3& Up, wedge_shape& Out)
    {
        xmath::fvec3 Dir = B - A;
        const float  Len = Dir.Length();
        if (Len < 1.0e-5f) return false;
        Dir = Dir / Len;

        // The bone's own Right/Up come straight out of its world matrix, which can carry non-uniform
        // scale or shear from the source rig (twist/helper joints especially) - using them as-is can
        // collapse the ring toward a line, rendering that bone as a flat 2D sliver instead of a solid
        // wedge. Re-orthogonalize against Dir (Gram-Schmidt) so every bone gets a clean, non-degenerate
        // cross-section regardless of how well-formed its source matrix is.
        xmath::fvec3 UpOrtho = Up - Dir * Dir.Dot(Up);
        if (UpOrtho.Length() < 1.0e-4f)
            UpOrtho = (std::abs(Dir.Dot(xmath::fvec3(0, 1, 0))) < 0.99f) ? xmath::fvec3(0, 1, 0) : xmath::fvec3(1, 0, 0);
        UpOrtho.Normalize();
        const xmath::fvec3 RightOrtho = Dir.Cross(UpOrtho).Normalize();
        UpOrtho = RightOrtho.Cross(Dir).Normalize();

        const float HeadWidth = std::max(Len * 0.10f, 1.0e-4f);
        const float HeadDist  = std::min(Len * 0.22f, HeadWidth * 2.4f);
        const xmath::fvec3 Center = A + Dir * HeadDist;

        Out.m_Ring =
        { Center + RightOrtho * HeadWidth
        , Center + UpOrtho    * HeadWidth
        , Center - RightOrtho * HeadWidth
        , Center - UpOrtho    * HeadWidth
        };
        return true;
    }

    //---------------------------------------------------------------------------
    // Root marker - a small sphere gizmo for any parentless bone. A regular wedge is drawn BETWEEN
    // a bone and its parent, so a root (no parent) never gets one and, before this, rendered as
    // nothing at all. Always solid yellow (not virtual/normal color-coded like a regular wedge -
    // this is a landmark, not a limb segment; virtual state is still visible via the tree's checkbox
    // and text color), using the same fill alpha/depth-tint machinery as a regular wedge's fill.
    //---------------------------------------------------------------------------

    struct sphere_frame
    {
        xmath::fvec3    m_Right;
        xmath::fvec3    m_Up;
        xmath::fvec3    m_Forward;
    };

    inline bool ComputeOrthoFrame(const xmath::fvec3& Right, const xmath::fvec3& Up, sphere_frame& Out)
    {
        xmath::fvec3 R = Right;
        xmath::fvec3 U = Up;
        if (R.Length() < 1.0e-5f || U.Length() < 1.0e-5f) return false;
        R.Normalize();
        U.Normalize();

        xmath::fvec3 F = R.Cross(U);
        if (F.Length() < 1.0e-4f) return false;
        F.Normalize();
        U = F.Cross(R).Normalize(); // re-orthogonalize, same reasoning as ComputeWedgeShape

        Out = { R, U, F };
        return true;
    }

    inline xmath::fvec3 SpherePoint(const xmath::fvec3& Center, const sphere_frame& Frame, float Radius, float Theta, float Phi)
    {
        // Phi in [0,pi] sweeps from the +Forward pole to the -Forward pole; Theta in [0,2pi) sweeps
        // around the Forward axis through the Right/Up equatorial plane.
        return Center
             + Frame.m_Forward * (Radius * std::cos(Phi))
             + Frame.m_Right   * (Radius * std::sin(Phi) * std::cos(Theta))
             + Frame.m_Up      * (Radius * std::sin(Phi) * std::sin(Theta));
    }

    // Sized off the distance to the root's nearest child so the marker reads at the same scale as
    // the wedges hanging off it. OverallRadius (the whole skeleton's own bounding radius, already
    // computed in ComputeBoneWorldsAndFraming) sets a scale-appropriate FLOOR for both that and the
    // degenerate no-children case - a fixed absolute floor (what this used to fall back to) reads
    // fine on a 1-2 unit tall rig but is sub-pixel and invisible on rigs authored at a much larger
    // scale (e.g. centimeters, where the whole character's bounding radius is ~100+ units).
    inline float RootSphereRadius(const xskeleton::skeleton& Skeleton, const std::vector<bone_world>& World, int iRoot, float OverallRadius)
    {
        const auto  Bones = Skeleton.getBones();
        float       MinDist = -1.0f;
        for (int i = 0; i < int(Bones.size()); ++i)
        {
            if (Bones[i].m_iParent != iRoot) continue;
            const float D = (World[i].m_Position - World[iRoot].m_Position).Length();
            if (MinDist < 0.0f || D < MinDist) MinDist = D;
        }
        const float Floor = std::max(OverallRadius * 0.02f, 1.0e-4f);
        return (MinDist > 1.0e-5f) ? std::max(MinDist * 0.18f, Floor) : Floor * 1.5f;
    }

    constexpr int g_RootSphereLonSegs  = 14;
    constexpr int g_RootSphereLatRings = 8;
    constexpr int g_RootSphereWireSegs = 28; // per great circle

    //---------------------------------------------------------------------------
    // Depth tint - CPU-computed per vertex every frame (camera-distance based), lerped toward the
    // background clear color. Selection is depth-invariant: always full brightness.
    //---------------------------------------------------------------------------

    struct wedge_style
    {
        xmath::fvec3    m_CameraPos         {};
        float           m_NearDepth         = 1.0f;
        float           m_FarDepth          = 15.0f;
        std::uint32_t   m_BackgroundColor   = IM_COL32(115, 115, 115, 255); // matches the viewport's own 0.45 gray
        std::uint32_t   m_NormalColor       = IM_COL32(255, 255, 255, 255); // white - now that the fill's alpha/boost are fixed, white reads as bright/clean rather than diluted-into-the-floor
        std::uint32_t   m_VirtualColor      = IM_COL32(255, 180, 84, 255);
        std::uint32_t   m_TwistColor        = IM_COL32(90, 110, 125, 255); // dim, desaturated - a real bone, just not one that should compete visually with the main limb chain
        std::uint32_t   m_SelectedColor     = IM_COL32(255, 40, 180, 255); // hot magenta - a distinct hue from the (now also white-ish) normal color, so selection still reads clearly
        std::uint32_t   m_RootColor         = IM_COL32(255, 220, 40, 255); // yellow - the root sphere's own color, unconditional on virtual/normal
        float           m_HoverBoost        = 2.5f; // multiplicative RGB brighten for whichever bone the mouse is over, regardless of selection/LOD/type color - a no-op for a channel already at 255 (e.g. plain white), which is why the fill ALSO goes fully opaque on hover (see BuildWedgeFillGeometry's TintAt)
        float           m_VirtualLineBoost  = 1.6f; // virtual bones are outline-only (no fill to lean on, see bFilled) and dashed on top of that - at rest, DepthTint's background blend plus the thin/gappy line reads as a dim, muddied color (easy to mistake for a totally different hue next to a nearby solid-filled bone) even though the same color looks fully correct the instant hover's boost brightens it. Compensates that gap permanently instead of relying on hover to reveal the true color.
    };

    inline std::uint32_t DepthTint(const xmath::fvec3& P, const xmath::fvec3& CameraPos, float NearD, float FarD, std::uint32_t BaseColor, std::uint32_t BgColor)
    {
        const float D     = (P - CameraPos).Length();
        const float Range = std::max(FarD - NearD, 0.001f);
        // Capped well under 1.0 so depth still reads as a cue, not a fade-to-neutral - at the old
        // 0.72 cap, a bone at typical viewing distance was already 72% blended into the gray
        // background, which is why every base color still looked "neutral" regardless of hue.
        const float T     = std::clamp((D - NearD) / Range, 0.0f, 1.0f) * 0.35f;

        auto Channel = [](std::uint32_t C, int Shift) -> int { return int((C >> Shift) & 0xFFu); };

        const int Br = Channel(BaseColor, 0),  Bg_ = Channel(BaseColor, 8),  Bb = Channel(BaseColor, 16), Ba = Channel(BaseColor, 24);
        const int Gr = Channel(BgColor,   0),  Gg  = Channel(BgColor,   8),  Gb = Channel(BgColor,   16);

        const int R = int(float(Br) + float(Gr - Br) * T);
        const int G = int(float(Bg_) + float(Gg - Bg_) * T);
        const int B = int(float(Bb) + float(Gb - Bb) * T);

        return IM_COL32(R, G, B, Ba);
    }

    // Multiplies just the RGB channels by Factor, clamped to 255, alpha untouched - shared by
    // VertexColor's hover highlight and FaceLit's per-triangle camera-facing shading.
    inline std::uint32_t ScaleColorRGB(std::uint32_t Color, float Factor)
    {
        auto Channel = [](std::uint32_t C, int Shift) -> int { return int((C >> Shift) & 0xFFu); };
        const int R = std::clamp(int(Channel(Color, 0)  * Factor), 0, 255);
        const int G = std::clamp(int(Channel(Color, 8)  * Factor), 0, 255);
        const int B = std::clamp(int(Channel(Color, 16) * Factor), 0, 255);
        return (Color & 0xFF000000u) | (std::uint32_t(B) << 16) | (std::uint32_t(G) << 8) | std::uint32_t(R);
    }

    // bHovered brightens the FINAL color multiplicatively, after selection/depth-tint have already
    // picked it - "no matter if it is selected or whatever" was the explicit ask, so this has to be
    // the last step, not folded into the selected/normal color choice itself.
    inline std::uint32_t VertexColor(const wedge_style& Style, const xmath::fvec3& P, bool bSelected, bool bHovered, std::uint32_t BaseColor)
    {
        std::uint32_t C = bSelected ? Style.m_SelectedColor : DepthTint(P, Style.m_CameraPos, Style.m_NearDepth, Style.m_FarDepth, BaseColor, Style.m_BackgroundColor);
        if (bHovered) C = ScaleColorRGB(C, Style.m_HoverBoost);
        return C;
    }

    //---------------------------------------------------------------------------
    // "Color by LOD" view option - one color per LOD tier, cycling if a rig somehow has more LODs
    // than the table. The compiled skeleton doesn't store a per-bone LOD value directly (see
    // xskeleton::skeleton::getLODBoneCounts' own comment): bones are sorted ascending by LOD, and
    // m_pLODBoneCount[L] is the cumulative count of bones active at LOD <= L, so a bone's own LOD is
    // just "the first L whose cumulative count exceeds this bone's index".
    //---------------------------------------------------------------------------

    constexpr std::uint32_t g_LODColors[] =
    { IM_COL32(255,  80,  80, 255)  // LOD0 - red
    , IM_COL32(255, 225,   0, 255)  // LOD1 - bright yellow (was orange - too close to red to read as "yellow" against this background)
    , IM_COL32(190, 255,  70, 255)  // LOD2 - yellow-green (shifted off LOD1's new yellow so the two tiers stay distinct)
    , IM_COL32(120, 255,  90, 255)  // LOD3 - green
    , IM_COL32( 80, 200, 255, 255)  // LOD4 - blue
    , IM_COL32(200, 120, 255, 255)  // LOD5 - purple
    };
    constexpr int g_nLODColors = int(sizeof(g_LODColors) / sizeof(g_LODColors[0]));

    inline int GetBoneLODLevel(const xskeleton::skeleton& Skeleton, int iBone)
    {
        const auto Counts = Skeleton.getLODBoneCounts();
        for (int L = 0; L < int(Counts.size()); ++L)
            if (iBone < int(Counts[L])) return L;
        return 0; // no LOD data at all - everything is LOD0
    }

    inline std::uint32_t LODColor(int Level)
    {
        return g_LODColors[std::clamp(Level, 0, g_nLODColors - 1)];
    }

    //---------------------------------------------------------------------------
    // "Color by Mask Weight" view option - visualizes one mask group's per-bone weight (see
    // xskeleton_desc::mask_group) as a color gradient, so an author can confirm a group covers
    // exactly the bones they intended without reading raw numbers off the tree.
    //---------------------------------------------------------------------------

    inline std::uint32_t MaskWeightColor(float Weight)
    {
        Weight = std::clamp(Weight, 0.0f, 1.0f);
        constexpr std::uint32_t Lo = IM_COL32( 70,  70,  70, 255); // weight 0 - reads as "uncolored", same family as the depth-tint background
        constexpr std::uint32_t Hi = IM_COL32( 60, 220, 255, 255); // weight 1 - bright cyan, distinct from every LOD tier's red/yellow/green/blue/purple ramp
        auto Channel = [](std::uint32_t C, int Shift) -> int { return int((C >> Shift) & 0xFFu); };
        auto Lerp8   = [&](int Shift) { const int A = Channel(Lo, Shift), B = Channel(Hi, Shift); return int(A + (B - A) * Weight); };
        return IM_COL32(Lerp8(0), Lerp8(8), Lerp8(16), 255);
    }

    // Reads the COMPILED resource's dense weight table (see xskeleton::skeleton::getMaskWeights) -
    // same "trust the compiled data for rendering, the descriptor only for editing" split GetBoneLODLevel
    // already uses. GroupIdx < 0 (group not found/not compiled yet) reads as weight 0 everywhere.
    inline float GetBoneMaskWeight(const xskeleton::skeleton& Skeleton, int GroupIdx, int iBone)
    {
        if (GroupIdx < 0) return 0.0f;
        const auto Weights = Skeleton.getMaskWeights(GroupIdx);
        if (iBone < 0 || iBone >= int(Weights.size())) return 0.0f;
        return float(Weights[iBone].m_Mask) / 65535.0f; // inverse of BuildMasks' std::lround(W * 65535.0f)
    }

    //---------------------------------------------------------------------------
    // "Exposed" view option - a plain binary highlight for whichever bones are marked as sockets
    // (xskeleton_desc::bone::m_bExpose, baked into bone_flags::SOCKET) - no gradient needed since
    // exposure isn't a continuous quantity like LOD or mask weight.
    //---------------------------------------------------------------------------

    inline std::uint32_t ExposeColor(bool bExposed)
    {
        constexpr std::uint32_t Unexposed = IM_COL32( 70,  70,  70, 255); // same dim "uncolored" gray MaskWeightColor's weight-0 end uses
        constexpr std::uint32_t Exposed   = IM_COL32(255, 140,  20, 255); // vivid orange - LOD1 moved off orange onto yellow, freeing it up, and it's nowhere near m_SelectedColor's magenta or mask-weight's cyan
        return bExposed ? Exposed : Unexposed;
    }

    // Centralizes what used to be duplicated at every BaseColor call site: mask-weight/expose/LOD
    // coloring (in that priority order, though in practice the caller only ever has one active at a
    // time - see render_color_mode) wins over the caller's own type-based default
    // (virtual/twist/normal/root).
    inline std::uint32_t ResolveBoneColor(const xskeleton::skeleton& Skeleton, int iBone, render_color_mode Mode, int MaskGroupIdx, std::uint32_t DefaultColor)
    {
        if (MaskGroupIdx >= 0)                            return MaskWeightColor(GetBoneMaskWeight(Skeleton, MaskGroupIdx, iBone));
        if (Mode == render_color_mode::EXPOSE_RENDER)
        {
            const auto Bones = Skeleton.getBones();
            return ExposeColor(iBone >= 0 && iBone < int(Bones.size()) && Bones[iBone].m_Flags.m_bSocket);
        }
        if (Mode == render_color_mode::LOD_RENDER)        return LODColor(GetBoneLODLevel(Skeleton, iBone));
        return DefaultColor;
    }

    //---------------------------------------------------------------------------
    // Buffer capacity: 8 edges/bone, dashed edges split into 3 emitted sub-segments (6 verts) vs
    // 1 segment (2 verts) when solid - worst case (every bone virtual) is 8*6=48 verts/bone.
    //---------------------------------------------------------------------------

    inline constexpr int   g_MaxWedgeVertices = 65536;

    // Compensates for the wedge fill's own alpha blend diluting brightness against the gray floor
    // (FillAlphaScale ~0.55 means the true color only ever contributes ~55% of what reaches the
    // screen) - see E23_WedgeFill_frag.glsl's own comment. Tunable in one place.
    inline constexpr float g_WedgeFillBoost   = 1.8f;

    inline void EmitSegment(std::vector<e19::draw_vert>& Verts, const xmath::fvec3& A, const xmath::fvec3& B, std::uint32_t ColorA, std::uint32_t ColorB)
    {
        e19::draw_vert VA{}; VA.m_X = A.m_X; VA.m_Y = A.m_Y; VA.m_Z = A.m_Z; VA.m_U = 0.0f; VA.m_V = 0.0f; VA.m_Color = ColorA;
        e19::draw_vert VB{}; VB.m_X = B.m_X; VB.m_Y = B.m_Y; VB.m_Z = B.m_Z; VB.m_U = 0.0f; VB.m_V = 0.0f; VB.m_Color = ColorB;
        Verts.push_back(VA);
        Verts.push_back(VB);
    }

    // Virtual bones are dashed by only emitting alternating sub-segments along the edge - LINE_LIST
    // has no native dash support, so this approximates it geometrically.
    inline void EmitEdge(std::vector<e19::draw_vert>& Verts, const xmath::fvec3& A, const xmath::fvec3& B, bool bDashed, const wedge_style& Style, bool bSelected, bool bHovered, std::uint32_t BaseColor)
    {
        auto ColorAt = [&](const xmath::fvec3& P) { return VertexColor(Style, P, bSelected, bHovered, BaseColor); };

        if (!bDashed)
        {
            EmitSegment(Verts, A, B, ColorAt(A), ColorAt(B));
            return;
        }

        constexpr int Splits = 6;
        for (int i = 0; i < Splits; i += 2)
        {
            const float t0 = float(i)     / float(Splits);
            const float t1 = float(i + 1) / float(Splits);
            const xmath::fvec3 P0 = A + (B - A) * t0;
            const xmath::fvec3 P1 = A + (B - A) * t1;
            EmitSegment(Verts, P0, P1, ColorAt(P0), ColorAt(P1));
        }
    }

    // Wireframe: 3 orthogonal great circles rather than a full lat/long wire mesh - reads clearly as
    // "a sphere" for a fraction of the vertex cost.
    inline void BuildRootSphereWireframe(const xmath::fvec3& Center, const sphere_frame& Frame, float Radius, bool bSelected, bool bHovered, const wedge_style& Style, std::uint32_t Color, std::vector<e19::draw_vert>& Verts)
    {
        auto Circle = [&](const xmath::fvec3& AxisA, const xmath::fvec3& AxisB)
        {
            xmath::fvec3 Prev = Center + AxisA * Radius;
            for (int i = 1; i <= g_RootSphereWireSegs; ++i)
            {
                const float Angle = (2.0f * std::numbers::pi_v<float>) * (float(i) / float(g_RootSphereWireSegs));
                const xmath::fvec3 Cur = Center + AxisA * (Radius * std::cos(Angle)) + AxisB * (Radius * std::sin(Angle));
                EmitEdge(Verts, Prev, Cur, false, Style, bSelected, bHovered, Color);
                Prev = Cur;
            }
        };
        Circle(Frame.m_Right,   Frame.m_Up);
        Circle(Frame.m_Up,      Frame.m_Forward);
        Circle(Frame.m_Forward, Frame.m_Right);
    }

    inline void BuildWedgeGeometry(const xskeleton::skeleton& Skeleton, const std::vector<bone_world>& World, const std::vector<bool>& IsTwistBone, const wedge_style& Style, const std::set<int>& SelectedBones, float OverallRadius, render_color_mode Mode, int MaskGroupIdx, int HoveredBone, std::vector<e19::draw_vert>& Verts, bool bOutlineOnly = false)
    {
        Verts.clear();

        const auto Bones = Skeleton.getBones();
        if (World.size() != Bones.size()) return;

        for (int i = 0; i < int(Bones.size()); ++i)
        {
            const int iParent = Bones[i].m_iParent;
            if (iParent < 0) continue; // root has no edge to draw

            const xmath::fvec3& A = World[iParent].m_Position;
            const xmath::fvec3& B = World[i].m_Position;

            wedge_shape Shape;
            if (!ComputeWedgeShape(A, B, World[i].m_Right, World[i].m_Up, Shape)) continue;

            // Dashing is a TYPE-based style choice (virtual placeholder, or a twist helper joint - see
            // LoadSkeleton - that shouldn't visually compete with the main limb chain) and stays the
            // same in both poses - it never encoded bind-data confidence (BuildWedgeFillGeometry no
            // longer does either, see FillAlphaScale's own comment).
            const bool          bSelected  = SelectedBones.count(i) != 0;
            const bool          bHovered   = i == HoveredBone;
            const bool          bVirtual   = Bones[i].m_Flags.m_bVirtual;
            const bool          bTwist     = i < int(IsTwistBone.size()) && IsTwistBone[i];
            const bool          bDashed    = (bVirtual || bTwist) && !bSelected;
            const std::uint32_t BaseColor  = ResolveBoneColor(Skeleton, i, Mode, MaskGroupIdx
                                            , bVirtual ? Style.m_VirtualColor : bTwist ? Style.m_TwistColor : Style.m_NormalColor);
            // See m_VirtualLineBoost's own comment - virtual bones have no fill to lean on, so their
            // outline needs a permanent brightness boost or they read as dim/muddy at rest.
            const std::uint32_t OutlineColor = bVirtual ? ScaleColorRGB(BaseColor, Style.m_VirtualLineBoost) : BaseColor;

            // BuildWedgeFillGeometry gives every non-virtual, non-twist bone (bFilled here) a solid
            // body already - drawing the full 8-edge wireframe (both tips to all 4 ring points) on
            // TOP of that solid shape just doubled up as visual clutter, since the fill alone already
            // reads clearly as "a tapered bone". The ring alone (its widest cross-section) is enough
            // of an outline/accent on a filled bone. Virtual/twist bones have no fill to lean on, so
            // they keep the full wireframe - it's the only thing conveying their shape at all.
            const bool bFilled = !bOutlineOnly && !bVirtual && !bTwist;      // with no fill pass under them (bOutlineOnly) every bone keeps its full wireframe
            if (bFilled)
            {
                for (int k = 0; k < 4; ++k)
                    EmitEdge(Verts, Shape.m_Ring[k], Shape.m_Ring[(k + 1) & 3], false, Style, bSelected, bHovered, OutlineColor);
            }
            else
            {
                for (int k = 0; k < 4; ++k)
                {
                    EmitEdge(Verts, A,               Shape.m_Ring[k], bDashed, Style, bSelected, bHovered, OutlineColor);
                    EmitEdge(Verts, Shape.m_Ring[k],  B,               bDashed, Style, bSelected, bHovered, OutlineColor);
                }
            }

            if (Verts.size() > std::size_t(g_MaxWedgeVertices - 96))
                break; // stay comfortably under the buffer's capacity
        }

        for (int i = 0; i < int(Bones.size()); ++i)
        {
            if (Bones[i].m_iParent >= 0) continue; // only parentless bones get a marker

            const bool bSelected = SelectedBones.count(i) != 0;
            const bool bHovered  = i == HoveredBone;

            sphere_frame Frame;
            if (!ComputeOrthoFrame(World[i].m_Right, World[i].m_Up, Frame)) continue;

            const std::uint32_t RootColor = ResolveBoneColor(Skeleton, i, Mode, MaskGroupIdx, Style.m_RootColor);
            BuildRootSphereWireframe(World[i].m_Position, Frame, RootSphereRadius(Skeleton, World, i, OverallRadius), bSelected, bHovered, Style, RootColor, Verts);
        }
    }

    //---------------------------------------------------------------------------
    // A very light fill so a wedge registers as a solid shape rather than just a hairline outline -
    // the concept mockup this design comes from always paired the two ("Pass 1 - very light fill...
    // Pass 2 - the outline is the real signal now, not a backstop"), but only the outline pass ever
    // got built here. Virtual/twist bones stay outline-only (a deliberate style choice, unrelated to
    // bind-data confidence). Inferred bones (no real bind data - see BuildWedgeGeometry) still get
    // filled, just dimmer than confident ones - for rigs where most bones lack real bind data, bind
    // pose would otherwise render as an almost-invisible dashed skeleton next to frozen pose's fully
    // solid one; a dimmer fill keeps bind pose reading as a body while still flagging uncertainty.
    //---------------------------------------------------------------------------

    inline void EmitTri(std::vector<e19::draw_vert>& Verts, const xmath::fvec3& A, const xmath::fvec3& B, const xmath::fvec3& C, std::uint32_t Color)
    {
        e19::draw_vert V{}; V.m_U = 0.0f; V.m_V = 0.0f; V.m_Color = Color;
        V.m_X = A.m_X; V.m_Y = A.m_Y; V.m_Z = A.m_Z; Verts.push_back(V);
        V.m_X = B.m_X; V.m_Y = B.m_Y; V.m_Z = B.m_Z; Verts.push_back(V);
        V.m_X = C.m_X; V.m_Y = C.m_Y; V.m_Z = C.m_Z; Verts.push_back(V);
    }

    // "Headlight" shading baked per-triangle on the CPU: a face angled toward the camera reads
    // brighter, one angled away reads dimmer - gives the otherwise flat-tinted wedge fills a sense
    // of 3D form without needing real normals in e19::draw_vert (shared with other examples, not
    // something to extend just for this). Every EmitTri call here is already one flat face with a
    // single color for all 3 vertices, so computing the face normal from those same 3 points and
    // treating the camera as the light direction is exact, not an approximation.
    inline std::uint32_t FaceLit(const xmath::fvec3& P0, const xmath::fvec3& P1, const xmath::fvec3& P2, const xmath::fvec3& CameraPos, std::uint32_t Color)
    {
        // NormalizeSafe, not Normalize: a degenerate (zero-area) triangle - e.g. the root sphere's
        // own poles, already noted as zero-area where they're built - has a zero-length cross
        // product, and plain Normalize() asserts on that (xmath_fvec3_inline.h's own documented
        // behavior). NormalizeSafe just yields zero instead, which correctly contributes no lighting
        // rather than crashing.
        xmath::fvec3       Normal = (P1 - P0).Cross(P2 - P0);
        Normal.NormalizeSafe();
        const xmath::fvec3 Center = (P0 + P1 + P2) * (1.0f / 3.0f);
        xmath::fvec3       ViewDir = CameraPos - Center;
        ViewDir.NormalizeSafe();
        const float NdotV = std::max(0.0f, Normal.Dot(ViewDir));

        constexpr float Ambient = 0.55f; // never fully dark on the away-facing side
        const float     Lit     = Ambient + (1.0f - Ambient) * NdotV;
        return ScaleColorRGB(Color, Lit);
    }

    // Per-object (not per-triangle) back-to-front ordering - the standard, good-enough approximation
    // for alpha blending a scene of separate translucent objects without a full triangle sort. Each
    // entry is either a regular wedge (bIsRoot false, iBone/iParent both meaningful) or a root sphere
    // (bIsRoot true, iParent unused).
    struct fill_entry
    {
        int     m_iBone;
        int     m_iParent;
        bool    m_bIsRoot;
        float   m_CameraDist;
    };

    inline void BuildWedgeFillGeometry(const xskeleton::skeleton& Skeleton, const std::vector<bone_world>& World, const std::vector<bool>& IsTwistBone, const wedge_style& Style, const std::set<int>& SelectedBones, float OverallRadius, render_color_mode Mode, int MaskGroupIdx, int HoveredBone, std::vector<e19::draw_vert>& Verts)
    {
        Verts.clear();

        const auto Bones = Skeleton.getBones();
        if (World.size() != Bones.size()) return;

        // This, not the base RGB hue, was the real reason the skeleton kept reading as "dim/neutral"
        // through several color changes: at 0.16 alpha, ANY color is 84% gray floor showing through -
        // the base hue barely matters once it's diluted that much.
        //
        // Bind pose used to dim further still (InferredFillAlphaScale) for bones whose bind data was
        // propagated rather than authored (m_bRealBindData==false) - meaningful in isolation, but it
        // made Bind and Frozen look like two different intensity levels overall for any rig (like this
        // one) where most bones only have propagated bind data. Frozen's flat, always-full intensity
        // is the one that reads correctly, so both poses now just use it.
        constexpr float FillAlphaScale = 0.55f;

        std::vector<fill_entry> Entries;
        Entries.reserve(Bones.size());
        for (int i = 0; i < int(Bones.size()); ++i)
        {
            const int iParent = Bones[i].m_iParent;
            if (iParent < 0)
            {
                const float Dist = (World[i].m_Position - Style.m_CameraPos).Length();
                Entries.push_back({ i, -1, true, Dist });
                continue;
            }

            if (Bones[i].m_Flags.m_bVirtual) continue; // matches wedges: virtual bones stay outline-only
            if (i < int(IsTwistBone.size()) && IsTwistBone[i]) continue;

            const float Dist = ((World[iParent].m_Position + World[i].m_Position) * 0.5f - Style.m_CameraPos).Length();
            Entries.push_back({ i, iParent, false, Dist });
        }

        // Far first - each entry then draws over whatever's already behind it, the standard way to
        // get correct-looking alpha blending without sorting individual triangles.
        std::sort(Entries.begin(), Entries.end(), [](const fill_entry& A, const fill_entry& B) { return A.m_CameraDist > B.m_CameraDist; });

        for (auto& E : Entries)
        {
            const bool  bSelected = SelectedBones.count(E.m_iBone) != 0;
            const bool  bHovered  = E.m_iBone == HoveredBone;

            if (E.m_bIsRoot)
            {
                sphere_frame Frame;
                if (!ComputeOrthoFrame(World[E.m_iBone].m_Right, World[E.m_iBone].m_Up, Frame)) continue;

                const float ThisAlpha = FillAlphaScale;
                const std::uint32_t RootBaseColor = ResolveBoneColor(Skeleton, E.m_iBone, Mode, MaskGroupIdx, Style.m_RootColor);
                const auto  TintAt    = [&](const xmath::fvec3& P) -> std::uint32_t
                {
                    const std::uint32_t C = VertexColor(Style, P, bSelected, bHovered, RootBaseColor);
                    // Fully opaque, not just RGB-boosted: a white bone's RGB is already at 255, so
                    // multiplying it (VertexColor's own hover boost) is a no-op - alpha is the only
                    // lever left that can actually make a maxed-out color look brighter once blended
                    // against the floor. Same reasoning selection already gets.
                    if (bSelected || bHovered) return C;
                    const int A8 = int(((C >> 24) & 0xFFu) * ThisAlpha);
                    return (C & 0x00FFFFFFu) | (std::uint32_t(A8) << 24);
                };

                const float Radius = RootSphereRadius(Skeleton, World, E.m_iBone, OverallRadius);
                for (int j = 0; j < g_RootSphereLatRings; ++j)
                {
                    const float Phi0 = std::numbers::pi_v<float> * (float(j)     / float(g_RootSphereLatRings));
                    const float Phi1 = std::numbers::pi_v<float> * (float(j + 1) / float(g_RootSphereLatRings));
                    for (int i = 0; i < g_RootSphereLonSegs; ++i)
                    {
                        const float Theta0 = (2.0f * std::numbers::pi_v<float>) * (float(i)     / float(g_RootSphereLonSegs));
                        const float Theta1 = (2.0f * std::numbers::pi_v<float>) * (float(i + 1) / float(g_RootSphereLonSegs));

                        const xmath::fvec3 P00 = SpherePoint(World[E.m_iBone].m_Position, Frame, Radius, Theta0, Phi0);
                        const xmath::fvec3 P10 = SpherePoint(World[E.m_iBone].m_Position, Frame, Radius, Theta1, Phi0);
                        const xmath::fvec3 P01 = SpherePoint(World[E.m_iBone].m_Position, Frame, Radius, Theta0, Phi1);
                        const xmath::fvec3 P11 = SpherePoint(World[E.m_iBone].m_Position, Frame, Radius, Theta1, Phi1);

                        // Winding order matters now that cull::BACK (the pipeline default) is
                        // actually removing something visible - (P00,P10,P11)/(P00,P11,P01) computes
                        // an INWARD-pointing normal for this Right/Up/Forward parametrization
                        // (verified by hand), so the OUTER hemisphere was the one getting culled and
                        // the inner one showed through instead. Swapped to the outward winding.
                        EmitTri(Verts, P00, P11, P10, FaceLit(P00, P11, P10, Style.m_CameraPos, TintAt(P00))); // degenerate (zero-area) at the poles - harmless
                        EmitTri(Verts, P00, P01, P11, FaceLit(P00, P01, P11, Style.m_CameraPos, TintAt(P00)));
                    }
                }
                continue;
            }

            const xmath::fvec3& A = World[E.m_iParent].m_Position;
            const xmath::fvec3& B = World[E.m_iBone].m_Position;

            wedge_shape Shape;
            if (!ComputeWedgeShape(A, B, World[E.m_iBone].m_Right, World[E.m_iBone].m_Up, Shape)) continue;

            const float ThisAlpha = FillAlphaScale;
            const std::uint32_t WedgeBaseColor = ResolveBoneColor(Skeleton, E.m_iBone, Mode, MaskGroupIdx, Style.m_NormalColor);
            const auto  TintAt    = [&](const xmath::fvec3& P) -> std::uint32_t
            {
                const std::uint32_t C = VertexColor(Style, P, bSelected, bHovered, WedgeBaseColor);
                // Fully opaque, not just RGB-boosted - see the root sphere's identical TintAt for why.
                if (bSelected || bHovered) return C;
                const int A8 = int(((C >> 24) & 0xFFu) * ThisAlpha);
                return (C & 0x00FFFFFFu) | (std::uint32_t(A8) << 24);
            };

            for (int k = 0; k < 4; ++k)
            {
                const xmath::fvec3& R0 = Shape.m_Ring[k];
                const xmath::fvec3& R1 = Shape.m_Ring[(k + 1) & 3];
                EmitTri(Verts, A, R0, R1, FaceLit(A, R0, R1, Style.m_CameraPos, TintAt(A)));
                EmitTri(Verts, B, R1, R0, FaceLit(B, R1, R0, Style.m_CameraPos, TintAt(B)));
            }

            if (Verts.size() > std::size_t(g_MaxWedgeVertices - 24))
                break;
        }
    }

    //---------------------------------------------------------------------------
    // GPU picking geometry - one [Start, Start+Count) slice of Verts per bone (root sphere or
    // wedge, same shapes as the visual passes), used to issue one small Draw per bone with a
    // per-draw BoneID push constant (see the pick pipeline in E23_Example). Deliberately DECOUPLED
    // from BuildWedgeFillGeometry: that one skips virtual/twist bones because they're meant to stay
    // outline-only on screen, but a bone being visually outline-only shouldn't make it unclickable -
    // every bone gets a solid pickable volume here regardless of how it's styled. Color is
    // irrelevant (the pick fragment shader never writes it to the visible framebuffer), so 0 is
    // used throughout.
    //---------------------------------------------------------------------------

    struct pick_range
    {
        int     m_iBone;
        int     m_Start;
        int     m_Count;
    };

    // A click freezes the mouse position and modifier-key state; the actual GPU draw/readback
    // happens over the next few frames (see PickDelayFrames' own comment) using THIS frozen
    // position, not wherever the mouse has drifted to by the time the result is ready.
    struct pick_request
    {
        ImVec2  m_MousePos; // framebuffer-pixel space (already DisplayPos-adjusted), not raw ImGui screen space - see where this is constructed
        bool    m_bCtrl        = false;
        bool    m_bShift       = false;
        bool    m_bDrawIssued  = false;
        int     m_FramesLeft   = 0;
    };

    inline void BuildPickGeometry(const xskeleton::skeleton& Skeleton, const std::vector<bone_world>& World, float OverallRadius, std::vector<e19::draw_vert>& Verts, std::vector<pick_range>& OutRanges)
    {
        Verts.clear();
        OutRanges.clear();

        const auto Bones = Skeleton.getBones();
        if (World.size() != Bones.size()) return;

        auto EmitRange = [&](int iBone, int Start)
        {
            const int Count = int(Verts.size()) - Start;
            if (Count > 0) OutRanges.push_back({ iBone, Start, Count });
        };

        for (int i = 0; i < int(Bones.size()); ++i)
        {
            const int Start   = int(Verts.size());
            const int iParent = Bones[i].m_iParent;

            if (iParent < 0)
            {
                sphere_frame Frame;
                if (!ComputeOrthoFrame(World[i].m_Right, World[i].m_Up, Frame)) continue;

                const float Radius = RootSphereRadius(Skeleton, World, i, OverallRadius);
                for (int j = 0; j < g_RootSphereLatRings; ++j)
                {
                    const float Phi0 = std::numbers::pi_v<float> * (float(j)     / float(g_RootSphereLatRings));
                    const float Phi1 = std::numbers::pi_v<float> * (float(j + 1) / float(g_RootSphereLatRings));
                    for (int k = 0; k < g_RootSphereLonSegs; ++k)
                    {
                        const float Theta0 = (2.0f * std::numbers::pi_v<float>) * (float(k)     / float(g_RootSphereLonSegs));
                        const float Theta1 = (2.0f * std::numbers::pi_v<float>) * (float(k + 1) / float(g_RootSphereLonSegs));

                        const xmath::fvec3 P00 = SpherePoint(World[i].m_Position, Frame, Radius, Theta0, Phi0);
                        const xmath::fvec3 P10 = SpherePoint(World[i].m_Position, Frame, Radius, Theta1, Phi0);
                        const xmath::fvec3 P01 = SpherePoint(World[i].m_Position, Frame, Radius, Theta0, Phi1);
                        const xmath::fvec3 P11 = SpherePoint(World[i].m_Position, Frame, Radius, Theta1, Phi1);

                        // Same outward-winding fix as BuildWedgeFillGeometry's root sphere - without
                        // it the pick pipeline's own cull::BACK would make the visible hemisphere
                        // unpickable and the hidden one pickable instead.
                        EmitTri(Verts, P00, P11, P10, 0);
                        EmitTri(Verts, P00, P01, P11, 0);
                    }
                }
                EmitRange(i, Start);
                continue;
            }

            const xmath::fvec3& A = World[iParent].m_Position;
            const xmath::fvec3& B = World[i].m_Position;

            wedge_shape Shape;
            if (!ComputeWedgeShape(A, B, World[i].m_Right, World[i].m_Up, Shape)) continue;

            for (int k = 0; k < 4; ++k)
            {
                const xmath::fvec3& R0 = Shape.m_Ring[k];
                const xmath::fvec3& R1 = Shape.m_Ring[(k + 1) & 3];
                EmitTri(Verts, A, R0, R1, 0);
                EmitTri(Verts, B, R1, R0, 0);
            }
            EmitRange(i, Start);

            if (Verts.size() > std::size_t(g_MaxWedgeVertices - 24))
                break;
        }
    }

    //---------------------------------------------------------------------------
    // Viewport picking - ray-vs-triangle (Moller-Trumbore) against the same 8 wedge faces per
    // bone (4 near: A-ring[k]-ring[k+1], 4 far: B-ring[k+1]-ring[k]), closest hit wins.
    //---------------------------------------------------------------------------

    inline bool RayTriangleIntersect(const xmath::fvec3& Origin, const xmath::fvec3& Dir, const xmath::fvec3& V0, const xmath::fvec3& V1, const xmath::fvec3& V2, float& OutT)
    {
        constexpr float Epsilon = 1.0e-6f;

        const xmath::fvec3 Edge1 = V1 - V0;
        const xmath::fvec3 Edge2 = V2 - V0;
        const xmath::fvec3 H     = Dir.Cross(Edge2);
        const float        A     = Edge1.Dot(H);
        if (std::fabs(A) < Epsilon) return false;

        const float        F = 1.0f / A;
        const xmath::fvec3  S = Origin - V0;
        const float         U = F * S.Dot(H);
        if (U < 0.0f || U > 1.0f) return false;

        const xmath::fvec3 Q = S.Cross(Edge1);
        const float        V = F * Dir.Dot(Q);
        if (V < 0.0f || U + V > 1.0f) return false;

        const float T = F * Edge2.Dot(Q);
        if (T <= Epsilon) return false;

        OutT = T;
        return true;
    }

    // Generic (Dir need not be unit length - T then matches RayTriangleIntersect's own convention of
    // "same units as Dir", which is all that matters since both are only ever compared against each
    // other for the same ray).
    inline bool RaySphereIntersect(const xmath::fvec3& Origin, const xmath::fvec3& Dir, const xmath::fvec3& Center, float Radius, float& OutT)
    {
        const xmath::fvec3 OC = Origin - Center;
        const float A = Dir.Dot(Dir);
        if (A < 1.0e-12f) return false;
        const float B = 2.0f * Dir.Dot(OC);
        const float C = OC.Dot(OC) - Radius * Radius;
        const float Disc = B * B - 4.0f * A * C;
        if (Disc < 0.0f) return false;

        const float SqrtDisc = std::sqrt(Disc);
        float T = (-B - SqrtDisc) / (2.0f * A);
        if (T <= 1.0e-6f) T = (-B + SqrtDisc) / (2.0f * A);
        if (T <= 1.0e-6f) return false;

        OutT = T;
        return true;
    }

    inline void PickWedge(const xskeleton::skeleton& Skeleton, const std::vector<bone_world>& World, const xmath::fvec3& Origin, const xmath::fvec3& Dir, float OverallRadius, int& OutBone, float& OutT)
    {
        OutBone = -1;
        OutT    = std::numeric_limits<float>::max();

        const auto Bones = Skeleton.getBones();
        if (World.size() != Bones.size()) return;

        for (int i = 0; i < int(Bones.size()); ++i)
        {
            const int iParent = Bones[i].m_iParent;
            if (iParent < 0) continue;

            const xmath::fvec3& A = World[iParent].m_Position;
            const xmath::fvec3& B = World[i].m_Position;

            wedge_shape Shape;
            if (!ComputeWedgeShape(A, B, World[i].m_Right, World[i].m_Up, Shape)) continue;

            for (int k = 0; k < 4; ++k)
            {
                const xmath::fvec3& R0 = Shape.m_Ring[k];
                const xmath::fvec3& R1 = Shape.m_Ring[(k + 1) & 3];

                float T;
                if (RayTriangleIntersect(Origin, Dir, A, R0, R1, T) && T < OutT) { OutT = T; OutBone = i; }
                if (RayTriangleIntersect(Origin, Dir, B, R1, R0, T) && T < OutT) { OutT = T; OutBone = i; }
            }
        }

        for (int i = 0; i < int(Bones.size()); ++i)
        {
            if (Bones[i].m_iParent >= 0) continue;

            float T;
            if (RaySphereIntersect(Origin, Dir, World[i].m_Position, RootSphereRadius(Skeleton, World, i, OverallRadius), T) && T < OutT) { OutT = T; OutBone = i; }
        }
    }

}

#endif // XSKELETON_EDITOR_VIEW_H
