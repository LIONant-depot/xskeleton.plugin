#ifndef XSKELETON_JOINTS_H
#define XSKELETON_JOINTS_H
#pragma once
// Joint limits of a skeleton (xLION_Docs animation_skeleton_joints_design_v2, J1), shared by the compiler and the editor so both resolve the
// same way: every bone from its override, else the pinned profile through the bone mapping, else Unspecified (free). Deterministic from
// the descriptor and the skeleton alone. The evaluator is xanimation::joint_limit (dependencies/xAnimation/source/ik/xanimation_joint_limits.h).
#include "plugins/xskeleton.plugin/source/xskeleton_descriptor.h"
#include "dependencies/xAnimation/source/xanimation.h"
#include "dependencies/xstrtool/source/xstrtool.h"
#include <format>
#include <charconv>

namespace xskeleton_joints
{
    struct resolution
    {
        std::vector<xanimation::joint_limit>    m_Limits;
        std::vector<std::string>                m_Source;       // per bone: "override (...)", "preset", "preset, frame needs confirmation", "unspecified"
        std::vector<std::string>                m_Warnings;
        std::vector<std::string>                m_Errors;       // a malformed override: the compile fails, naming the field
        int                                     m_nOverride = 0, m_nPreset = 0, m_nUnspecified = 0, m_nNeedFrame = 0;
    };

    inline constexpr float deg_v = 3.14159265f / 180.0f;

    inline xanimation::joint_limit LimitFromOverride(const xskeleton_desc::joint_override& O, const xmath::fquat& Rest, const xmath::fquat& SuggestedFrame)
    {
        xanimation::joint_limit L;
        L.m_Type  = static_cast<xanimation::joint_type>(O.m_Geometry);
        L.m_Rest  = Rest.NormalizeSafeCopy();
        L.m_Frame = O.m_bFrameSet ? O.m_Frame : SuggestedFrame;
        if (O.m_bFrameSet && std::abs(L.m_Frame.LengthSq() - 1.0f) < 1e-2f) L.m_Frame = L.m_Frame.NormalizeCopy();
        L.m_HingeMin = O.m_HingeMin * deg_v; L.m_HingeMax = O.m_HingeMax * deg_v;
        L.m_SwingY = static_cast<xanimation::axis_motion>(O.m_SwingY); L.m_SwingZ = static_cast<xanimation::axis_motion>(O.m_SwingZ); L.m_Twist = static_cast<xanimation::axis_motion>(O.m_Twist);
        L.m_SwingCy = O.m_SwingCentre.m_X * deg_v; L.m_SwingCz = O.m_SwingCentre.m_Y * deg_v; L.m_SwingRy = O.m_SwingRadius.m_X * deg_v; L.m_SwingRz = O.m_SwingRadius.m_Y * deg_v;
        L.m_TwistCentre = O.m_TwistCentre * deg_v; L.m_TwistHalf = O.m_TwistHalf * deg_v;
        return L;
    }

    // the override that reproduces a limit (the geometry fields; source, review and provenance are the caller's)
    inline void SetOverrideFields(xskeleton_desc::joint_override& O, const xanimation::joint_limit& L, bool bFrameSet)
    {
        O.m_Geometry = static_cast<xskeleton_desc::joint_geometry>(L.m_Type);
        O.m_bFrameSet = bFrameSet; O.m_Frame = bFrameSet ? L.m_Frame : xmath::fquat::fromIdentity();
        O.m_HingeMin = L.m_HingeMin / deg_v; O.m_HingeMax = L.m_HingeMax / deg_v;
        O.m_SwingY = static_cast<xskeleton_desc::joint_axis>(L.m_SwingY); O.m_SwingZ = static_cast<xskeleton_desc::joint_axis>(L.m_SwingZ); O.m_Twist = static_cast<xskeleton_desc::joint_axis>(L.m_Twist);
        O.m_SwingCentre = { L.m_SwingCy / deg_v, L.m_SwingCz / deg_v }; O.m_SwingRadius = { L.m_SwingRy / deg_v, L.m_SwingRz / deg_v };
        O.m_TwistCentre = L.m_TwistCentre / deg_v; O.m_TwistHalf = L.m_TwistHalf / deg_v;
    }

    // what a review vouches for: the override's geometry and frame (not its review state, provenance or label)
    inline std::string ReviewHash(const xskeleton_desc::joint_override& O)
    {
        const auto S = std::format("{}|{}|{}|{:.4f},{:.4f},{:.4f},{:.4f}|{:.3f},{:.3f}|{},{},{}|{:.3f},{:.3f},{:.3f},{:.3f}|{:.3f},{:.3f}", O.m_BoneName, int(O.m_Geometry), O.m_bFrameSet
            , O.m_Frame.m_X, O.m_Frame.m_Y, O.m_Frame.m_Z, O.m_Frame.m_W, O.m_HingeMin, O.m_HingeMax, int(O.m_SwingY), int(O.m_SwingZ), int(O.m_Twist)
            , O.m_SwingCentre.m_X, O.m_SwingCentre.m_Y, O.m_SwingRadius.m_X, O.m_SwingRadius.m_Y, O.m_TwistCentre, O.m_TwistHalf);
        return std::format("{:08X}", xstrtool::CRC32(S));
    }

    inline resolution Resolve(const xskeleton_desc::descriptor& Desc, std::span<const std::string> Names, std::span<const std::int32_t> Parents, const xanimation::pose& Rest)
    {
        resolution R;
        const int N = static_cast<int>(Names.size());
        std::vector<std::string_view> Views(Names.begin(), Names.end());
        std::vector<xanimation::joint_preset_row> Rows(N);
        if (Desc.m_JointProfile == xskeleton_desc::joint_profile::HUMANOID_V1) R.m_Limits = xanimation::HumanoidJointPresetV1(xanimation::skeleton_view{ Parents }, Views, Rest, &Rows);
        else { R.m_Limits.resize(N); for (int b = 0; b < N; ++b) R.m_Limits[b].m_Rest = Rest.m_Rotation[b]; }
        R.m_Source.resize(N);
        for (int b = 0; b < N; ++b)
        {
            const bool bPreset = Rows[b].m_Source == xanimation::joint_source::PRESET;
            const bool bFrame  = bPreset && R.m_Limits[b].m_Type != xanimation::joint_type::FREE && Rows[b].m_Frame == xanimation::frame_confidence::NEEDS_CONFIRMATION;
            R.m_Source[b] = bPreset ? (bFrame ? "preset, frame needs confirmation" : "preset") : "unspecified";
            if (bPreset) ++R.m_nPreset; else ++R.m_nUnspecified;
            if (bFrame) ++R.m_nNeedFrame;
        }
        for (const auto& O : Desc.m_JointOverrides)
        {
            int b = -1;
            for (int i = 0; i < N; ++i) if (Names[i] == O.m_BoneName) { b = i; break; }
            if (b < 0) { R.m_Warnings.push_back(std::format("Joint override for '{}': no bone has that name (kept in the descriptor, not applied)", O.m_BoneName)); continue; }
            xanimation::frame_confidence C;
            const bool bPresetFrame = Rows[b].m_Source == xanimation::joint_source::PRESET && R.m_Limits[b].m_Type != xanimation::joint_type::FREE;
            const auto Suggested = bPresetFrame ? R.m_Limits[b].m_Frame : xanimation::SuggestFrame(Parents, Rest, b, C);
            const auto L = LimitFromOverride(O, Rest.m_Rotation[b], Suggested);
            if (const char* pErr = xanimation::Validate(L); pErr) { R.m_Errors.push_back(std::format("Joint override for '{}': {}", O.m_BoneName, pErr)); continue; }
            if (Rows[b].m_Source == xanimation::joint_source::PRESET) --R.m_nPreset; else --R.m_nUnspecified;
            ++R.m_nOverride;
            R.m_Limits[b] = L;
            R.m_Source[b] = std::format("override ({}{})", O.m_Source == xskeleton_desc::joint_source::FITTED ? "fitted" : O.m_Source == xskeleton_desc::joint_source::PRESET ? "preset copy" : "custom"
                                       , O.m_Review == xskeleton_desc::joint_review::REVIEWED ? ", reviewed" : O.m_Review == xskeleton_desc::joint_review::NEEDS_REVIEW ? ", needs review" : ", pending");
        }
        return R;
    }

    inline xskeleton::skeleton::joint ToRuntime(const xanimation::joint_limit& L) noexcept
    {
        xskeleton::skeleton::joint J{};
        J.m_Type = static_cast<std::uint8_t>(L.m_Type);
        J.m_SwingY = static_cast<std::uint8_t>(L.m_SwingY); J.m_SwingZ = static_cast<std::uint8_t>(L.m_SwingZ); J.m_Twist = static_cast<std::uint8_t>(L.m_Twist);
        J.m_Frame[0] = L.m_Frame.m_X; J.m_Frame[1] = L.m_Frame.m_Y; J.m_Frame[2] = L.m_Frame.m_Z; J.m_Frame[3] = L.m_Frame.m_W;
        J.m_HingeMin = L.m_HingeMin; J.m_HingeMax = L.m_HingeMax;
        J.m_SwingCy = L.m_SwingCy; J.m_SwingCz = L.m_SwingCz; J.m_SwingRy = L.m_SwingRy; J.m_SwingRz = L.m_SwingRz;
        J.m_TwistCentre = L.m_TwistCentre; J.m_TwistHalf = L.m_TwistHalf;
        return J;
    }
    // a compiled joint back into the evaluator (Rest: the bone's rest rotation from the skeleton's rest array)
    inline xanimation::joint_limit FromRuntime(const xskeleton::skeleton::joint& J, const xmath::fquat& Rest) noexcept
    {
        xanimation::joint_limit L;
        L.m_Type = static_cast<xanimation::joint_type>(J.m_Type);
        L.m_SwingY = static_cast<xanimation::axis_motion>(J.m_SwingY); L.m_SwingZ = static_cast<xanimation::axis_motion>(J.m_SwingZ); L.m_Twist = static_cast<xanimation::axis_motion>(J.m_Twist);
        L.m_Rest = Rest; L.m_Frame = xmath::fquat(J.m_Frame[0], J.m_Frame[1], J.m_Frame[2], J.m_Frame[3]);
        L.m_HingeMin = J.m_HingeMin; L.m_HingeMax = J.m_HingeMax;
        L.m_SwingCy = J.m_SwingCy; L.m_SwingCz = J.m_SwingCz; L.m_SwingRy = J.m_SwingRy; L.m_SwingRz = J.m_SwingRz;
        L.m_TwistCentre = J.m_TwistCentre; L.m_TwistHalf = J.m_TwistHalf;
        return L;
    }

    // The joint section of a descriptor as text (mode, profile, every override, every field) and back: what an undo step of a joint edit
    // stores. Its own format because the generic property snapshot skips enum fields. Floats are written shortest-exact.
    inline std::string JointState(const xskeleton_desc::descriptor& D)
    {
        std::string S = std::format("{}\t{}\n", int(D.m_JointLimits), int(D.m_JointProfile));
        auto Clean = [](std::string T) { for (auto& c : T) if (c == '\t' || c == '\n' || c == '\r') c = ' '; return T; };
        for (const auto& O : D.m_JointOverrides)
            S += std::format("{}\t{}\t{}\t{}\t{} {} {} {}\t{} {}\t{} {} {}\t{} {} {} {}\t{} {}\t{}\t{}\t{}\n", Clean(O.m_BoneName), int(O.m_Source), int(O.m_Geometry), int(O.m_bFrameSet)
                , O.m_Frame.m_X, O.m_Frame.m_Y, O.m_Frame.m_Z, O.m_Frame.m_W, O.m_HingeMin, O.m_HingeMax, int(O.m_SwingY), int(O.m_SwingZ), int(O.m_Twist)
                , O.m_SwingCentre.m_X, O.m_SwingCentre.m_Y, O.m_SwingRadius.m_X, O.m_SwingRadius.m_Y, O.m_TwistCentre, O.m_TwistHalf, int(O.m_Review), Clean(O.m_ReviewedHash), Clean(O.m_Provenance));
        return S;
    }
    inline bool ApplyJointState(xskeleton_desc::descriptor& D, std::string_view S)
    {
        auto Lines = [&] { std::vector<std::string> L; std::size_t a = 0; while (a < S.size()) { auto b = S.find('\n', a); if (b == S.npos) b = S.size(); L.emplace_back(S.substr(a, b - a)); a = b + 1; } return L; }();
        if (Lines.empty()) return false;
        auto Fields = [](const std::string& L) { std::vector<std::string> F; std::size_t a = 0; while (true) { auto b = L.find('\t', a); F.push_back(L.substr(a, b == L.npos ? L.npos : b - a)); if (b == L.npos) break; a = b + 1; } return F; };
        auto Nums = [](const std::string& T) { std::vector<float> V; const char* p = T.data(); const char* e = p + T.size(); while (p < e) { while (p < e && *p == ' ') ++p; if (p >= e) break; float f = 0; auto R = std::from_chars(p, e, f); if (R.ec != std::errc{}) return std::vector<float>{}; V.push_back(f); p = R.ptr; } return V; };
        const auto H = Nums(Fields(Lines[0]).size() == 2 ? Fields(Lines[0])[0] + " " + Fields(Lines[0])[1] : "");
        if (H.size() != 2) return false;
        std::vector<xskeleton_desc::joint_override> V;
        for (std::size_t i = 1; i < Lines.size(); ++i)
        {
            if (Lines[i].empty()) continue;
            const auto F = Fields(Lines[i]);
            if (F.size() != 12) return false;
            xskeleton_desc::joint_override O;
            const auto Fr = Nums(F[4]), Hi = Nums(F[5]), Ax = Nums(F[6]), Sw = Nums(F[7]), Tw = Nums(F[8]), Head = Nums(F[1] + " " + F[2] + " " + F[3] + " " + F[9]);
            if (Fr.size() != 4 || Hi.size() != 2 || Ax.size() != 3 || Sw.size() != 4 || Tw.size() != 2 || Head.size() != 4) return false;
            O.m_BoneName = F[0];
            O.m_Source = static_cast<xskeleton_desc::joint_source>(int(Head[0])); O.m_Geometry = static_cast<xskeleton_desc::joint_geometry>(int(Head[1])); O.m_bFrameSet = Head[2] != 0;
            O.m_Frame = xmath::fquat(Fr[0], Fr[1], Fr[2], Fr[3]);
            O.m_HingeMin = Hi[0]; O.m_HingeMax = Hi[1];
            O.m_SwingY = static_cast<xskeleton_desc::joint_axis>(int(Ax[0])); O.m_SwingZ = static_cast<xskeleton_desc::joint_axis>(int(Ax[1])); O.m_Twist = static_cast<xskeleton_desc::joint_axis>(int(Ax[2]));
            O.m_SwingCentre = { Sw[0], Sw[1] }; O.m_SwingRadius = { Sw[2], Sw[3] }; O.m_TwistCentre = Tw[0]; O.m_TwistHalf = Tw[1];
            O.m_Review = static_cast<xskeleton_desc::joint_review>(int(Head[3])); O.m_ReviewedHash = F[10]; O.m_Provenance = F[11];
            V.push_back(std::move(O));
        }
        D.m_JointLimits = static_cast<xskeleton_desc::joint_limits_mode>(int(H[0])); D.m_JointProfile = static_cast<xskeleton_desc::joint_profile>(int(H[1]));
        D.m_JointOverrides = std::move(V);
        return true;
    }

    // one line per bone, degrees: what JointsInfo and JointLimits.txt show
    inline std::string Describe(const xanimation::joint_limit& L)
    {
        constexpr float D = 1.0f / deg_v;
        switch (L.m_Type)
        {
        case xanimation::joint_type::FREE:  return "free";
        case xanimation::joint_type::FIXED: return "fixed";
        case xanimation::joint_type::HINGE: return std::format("hinge {:.1f}..{:.1f}", L.m_HingeMin * D, L.m_HingeMax * D);
        default:
        {
            static constexpr const char* M[] = { "free", "limited", "locked" };
            return std::format("ball swing y {} {:.1f} r {:.1f}, z {} {:.1f} r {:.1f}; twist {} {:.1f} +- {:.1f}", M[int(L.m_SwingY)], L.m_SwingCy * D, L.m_SwingRy * D
                , M[int(L.m_SwingZ)], L.m_SwingCz * D, L.m_SwingRz * D, M[int(L.m_Twist)], L.m_TwistCentre * D, L.m_TwistHalf * D);
        }
        }
    }
}
#endif
