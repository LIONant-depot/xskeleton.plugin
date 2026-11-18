#ifndef XSKELETON_JOINT_EVIDENCE_H
#define XSKELETON_JOINT_EVIDENCE_H
#pragma once
// Joint evidence (joint limits design v2, J4: "clips are evidence, not authority"): what an AnimPackage's clips do with each joint of its
// skeleton - every frame's local rotation per bone - written by the package compile next to its log (JointEvidence.bin), read by the
// Skeleton Editor's "Analyze clips". It never changes the skeleton by itself: an analysis only proposes, the user applies.
// Keyed by the skeleton guid and the bone names (in the skeleton's compiled order): a skeleton whose bones changed since makes the
// evidence stale (refused).
#include "dependencies/xmath/source/xmath.h"
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace xskeleton_joint_evidence
{
    struct clip
    {
        std::string                 m_Name;
        int                         m_nFrames = 0;
        std::vector<xmath::fquat>   m_Rotations;    // m_nFrames x nBones, frame major
    };
    struct evidence
    {
        std::uint64_t               m_Skeleton = 0;
        std::vector<std::string>    m_Bones;
        std::vector<clip>           m_Clips;
    };

    inline constexpr std::uint32_t magic_v = 0x31454A58;     // "XJE1"

    inline bool Write(const std::wstring& Path, const evidence& E)
    {
        std::FILE* f = nullptr; if (_wfopen_s(&f, Path.c_str(), L"wb") != 0) f = nullptr;      // (Windows-only code: its own _s form)
        if (!f) return false;
        auto W = [&](const void* p, std::size_t n) { std::fwrite(p, 1, n, f); };
        auto Str = [&](const std::string& S) { const auto n = static_cast<std::uint16_t>(std::min<std::size_t>(S.size(), 0xFFFF)); W(&n, 2); W(S.data(), n); };
        W(&magic_v, 4); W(&E.m_Skeleton, 8);
        const auto nB = static_cast<std::uint32_t>(E.m_Bones.size()); W(&nB, 4);
        for (const auto& B : E.m_Bones) Str(B);
        const auto nC = static_cast<std::uint32_t>(E.m_Clips.size()); W(&nC, 4);
        for (const auto& C : E.m_Clips)
        {
            Str(C.m_Name);
            const auto nF = static_cast<std::uint32_t>(C.m_nFrames); W(&nF, 4);
            for (const auto& Q : C.m_Rotations) { const float v[4] = { Q.m_X, Q.m_Y, Q.m_Z, Q.m_W }; W(v, 16); }
        }
        const bool Ok = std::ferror(f) == 0;
        std::fclose(f);
        return Ok;
    }

    inline bool Read(const std::wstring& Path, evidence& E)
    {
        E = {};
        std::FILE* f = nullptr; if (_wfopen_s(&f, Path.c_str(), L"rb") != 0) f = nullptr;      // (Windows-only code: its own _s form)
        if (!f) return false;
        bool Ok = true;
        auto R = [&](void* p, std::size_t n) { if (Ok && std::fread(p, 1, n, f) != n) Ok = false; };
        auto Str = [&](std::string& S) { std::uint16_t n = 0; R(&n, 2); S.resize(Ok ? n : 0); if (n) R(S.data(), n); };
        std::uint32_t Magic = 0; R(&Magic, 4); Ok = Ok && Magic == magic_v;
        R(&E.m_Skeleton, 8);
        std::uint32_t nB = 0; R(&nB, 4); if (nB > 0xFFFF) Ok = false;
        if (Ok) { E.m_Bones.resize(nB); for (auto& B : E.m_Bones) Str(B); }
        std::uint32_t nC = 0; R(&nC, 4); if (nC > 0xFFFF) Ok = false;
        for (std::uint32_t c = 0; Ok && c < nC; ++c)
        {
            auto& C = E.m_Clips.emplace_back();
            Str(C.m_Name);
            std::uint32_t nF = 0; R(&nF, 4); if (nF > 1000000) { Ok = false; break; }
            C.m_nFrames = int(nF);
            C.m_Rotations.resize(std::size_t(nF) * nB);
            for (auto& Q : C.m_Rotations) { float v[4]; R(v, 16); Q = xmath::fquat(v[0], v[1], v[2], v[3]); }
        }
        std::fclose(f);
        if (!Ok) E = {};
        return Ok;
    }
}
#endif
