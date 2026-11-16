#ifndef XSKELETON_MANIFEST_LIMITS_H
#define XSKELETON_MANIFEST_LIMITS_H
#pragma once
// A skeleton's joint limits read back from its bone manifest as xanimation::joint_limit (the evaluator's form): what every consumer of
// the manifest uses (an anim package's baked IK keys, the ragdoll's joints). Empty: the skeleton's JointLimits are Off, or the manifest
// is older than the joint export.
#include "dependencies/xAnimation/source/ik/xanimation_joint_limits.h"
#include "xskeleton_bone_manifest.h"
#include <vector>

namespace xskeleton_desc
{
    inline std::vector<xanimation::joint_limit> ManifestLimits(const bone_manifest& M)
    {
        std::vector<xanimation::joint_limit> L;
        if (M.m_Joints.size() != M.m_Bones.size()) return L;
        L.resize(M.m_Joints.size());
        for (std::size_t i = 0; i < L.size(); ++i)
        {
            const auto& J = M.m_Joints[i]; const auto& B = M.m_Bones[i]; auto& O = L[i];
            O.m_Type   = static_cast<xanimation::joint_type>(J.m_Type);
            O.m_SwingY = static_cast<xanimation::axis_motion>(J.m_SwingY); O.m_SwingZ = static_cast<xanimation::axis_motion>(J.m_SwingZ); O.m_Twist = static_cast<xanimation::axis_motion>(J.m_Twist);
            O.m_Rest   = xmath::fquat(B.m_RestRotX, B.m_RestRotY, B.m_RestRotZ, B.m_RestRotW).NormalizeSafeCopy();
            O.m_Frame  = xmath::fquat(J.m_FrameX, J.m_FrameY, J.m_FrameZ, J.m_FrameW);
            O.m_HingeMin = J.m_HingeMin; O.m_HingeMax = J.m_HingeMax;
            O.m_SwingCy = J.m_SwingCy; O.m_SwingCz = J.m_SwingCz; O.m_SwingRy = J.m_SwingRy; O.m_SwingRz = J.m_SwingRz;
            O.m_TwistCentre = J.m_TwistCentre; O.m_TwistHalf = J.m_TwistHalf;
            O.m_Relaxed = xmath::fquat(J.m_RelaxedX, J.m_RelaxedY, J.m_RelaxedZ, J.m_RelaxedW).NormalizeSafeCopy(); O.m_Tone = J.m_Tone;
        }
        return L;
    }
}
#endif
