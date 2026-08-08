#ifndef XSKELETON_XGPU_RSC_LOADER_H
#define XSKELETON_XGPU_RSC_LOADER_H
#pragma once

#include "dependencies/xresource_mgr/source/xresource_mgr.h"
#include "xskeleton.h"

// All the information about the resource
namespace xrsc
{
    inline static constexpr auto    skeleton_type_guid_v    = xresource::type_guid(xresource::guid_generator::Instance64FromString("xSkeleton"));
    using                           skeleton                = xresource::def_guid<skeleton_type_guid_v>;
}

// Now we specify the loader and we must fill in all the information.
// Unlike xgeom_static, a skeleton owns no GPU buffers - it's reference data (hierarchy, bind pose,
// masks) consumed by the CPU-side animation system. So there's no xgpu-wrapper subclass here, no
// device buffer creation in Load(), and nothing to release beyond the resource's own memory in
// Destroy() - the data_type is xskeleton::skeleton itself.
template<>
struct xresource::loader< xrsc::skeleton_type_guid_v >
{
    //--- Expected static parameters ---
    constexpr static inline auto            type_name_v         = L"Skeleton";     // This name is used to construct the path to the resource (if not provided)
    constexpr static inline auto            use_death_march_v   = false;           // xGPU already has a death march implemented inside itself...
    using                                   data_type           = xskeleton::skeleton;

    static data_type*                       Load        (xresource::mgr& Mgr, const full_guid& GUID);
    static void                             Destroy     (xresource::mgr& Mgr, data_type&& Data, const full_guid& GUID);
};

#endif
