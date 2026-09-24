#include "xskeleton.h"
#include "xskeleton_xgpu_rsc_loader.h"

#include "dependencies/xresource_guid/source/bridges/xresource_xproperty_bridge.h"

//
// Register the loader and the properties
//
inline static auto s_SkeletonRegistrations = xresource::common_registrations<xrsc::skeleton_type_guid_v>{};

//------------------------------------------------------------------
// A skeleton is pure CPU reference data (hierarchy, bind pose, masks) - unlike xgeom_static, there's
// no device to create buffers on and no default-material-instance references to resolve, so Load()
// is just a deserialize.

xresource::loader< xrsc::skeleton_type_guid_v >::data_type* xresource::loader< xrsc::skeleton_type_guid_v >::Load(xresource::mgr& Mgr, const full_guid& GUID)
{
    std::wstring           Path       = Mgr.getResourcePath(GUID, type_name_v);
    xskeleton::skeleton*   pSkeleton  = nullptr;

    // A missing/not-yet-compiled resource is an expected, recoverable case (same reasoning as
    // xtexture_xgpu_rsc_loader.cpp's identical fix) - every caller already handles getResource()
    // returning null. Unlike the geometry loaders, nothing here dereferences pSkeleton before
    // returning it, so this was never actually unsafe in Release - just an unconditional Debug abort
    // for an ordinary, expected condition.
    xserializer::stream Stream;
    if (auto Err = Stream.Load(Path, pSkeleton); Err)
    {
        return nullptr;
    }

    return pSkeleton;
}

//------------------------------------------------------------------

void xresource::loader< xrsc::skeleton_type_guid_v >::Destroy(xresource::mgr& Mgr, data_type&& Data, const full_guid& GUID)
{
    xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, &Data);
}
