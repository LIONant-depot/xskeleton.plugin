include(${CMAKE_BINARY_DIR}/_deps/xcmake_tools/Common.cmake)

# Fetch dependencies
FetchAndPopulate("https://github.com/LIONant-depot/xbits.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xserializer.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xraw3d.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xresource_pipeline_v2.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xstrtool.git" "main" )
FetchAndPopulate("https://github.com/LIONant-depot/xscheduler.git" "main")


DefineInterfaceComponent(xskeleton_compiler_ "dependencies"
  "source/xskeleton_details.h"
  "source/xskeleton_descriptor.h"
  "source/xskeleton.h"
  "source/Compiler/xskeleton_compiler.cpp"
  "source/Compiler/xskeleton_compiler.h"
  "**XGPU"
  "source/xskeleton_xgpu_rsc_loader.h"
  "source/xskeleton_xgpu_runtime.h"
)
