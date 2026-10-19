#ifndef XSKELETON_COMPILER_H
#define XSKELETON_COMPILER_H
#pragma once

#include "dependencies/xresource_pipeline_v2/source/xresource_pipeline.h"

namespace xskeleton_compiler
{
    // The compiler's own checks on synthetic rigs (no project needed): xskeleton_compiler.exe -SELFTEST. Returns the number of failures.
    int SelfTest();

    enum class state : std::uint8_t
    { OK
    , FAILURE
    };

    struct instance : xresource_pipeline::compiler::base
    {
        static std::unique_ptr<instance> Create(void);
    };
}

#endif
