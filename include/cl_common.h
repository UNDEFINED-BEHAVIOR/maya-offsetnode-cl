/**
  Contains various helper functions.
*/

#ifndef CVWRAP_COMMON_H
#define CVWRAP_COMMON_H
#define _SHOW_EXEC_PATH

#define VEXCL_SHOW_KERNELS

// blank out unsupported defines
#define CL_EXT_SUFFIX__VERSION_1_2  
// Maya clew only supports up to 1_1
#define BOOST_COMPUTE_MAX_CL_VERSION 101

// #define USE_VEXCL

// pre include everything from maya clew to override all cl headers
// including everythign early
#include <clew/clew_cl.h>
#include <clew/clew_cl_platform.h>

#ifdef USE_VEXCL
#include <vexcl/vexcl.hpp>
#endif

#endif
