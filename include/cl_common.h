/**
  Contains various helper functions.
*/

#ifndef CVWRAP_COMMON_H
#define CVWRAP_COMMON_H
#define _SHOW_EXEC_PATH

#define VEXCL_SHOW_KERNELS
#define VEXCL_SHOW_COPIES
// #define VEXCL_CACHE_KERNELS
#define BOOST_COMPUTE_DEBUG_KERNEL_COMPILATION

// blank out unsupported defines
#define CL_EXT_SUFFIX__VERSION_1_2  
// Maya clew only supports up to 1_1
#define BOOST_COMPUTE_MAX_CL_VERSION 101

#define USE_VEXCL

#define VEXCL_MAYA_DEBUG

#include <maya/MStreamUtils.h>


// pre include everything from maya clew to override all cl headers
// including everythign early

#include <clew/clew_cl.h>
#include <clew/clew_cl_platform.h>

#ifdef USE_VEXCL

#define cout cout_____
#define cerr cerr_____
#include <iostream>
#undef cout
#undef cerr

namespace std {
  ostream& cout = MStreamUtils::stdOutStream();
  ostream& cerr = MStreamUtils::stdErrorStream();
}

// #define std::cout MStreamUtils::stdOutStream()
// #define std::cerr MStreamUtils::stdErrorStream()
#include <boost/compute.hpp>
// #undef std::cout
// #undef std::cerr
#include <vexcl/vexcl.hpp>

#endif

#endif
