cmake_minimum_required (VERSION 3.18.4)

project (vectorscan C CXX)

set (HS_MAJOR_VERSION 5)
set (HS_MINOR_VERSION 4)
set (HS_PATCH_VERSION 11)
set (HS_VERSION ${HS_MAJOR_VERSION}.${HS_MINOR_VERSION}.${HS_PATCH_VERSION})

string (TIMESTAMP BUILD_DATE "%Y-%m-%d")
message(STATUS "Build date: ${BUILD_DATE}")

# Dependencies check

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
include(CheckCXXSymbolExists)
INCLUDE (CheckFunctionExists)
INCLUDE (CheckIncludeFiles)
INCLUDE (CheckIncludeFileCXX)
INCLUDE (CheckLibraryExists)
INCLUDE (CheckSymbolExists)
include (CMakeDependentOption)
include (GNUInstallDirs)
include (${CMAKE_MODULE_PATH}/boost.cmake)
include (${CMAKE_MODULE_PATH}/ragel.cmake)

find_package(PkgConfig QUIET)

find_program(RAGEL ragel)

if(${RAGEL} STREQUAL "RAGEL-NOTFOUND")
    message(FATAL_ERROR "Ragel state machine compiler not found")
endif()

# Add ccache to speed builds
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

# Build type check

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Default build type 'Release with debug info'")
    set(CMAKE_BUILD_TYPE RELWITHDEBINFO CACHE STRING "" FORCE )
else()
    string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
    message(STATUS "Build type ${CMAKE_BUILD_TYPE}")
endif()

if(CMAKE_BUILD_TYPE MATCHES NONE|RELEASE|RELWITHDEBINFO|MINSIZEREL)
    message(STATUS "using release build")
    set(RELEASE_BUILD TRUE)
else()
    set(RELEASE_BUILD FALSE)
endif()

set(BINDIR "${PROJECT_BINARY_DIR}/bin")
set(LIBDIR "${PROJECT_BINARY_DIR}/lib")

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

# First for the generic no-config case
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${LIBDIR}")
# Second, for multi-config builds (e.g. msvc)
foreach (OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string (TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${BINDIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${LIBDIR}")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${LIBDIR}")
endforeach (OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

set(CMAKE_INCLUDE_CURRENT_DIR 1)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})
include_directories(SYSTEM include)

# Compiler detection

include (${CMAKE_MODULE_PATH}/compiler.cmake)

# CMake options

if (BUILD_STATIC_AND_SHARED)
    message(FATAL_ERROR "This option is no longer supported, please set at least one of BUILD_STATIC_LIBS and BUILD_SHARED_LIBS")
endif()

option(BUILD_SHARED_LIBS "Build shared libs" OFF)
option(BUILD_STATIC_LIBS "Build static libs" OFF)

if (BUILD_SHARED_LIBS)
    message(STATUS "Building shared libraries")
endif()
if (BUILD_STATIC_LIBS)
    message(STATUS "Building static libraries")
endif()

if (NOT BUILD_STATIC_LIBS AND NOT BUILD_SHARED_LIBS)
    # if none are set build static libs
    message(STATUS "Neither shared nor static libraries were requested, building static libraries")
    set(BUILD_STATIC_LIBS ON)
endif ()

CMAKE_DEPENDENT_OPTION(DUMP_SUPPORT "Dump code support; normally on, except in release builds" ON "NOT RELEASE_BUILD" OFF)
CMAKE_DEPENDENT_OPTION(DISABLE_ASSERTS "Disable assert(); Asserts are enabled in debug builds, disabled in release builds" OFF "NOT RELEASE_BUILD" ON)

option(DEBUG_OUTPUT "Enable debug output (warning: very verbose)" OFF)
if(DEBUG_OUTPUT)
    add_definitions(-DDEBUG)
    set(RELEASE_BUILD FALSE)
endif(DEBUG_OUTPUT)


#for config
if (RELEASE_BUILD)
    set(HS_OPTIMIZE ON)
    add_definitions(-DNDEBUG)
endif()

# Architecture detection

include (${CMAKE_MODULE_PATH}/platform.cmake)

# Detect OS and if Fat Runtime is available
include (${CMAKE_MODULE_PATH}/osdetection.cmake)

if(ARCH_X86_64 AND BUILD_SSE2_SIMDE AND NOT FAT_RUNTIME)
    set(SIMDE_BACKEND True)
endif()

if(SIMDE_BACKEND)
    include (${CMAKE_MODULE_PATH}/simde.cmake)
elseif (ARCH_IA32 OR ARCH_X86_64)
    include (${CMAKE_MODULE_PATH}/cflags-x86.cmake)
elseif (ARCH_ARM32 OR ARCH_AARCH64)
    include (${CMAKE_MODULE_PATH}/cflags-arm.cmake)
elseif (ARCH_PPC64EL)
    include (${CMAKE_MODULE_PATH}/cflags-ppc64le.cmake)
    
    # fix unit-internal seg fault
    if (CMAKE_SYSTEM_NAME MATCHES "FreeBSD" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
    endif()

else ()
    message(FATAL_ERROR "Unsupported platform")
endif ()

if (ARCH_PPC64EL)
    set(ARCH_FLAG mcpu)
else ()
    set(ARCH_FLAG march)
endif ()

# Detect Native arch flags if requested
include (${CMAKE_MODULE_PATH}/archdetect.cmake)

# Configure Compiler flags (Generic)

include (${CMAKE_MODULE_PATH}/sanitize.cmake)

if (NOT FAT_RUNTIME)
    if (GNUCC_TUNE)
        message(STATUS "GNUCC_TUNE is set")
        set(ARCH_C_FLAGS "-${ARCH_FLAG}=${GNUCC_ARCH} -${TUNE_FLAG}=${GNUCC_TUNE}")
        set(ARCH_CXX_FLAGS "-${ARCH_FLAG}=${GNUCC_ARCH} -${TUNE_FLAG}=${GNUCC_TUNE}")
    else()
        message(STATUS "GNUCC_TUNE is not set")
        set(ARCH_C_FLAGS "-${ARCH_FLAG}=${GNUCC_ARCH} -mtune=${TUNE_FLAG} ${ARCH_C_FLAGS}")
        set(ARCH_CXX_FLAGS "-${ARCH_FLAG}=${GNUCC_ARCH} -mtune=${TUNE_FLAG} ${ARCH_CXX_FLAGS}")
    endif()
endif()

# remove CMake's idea of optimisation
foreach (CONFIG ${CMAKE_BUILD_TYPE} ${CMAKE_CONFIGURATION_TYPES})
    string(REGEX REPLACE "-O[^ ]*" "" CMAKE_C_FLAGS_${CONFIG} "${CMAKE_C_FLAGS_${CONFIG}}")
    string(REGEX REPLACE "-O[^ ]*" "" CMAKE_CXX_FLAGS_${CONFIG} "${CMAKE_CXX_FLAGS_${CONFIG}}")
endforeach ()

message(STATUS "ARCH_C_FLAGS   : ${ARCH_C_FLAGS}")
message(STATUS "ARCH_CXX_FLAGS : ${ARCH_CXX_FLAGS}")

if(RELEASE_BUILD)
    if (NOT CMAKE_BUILD_TYPE MATCHES MINSIZEREL)
        set(OPT_C_FLAG "-O3")
        set(OPT_CXX_FLAG "-O3")
    else ()
        set(OPT_C_FLAG "-Os")
        set(OPT_CXX_FLAG "-Os")
    endif ()
else()
    set(OPT_C_FLAG "-O0")
    set(OPT_CXX_FLAG "-O0")
endif(RELEASE_BUILD)

include (${CMAKE_MODULE_PATH}/cflags-generic.cmake)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ARCH_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARCH_CXX_FLAGS}")

# PCRE check, we have a fixed requirement for PCRE to use Chimera
# and hscollider
set(PCRE_REQUIRED_MAJOR_VERSION 8)
set(PCRE_REQUIRED_MINOR_VERSION 41)
set(PCRE_REQUIRED_VERSION ${PCRE_REQUIRED_MAJOR_VERSION}.${PCRE_REQUIRED_MINOR_VERSION})
include (${CMAKE_MODULE_PATH}/pcre.cmake)
if (NOT CORRECT_PCRE_VERSION)
    message(STATUS "PCRE ${PCRE_REQUIRED_VERSION} or above not found")
endif()

# we need static libs for Chimera - too much deep magic for shared libs
if (CORRECT_PCRE_VERSION AND PCRE_BUILD_SOURCE AND BUILD_STATIC_LIBS)
    set(BUILD_CHIMERA TRUE)
endif()

set(RAGEL_C_FLAGS "-Wno-unused -funsigned-char")

set_source_files_properties(
    src/parser/Parser.cpp
    PROPERTIES
        COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(src/parser/Parser.rl)

set_source_files_properties(
   src/parser/control_verbs.cpp
    PROPERTIES
        COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(src/parser/control_verbs.rl)

# BSD has the _np funcs in a _np header
CHECK_INCLUDE_FILE_CXX(pthread_np.h HAVE_PTHREAD_NP_H)
if (HAVE_PTHREAD_NP_H)
    set (PTHREAD_NP_INC pthread_np.h)
else ()
    set (PTHREAD_NP_INC pthread.h)
endif ()
CHECK_CXX_SYMBOL_EXISTS(pthread_setaffinity_np ${PTHREAD_NP_INC} HAVE_DECL_PTHREAD_SETAFFINITY_NP)

CHECK_FUNCTION_EXISTS(malloc_info HAVE_MALLOC_INFO)
CHECK_FUNCTION_EXISTS(shmget HAVE_SHMGET)
set(HAVE_SHMGET ${HAVE_SHMGET} CACHE BOOL "shmget()")

# do substitutions
configure_file(${CMAKE_MODULE_PATH}/config.h.in ${PROJECT_BINARY_DIR}/config.h)
configure_file(src/hs_version.h.in ${PROJECT_BINARY_DIR}/hs_version.h)

configure_file(libhs.pc.in libhs.pc @ONLY) # only replace @ quoted vars
install(FILES ${CMAKE_BINARY_DIR}/libhs.pc
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

# only set these after all tests are done
if (NOT FAT_RUNTIME)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS} ${HS_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS} ${HS_CXX_FLAGS}")
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")
endif()

SET(hs_HEADERS
    ${PROJECT_BINARY_DIR}/hs_version.h
    src/hs.h
    src/hs_common.h
    src/hs_compile.h
    src/hs_runtime.h
)
install(FILES ${hs_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hs")

set (hs_exec_common_SRCS
    src/alloc.c
    src/scratch.c
    src/util/arch/common/cpuid_flags.h
    src/util/multibit.c
    )
if (SIMDE_BACKEND)
set (hs_exec_common_SRCS
    ${hs_exec_common_SRCS}
    src/util/arch/simde/cpuid_flags.c)
elseif (ARCH_IA32 OR ARCH_X86_64)
set (hs_exec_common_SRCS
    ${hs_exec_common_SRCS}
    src/util/arch/x86/cpuid_flags.c
    )
elseif (ARCH_ARM32 OR ARCH_AARCH64)
set (hs_exec_common_SRCS
    ${hs_exec_common_SRCS}
    src/util/arch/arm/cpuid_flags.c
    )
elseif (ARCH_PPC64EL)
set (hs_exec_common_SRCS
    ${hs_exec_common_SRCS}
    src/util/arch/ppc64el/cpuid_flags.c)
endif ()

set (hs_exec_SRCS
    ${hs_HEADERS}
    src/hs_version.h.in
    src/ue2common.h
    src/allocator.h
    src/crc32.c
    src/crc32.h
    src/report.h
    src/runtime.c
    src/stream_compress.c
    src/stream_compress.h
    src/stream_compress_impl.h
    src/fdr/fdr.c
    src/fdr/fdr.h
    src/fdr/fdr_internal.h
    src/fdr/fdr_confirm.h
    src/fdr/fdr_confirm_runtime.h
    src/fdr/flood_runtime.h
    src/fdr/fdr_loadval.h
    src/fdr/teddy.cpp
    src/fdr/teddy.h
    src/fdr/teddy_internal.h
    src/fdr/teddy_runtime_common.h
    src/hwlm/hwlm.c
    src/hwlm/hwlm.h
    src/hwlm/hwlm_internal.h
    src/hwlm/noodle_engine.cpp
    src/hwlm/noodle_engine.h
    src/hwlm/noodle_internal.h
    src/nfa/accel.c
    src/nfa/accel.h
    src/nfa/castle.c
    src/nfa/castle.h
    src/nfa/castle_internal.h
    src/nfa/gough.c
    src/nfa/gough_internal.h
    src/nfa/lbr.c
    src/nfa/lbr.h
    src/nfa/lbr_common_impl.h
    src/nfa/lbr_internal.h
    src/nfa/limex_accel.c
    src/nfa/limex_accel.h
    src/nfa/limex_exceptional.h
    src/nfa/limex_native.c
    src/nfa/limex_ring.h
    src/nfa/limex_64.c
    src/nfa/limex_simd128.c
    src/nfa/limex_simd256.c
    src/nfa/limex_simd384.c
    src/nfa/limex_simd512.c
    src/nfa/limex.h
    src/nfa/limex_common_impl.h
    src/nfa/limex_context.h
    src/nfa/limex_internal.h
    src/nfa/limex_runtime.h
    src/nfa/limex_runtime_impl.h
    src/nfa/limex_shuffle.h
    src/nfa/limex_state_impl.h
    src/nfa/mcclellan.c
    src/nfa/mcclellan.h
    src/nfa/mcclellan_common_impl.h
    src/nfa/mcclellan_internal.h
    src/nfa/mcsheng.c
    src/nfa/mcsheng_data.c
    src/nfa/mcsheng.h
    src/nfa/mcsheng_internal.h
    src/nfa/mpv.h
    src/nfa/mpv.c
    src/nfa/mpv_internal.h
    src/nfa/nfa_api.h
    src/nfa/nfa_api_dispatch.c
    src/nfa/nfa_internal.h
    src/nfa/nfa_rev_api.h
    src/nfa/repeat.c
    src/nfa/repeat.h
    src/nfa/repeat_internal.h
    src/nfa/sheng.c
    src/nfa/sheng.h
    src/nfa/sheng_defs.h
    src/nfa/sheng_impl.h
    src/nfa/sheng_impl4.h
    src/nfa/sheng_internal.h
    src/nfa/shufti.cpp
    src/nfa/shufti.h
    src/nfa/tamarama.c
    src/nfa/tamarama.h
    src/nfa/tamarama_internal.h
    src/nfa/truffle.cpp
    src/nfa/truffle.h
    src/nfa/vermicelli.hpp
    src/nfa/vermicelli_run.h
    src/som/som.h
    src/som/som_operation.h
    src/som/som_runtime.h
    src/som/som_runtime.c
    src/som/som_stream.c
    src/som/som_stream.h
    src/rose/block.c
    src/rose/catchup.h
    src/rose/catchup.c
    src/rose/infix.h
    src/rose/init.h
    src/rose/init.c
    src/rose/stream.c
    src/rose/stream_long_lit.h
    src/rose/stream_long_lit_hash.h
    src/rose/match.h
    src/rose/match.c
    src/rose/miracle.h
    src/rose/program_runtime.c
    src/rose/program_runtime.h
    src/rose/runtime.h
    src/rose/rose.h
    src/rose/rose_internal.h
    src/rose/rose_program.h
    src/rose/rose_types.h
    src/rose/rose_common.h
    src/rose/validate_mask.h
    src/rose/validate_shufti.h
    src/util/bitutils.h
    src/util/copybytes.h
    src/util/exhaust.h
    src/util/fatbit.h
    src/util/join.h
    src/util/multibit.h
    src/util/multibit.c
    src/util/multibit_compress.h
    src/util/multibit_internal.h
    src/util/pack_bits.h
    src/util/popcount.h
    src/util/pqueue.h
    src/util/scatter.h
    src/util/scatter_runtime.h
    src/util/simd_utils.h
    src/util/state_compress.h
    src/util/state_compress.c
    src/util/unaligned.h
    src/util/uniform_ops.h
    src/database.c
    src/database.h
)

if (SIMDE_BACKEND)
set (hs_exec_SRCS
    ${hs_exec_SRCS}
    src/nfa/vermicelli_simd.cpp
    src/util/supervector/arch/x86/impl.cpp)
elseif (ARCH_IA32 OR ARCH_X86_64)
set (hs_exec_SRCS
    ${hs_exec_SRCS}
    src/nfa/vermicelli_simd.cpp
    src/util/supervector/arch/x86/impl.cpp)
elseif (ARCH_ARM32 OR ARCH_AARCH64)
set (hs_exec_SRCS
    ${hs_exec_SRCS}
    src/util/supervector/arch/arm/impl.cpp)
elseif (ARCH_PPC64EL)
set (hs_exec_SRCS
    ${hs_exec_SRCS}
    src/nfa/vermicelli_simd.cpp
    src/util/supervector/arch/ppc64el/impl.cpp)
endif()


if (ARCH_IA32 OR ARCH_X86_64)
    set (hs_exec_avx2_SRCS
        src/fdr/teddy.cpp
        src/fdr/teddy_fat.cpp
        src/util/arch/x86/masked_move.c
        src/util/arch/x86/masked_move.h
    )
endif()

if (ARCH_ARM32 OR ARCH_AARCH64)
    set (hs_exec_neon_SRCS
        src/nfa/vermicelli_simd.cpp)
    set (hs_exec_sve_SRCS
        src/nfa/vermicelli_simd.cpp)
endif()

SET (hs_compile_SRCS
    ${hs_HEADERS}
    src/crc32.h
    src/database.h
    src/grey.cpp
    src/grey.h
    src/hs.cpp
    src/hs_internal.h
    src/hs_version.h.in
    src/scratch.h
    src/state.h
    src/ue2common.h
    src/compiler/asserts.cpp
    src/compiler/asserts.h
    src/compiler/compiler.cpp
    src/compiler/compiler.h
    src/compiler/error.cpp
    src/compiler/error.h
    src/compiler/expression_info.h
    src/fdr/engine_description.cpp
    src/fdr/engine_description.h
    src/fdr/fdr_compile.cpp
    src/fdr/fdr_compile.h
    src/fdr/fdr_compile_internal.h
    src/fdr/fdr_compile_util.cpp
    src/fdr/fdr_confirm_compile.cpp
    src/fdr/fdr_confirm.h
    src/fdr/fdr_engine_description.cpp
    src/fdr/fdr_engine_description.h
    src/fdr/fdr_internal.h
    src/fdr/flood_compile.cpp
    src/fdr/teddy_compile.cpp
    src/fdr/teddy_compile.h
    src/fdr/teddy_engine_description.cpp
    src/fdr/teddy_engine_description.h
    src/fdr/teddy_internal.h
    src/hwlm/hwlm_build.cpp
    src/hwlm/hwlm_build.h
    src/hwlm/hwlm_internal.h
    src/hwlm/hwlm_literal.cpp
    src/hwlm/hwlm_literal.h
    src/hwlm/noodle_build.cpp
    src/hwlm/noodle_build.h
    src/hwlm/noodle_internal.h
    src/nfa/accel.h
    src/nfa/accel_dfa_build_strat.cpp
    src/nfa/accel_dfa_build_strat.h
    src/nfa/accelcompile.cpp
    src/nfa/accelcompile.h
    src/nfa/callback.h
    src/nfa/castlecompile.cpp
    src/nfa/castlecompile.h
    src/nfa/dfa_build_strat.cpp
    src/nfa/dfa_build_strat.h
    src/nfa/dfa_min.cpp
    src/nfa/dfa_min.h
    src/nfa/goughcompile.cpp
    src/nfa/goughcompile.h
    src/nfa/goughcompile_accel.cpp
    src/nfa/goughcompile_internal.h
    src/nfa/goughcompile_reg.cpp
    src/nfa/mcclellan.h
    src/nfa/mcclellan_internal.h
    src/nfa/mcclellancompile.cpp
    src/nfa/mcclellancompile.h
    src/nfa/mcclellancompile_util.cpp
    src/nfa/mcclellancompile_util.h
    src/nfa/mcsheng_compile.cpp
    src/nfa/mcsheng_compile.h
    src/nfa/limex_compile.cpp
    src/nfa/limex_compile.h
    src/nfa/limex_accel.h
    src/nfa/limex_internal.h
    src/nfa/mpv_internal.h
    src/nfa/mpvcompile.cpp
    src/nfa/mpvcompile.h
    src/nfa/nfa_api.h
    src/nfa/nfa_api_queue.h
    src/nfa/nfa_api_util.h
    src/nfa/nfa_build_util.cpp
    src/nfa/nfa_build_util.h
    src/nfa/nfa_internal.h
    src/nfa/nfa_kind.h
    src/nfa/rdfa.cpp
    src/nfa/rdfa.h
    src/nfa/rdfa_graph.cpp
    src/nfa/rdfa_graph.h
    src/nfa/rdfa_merge.cpp
    src/nfa/rdfa_merge.h
    src/nfa/repeat_internal.h
    src/nfa/repeatcompile.cpp
    src/nfa/repeatcompile.h
    src/nfa/sheng_internal.h
    src/nfa/shengcompile.cpp
    src/nfa/shengcompile.h
    src/nfa/shufticompile.cpp
    src/nfa/shufticompile.h
    src/nfa/tamaramacompile.cpp
    src/nfa/tamaramacompile.h
    src/nfa/trufflecompile.cpp
    src/nfa/trufflecompile.h
    src/nfa/vermicellicompile.cpp
    src/nfa/vermicellicompile.h
    src/nfagraph/ng.cpp
    src/nfagraph/ng.h
    src/nfagraph/ng_anchored_acyclic.cpp
    src/nfagraph/ng_anchored_acyclic.h
    src/nfagraph/ng_anchored_dots.cpp
    src/nfagraph/ng_anchored_dots.h
    src/nfagraph/ng_asserts.cpp
    src/nfagraph/ng_asserts.h
    src/nfagraph/ng_builder.cpp
    src/nfagraph/ng_builder.h
    src/nfagraph/ng_calc_components.cpp
    src/nfagraph/ng_calc_components.h
    src/nfagraph/ng_cyclic_redundancy.cpp
    src/nfagraph/ng_cyclic_redundancy.h
    src/nfagraph/ng_depth.cpp
    src/nfagraph/ng_depth.h
    src/nfagraph/ng_dominators.cpp
    src/nfagraph/ng_dominators.h
    src/nfagraph/ng_edge_redundancy.cpp
    src/nfagraph/ng_edge_redundancy.h
    src/nfagraph/ng_equivalence.cpp
    src/nfagraph/ng_equivalence.h
    src/nfagraph/ng_execute.cpp
    src/nfagraph/ng_execute.h
    src/nfagraph/ng_expr_info.cpp
    src/nfagraph/ng_expr_info.h
    src/nfagraph/ng_extparam.cpp
    src/nfagraph/ng_extparam.h
    src/nfagraph/ng_fixed_width.cpp
    src/nfagraph/ng_fixed_width.h
    src/nfagraph/ng_fuzzy.cpp
    src/nfagraph/ng_fuzzy.h
    src/nfagraph/ng_haig.cpp
    src/nfagraph/ng_haig.h
    src/nfagraph/ng_holder.cpp
    src/nfagraph/ng_holder.h
    src/nfagraph/ng_is_equal.cpp
    src/nfagraph/ng_is_equal.h
    src/nfagraph/ng_lbr.cpp
    src/nfagraph/ng_lbr.h
    src/nfagraph/ng_literal_analysis.cpp
    src/nfagraph/ng_literal_analysis.h
    src/nfagraph/ng_literal_component.cpp
    src/nfagraph/ng_literal_component.h
    src/nfagraph/ng_literal_decorated.cpp
    src/nfagraph/ng_literal_decorated.h
    src/nfagraph/ng_mcclellan.cpp
    src/nfagraph/ng_mcclellan.h
    src/nfagraph/ng_mcclellan_internal.h
    src/nfagraph/ng_limex.cpp
    src/nfagraph/ng_limex.h
    src/nfagraph/ng_limex_accel.cpp
    src/nfagraph/ng_limex_accel.h
    src/nfagraph/ng_misc_opt.cpp
    src/nfagraph/ng_misc_opt.h
    src/nfagraph/ng_netflow.cpp
    src/nfagraph/ng_netflow.h
    src/nfagraph/ng_prefilter.cpp
    src/nfagraph/ng_prefilter.h
    src/nfagraph/ng_prune.cpp
    src/nfagraph/ng_prune.h
    src/nfagraph/ng_puff.cpp
    src/nfagraph/ng_puff.h
    src/nfagraph/ng_redundancy.cpp
    src/nfagraph/ng_redundancy.h
    src/nfagraph/ng_region.cpp
    src/nfagraph/ng_region.h
    src/nfagraph/ng_region_redundancy.cpp
    src/nfagraph/ng_region_redundancy.h
    src/nfagraph/ng_repeat.cpp
    src/nfagraph/ng_repeat.h
    src/nfagraph/ng_reports.cpp
    src/nfagraph/ng_reports.h
    src/nfagraph/ng_restructuring.cpp
    src/nfagraph/ng_restructuring.h
    src/nfagraph/ng_revacc.cpp
    src/nfagraph/ng_revacc.h
    src/nfagraph/ng_sep.cpp
    src/nfagraph/ng_sep.h
    src/nfagraph/ng_small_literal_set.cpp
    src/nfagraph/ng_small_literal_set.h
    src/nfagraph/ng_som.cpp
    src/nfagraph/ng_som.h
    src/nfagraph/ng_som_add_redundancy.cpp
    src/nfagraph/ng_som_add_redundancy.h
    src/nfagraph/ng_som_util.cpp
    src/nfagraph/ng_som_util.h
    src/nfagraph/ng_split.cpp
    src/nfagraph/ng_split.h
    src/nfagraph/ng_squash.cpp
    src/nfagraph/ng_squash.h
    src/nfagraph/ng_stop.cpp
    src/nfagraph/ng_stop.h
    src/nfagraph/ng_uncalc_components.cpp
    src/nfagraph/ng_uncalc_components.h
    src/nfagraph/ng_utf8.cpp
    src/nfagraph/ng_utf8.h
    src/nfagraph/ng_util.cpp
    src/nfagraph/ng_util.h
    src/nfagraph/ng_vacuous.cpp
    src/nfagraph/ng_vacuous.h
    src/nfagraph/ng_violet.cpp
    src/nfagraph/ng_violet.h
    src/nfagraph/ng_width.cpp
    src/nfagraph/ng_width.h
    src/parser/AsciiComponentClass.cpp
    src/parser/AsciiComponentClass.h
    src/parser/Component.cpp
    src/parser/Component.h
    src/parser/ComponentAlternation.cpp
    src/parser/ComponentAlternation.h
    src/parser/ComponentAssertion.cpp
    src/parser/ComponentAssertion.h
    src/parser/ComponentAtomicGroup.cpp
    src/parser/ComponentAtomicGroup.h
    src/parser/ComponentBackReference.cpp
    src/parser/ComponentBackReference.h
    src/parser/ComponentBoundary.cpp
    src/parser/ComponentBoundary.h
    src/parser/ComponentByte.cpp
    src/parser/ComponentByte.h
    src/parser/ComponentClass.cpp
    src/parser/ComponentClass.h
    src/parser/ComponentCondReference.cpp
    src/parser/ComponentCondReference.h
    src/parser/ComponentEUS.cpp
    src/parser/ComponentEUS.h
    src/parser/ComponentEmpty.cpp
    src/parser/ComponentEmpty.h
    src/parser/ComponentRepeat.cpp
    src/parser/ComponentRepeat.h
    src/parser/ComponentSequence.cpp
    src/parser/ComponentSequence.h
    src/parser/ComponentVisitor.cpp
    src/parser/ComponentVisitor.h
    src/parser/ComponentWordBoundary.cpp
    src/parser/ComponentWordBoundary.h
    src/parser/ConstComponentVisitor.cpp
    src/parser/ConstComponentVisitor.h
    src/parser/Parser.cpp
    src/parser/Parser.h
    src/parser/Utf8ComponentClass.cpp
    src/parser/Utf8ComponentClass.h
    src/parser/buildstate.cpp
    src/parser/buildstate.h
    src/parser/check_refs.cpp
    src/parser/check_refs.h
    src/parser/control_verbs.cpp
    src/parser/control_verbs.h
    src/parser/logical_combination.cpp
    src/parser/logical_combination.h
    src/parser/parse_error.cpp
    src/parser/parse_error.h
    src/parser/parser_util.cpp
    src/parser/position.h
    src/parser/position_info.h
    src/parser/prefilter.cpp
    src/parser/prefilter.h
    src/parser/shortcut_literal.cpp
    src/parser/shortcut_literal.h
    src/parser/ucp_table.cpp
    src/parser/ucp_table.h
    src/parser/unsupported.cpp
    src/parser/unsupported.h
    src/parser/utf8_validate.h
    src/parser/utf8_validate.cpp
    src/smallwrite/smallwrite_build.cpp
    src/smallwrite/smallwrite_build.h
    src/smallwrite/smallwrite_internal.h
    src/som/slot_manager.cpp
    src/som/slot_manager.h
    src/som/slot_manager_internal.h
    src/som/som.h
    src/som/som_operation.h
    src/rose/rose_build.h
    src/rose/rose_build_add.cpp
    src/rose/rose_build_add_internal.h
    src/rose/rose_build_add_mask.cpp
    src/rose/rose_build_anchored.cpp
    src/rose/rose_build_anchored.h
    src/rose/rose_build_bytecode.cpp
    src/rose/rose_build_castle.h
    src/rose/rose_build_castle.cpp
    src/rose/rose_build_compile.cpp
    src/rose/rose_build_convert.cpp
    src/rose/rose_build_convert.h
    src/rose/rose_build_dedupe.cpp
    src/rose/rose_build_engine_blob.cpp
    src/rose/rose_build_engine_blob.h
    src/rose/rose_build_exclusive.cpp
    src/rose/rose_build_exclusive.h
    src/rose/rose_build_groups.cpp
    src/rose/rose_build_groups.h
    src/rose/rose_build_impl.h
    src/rose/rose_build_infix.cpp
    src/rose/rose_build_infix.h
    src/rose/rose_build_instructions.cpp
    src/rose/rose_build_instructions.h
    src/rose/rose_build_lit_accel.cpp
    src/rose/rose_build_lit_accel.h
    src/rose/rose_build_long_lit.cpp
    src/rose/rose_build_long_lit.h
    src/rose/rose_build_lookaround.cpp
    src/rose/rose_build_lookaround.h
    src/rose/rose_build_matchers.cpp
    src/rose/rose_build_matchers.h
    src/rose/rose_build_merge.cpp
    src/rose/rose_build_merge.h
    src/rose/rose_build_misc.cpp
    src/rose/rose_build_misc.h
    src/rose/rose_build_program.cpp
    src/rose/rose_build_program.h
    src/rose/rose_build_resources.h
    src/rose/rose_build_role_aliasing.cpp
    src/rose/rose_build_scatter.cpp
    src/rose/rose_build_scatter.h
    src/rose/rose_build_util.h
    src/rose/rose_build_width.cpp
    src/rose/rose_build_width.h
    src/rose/rose_graph.h
    src/rose/rose_in_graph.h
    src/rose/rose_in_util.cpp
    src/rose/rose_in_util.h
    src/util/accel_scheme.h
    src/util/alloc.cpp
    src/util/alloc.h
    src/util/bitfield.h
    src/util/boundary_reports.h
    src/util/charreach.cpp
    src/util/charreach.h
    src/util/charreach_util.h
    src/util/clique.cpp
    src/util/clique.h
    src/util/compare.h
    src/util/compile_context.cpp
    src/util/compile_context.h
    src/util/compile_error.cpp
    src/util/compile_error.h
    src/util/container.h
    src/util/depth.cpp
    src/util/depth.h
    src/util/determinise.h
    src/util/dump_mask.cpp
    src/util/dump_mask.h
    src/util/fatbit_build.cpp
    src/util/fatbit_build.h
    src/util/flat_containers.h
    src/util/graph.h
    src/util/graph_range.h
    src/util/graph_small_color_map.h
    src/util/graph_undirected.h
    src/util/hash.h
    src/util/hash_dynamic_bitset.h
    src/util/insertion_ordered.h
    src/util/math.h
    src/util/multibit_build.cpp
    src/util/multibit_build.h
    src/util/noncopyable.h
    src/util/operators.h
    src/util/order_check.h
    src/util/partial_store.h
    src/util/partitioned_set.h
    src/util/popcount.h
    src/util/queue_index_factory.h
    src/util/report.h
    src/util/report_manager.cpp
    src/util/report_manager.h
    src/util/simd_utils.h
    src/util/small_vector.h
    src/util/target_info.cpp
    src/util/target_info.h
    src/util/ue2_graph.h
    src/util/ue2string.cpp
    src/util/ue2string.h
    src/util/unaligned.h
    src/util/unicode_def.h
    src/util/unicode_set.h
    src/util/uniform_ops.h
    src/util/unordered.h
    src/util/verify_types.h
)

set(hs_dump_SRCS
    src/scratch_dump.cpp
    src/scratch_dump.h
    src/fdr/fdr_dump.cpp
    src/hwlm/hwlm_dump.cpp
    src/hwlm/hwlm_dump.h
    src/nfa/accel_dump.cpp
    src/nfa/accel_dump.h
    src/nfa/castle_dump.cpp
    src/nfa/castle_dump.h
    src/nfagraph/ng_dump.cpp
    src/nfagraph/ng_dump.h
    src/nfa/goughcompile_dump.cpp
    src/nfa/goughcompile_dump.h
    src/nfa/goughdump.cpp
    src/nfa/goughdump.h
    src/nfa/lbr_dump.cpp
    src/nfa/limex_dump.cpp
    src/nfa/mcclellandump.cpp
    src/nfa/mcclellandump.h
    src/nfa/mcsheng_dump.cpp
    src/nfa/mcsheng_dump.h
    src/nfa/mpv_dump.cpp
    src/nfa/nfa_dump_api.h
    src/nfa/nfa_dump_dispatch.cpp
    src/nfa/nfa_dump_internal.cpp
    src/nfa/nfa_dump_internal.h
    src/nfa/shengdump.cpp
    src/nfa/shengdump.h
    src/nfa/tamarama_dump.cpp
    src/nfa/tamarama_dump.h
    src/parser/dump.cpp
    src/parser/dump.h
    src/parser/position_dump.h
    src/smallwrite/smallwrite_dump.cpp
    src/smallwrite/smallwrite_dump.h
    src/som/slot_manager_dump.cpp
    src/som/slot_manager_dump.h
    src/rose/rose_build_dump.cpp
    src/rose/rose_build_dump.h
    src/rose/rose_in_dump.cpp
    src/rose/rose_in_dump.h
    src/util/dump_charclass.cpp
    src/util/dump_charclass.h
    src/util/dump_util.cpp
    src/util/dump_util.h
)

if (DUMP_SUPPORT)
    set(hs_compile_SRCS ${hs_compile_SRCS} ${hs_dump_SRCS})
endif()

# we group things by sublibraries, specifying shared and static and then
# choose which ones to build

set (LIB_VERSION ${HS_VERSION})
set (LIB_SOVERSION ${HS_MAJOR_VERSION})

if (NOT FAT_RUNTIME)
    set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_common_SRCS})

    if (ARCH_IA32 OR ARCH_X86_64)
        if (BUILD_AVX2)
            set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
        endif()
    elseif (ARCH_AARCH64)
        if (BUILD_SVE2)
            set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_sve2_SRCS})
        elseif (BUILD_SVE)
            set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_sve_SRCS})
        else()
            set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_neon_SRCS})
        endif()
    endif()

    if (BUILD_STATIC_LIBS)
        add_library(hs_exec OBJECT ${hs_exec_SRCS})

        add_library(hs_runtime STATIC src/hs_version.c src/hs_valid_platform.c $<TARGET_OBJECTS:hs_exec>)
        set_target_properties(hs_runtime PROPERTIES LINKER_LANGUAGE C)

        add_library(hs_compile OBJECT ${hs_compile_SRCS})
        add_library(hs STATIC
            src/hs_version.c
            src/hs_valid_platform.c
            $<TARGET_OBJECTS:hs_exec>
            $<TARGET_OBJECTS:hs_compile>)
    endif (BUILD_STATIC_LIBS)

    if (BUILD_SHARED_LIBS)
        add_library(hs_exec_shared OBJECT ${hs_exec_SRCS})
        set_target_properties(hs_exec_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
        add_library(hs_compile_shared OBJECT ${hs_compile_SRCS})
        set_target_properties(hs_compile_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
    endif()
else ()
    if (ARCH_IA32 OR ARCH_X86_64)
        set(BUILD_WRAPPER "${PROJECT_SOURCE_DIR}/cmake/build_wrapper.sh")
        if (NOT BUILD_AVX512)
            set (DISPATCHER_DEFINE "-DDISABLE_AVX512_DISPATCH")
        endif (NOT BUILD_AVX512)
        if (NOT BUILD_AVX512VBMI)
            set (DISPATCHER_DEFINE "${DISPATCHER_DEFINE} -DDISABLE_AVX512VBMI_DISPATCH")
        endif (NOT BUILD_AVX512VBMI)
        if(BUILD_SSE2_SIMDE)
            set (DISPATCHER_DEFINE "${DISPATCHER_DEFINE} -DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2")
        endif(BUILD_SSE2_SIMDE)
        set_source_files_properties(src/dispatcher.c PROPERTIES
            COMPILE_FLAGS "-Wno-unused-parameter -Wno-unused-function ${DISPATCHER_DEFINE}")
        if(BUILD_SSE2_SIMDE AND NOT BUILD_AVX2 AND NOT BUILD_AVX512 AND NOT BUILD_AVX512VBMI)
            set_source_files_properties(src/crc32.c PROPERTIES
                COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2")
            set_source_files_properties(src/hs.cpp PROPERTIES
                COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2")

            string(REGEX REPLACE "-msse4.2" "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
            string(REGEX REPLACE "-msse4.2" "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        endif()

        if (BUILD_STATIC_LIBS)

            if (BUILD_SSE2_SIMDE)
              add_library(hs_exec_core2 OBJECT ${hs_exec_SRCS})
              list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_core2>)
              message("Building SIMDE SSE2 version..")
              include_directories(${PROJECT_SOURCE_DIR}/simde)
              if (CMAKE_COMPILER_IS_CLANG)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSIMDE_NO_CHECK_IMMEDIATE_CONSTANT")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSIMDE_NO_CHECK_IMMEDIATE_CONSTANT")
              endif()

              set_target_properties(hs_exec_core2 PROPERTIES
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                  COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2"
                  RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                  )
            else()
              add_library(hs_exec_core2 OBJECT ${hs_exec_SRCS})
              list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_core2>)
              set_target_properties(hs_exec_core2 PROPERTIES
                  COMPILE_FLAGS "-march=core2 -msse4.2"
                  RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                  )
	    endif(BUILD_SSE2_SIMDE)

            add_library(hs_exec_corei7 OBJECT ${hs_exec_SRCS})
            list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_corei7>)
            set_target_properties(hs_exec_corei7 PROPERTIES
                COMPILE_FLAGS "-march=corei7 -msse4.2"
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} corei7 ${CMAKE_MODULE_PATH}/keep.syms.in"
                )

            if (BUILD_AVX2)
                add_library(hs_exec_avx2 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_avx2>)
                set_target_properties(hs_exec_avx2 PROPERTIES
                    COMPILE_FLAGS "-march=core-avx2 -mavx2"
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX2)
            if (BUILD_AVX512)
                add_library(hs_exec_avx512 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_avx512>)
                set_target_properties(hs_exec_avx512 PROPERTIES
                    COMPILE_FLAGS "${SKYLAKE_FLAG}"
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512 ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX512)
            if (BUILD_AVX512VBMI)
                add_library(hs_exec_avx512vbmi OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_avx512vbmi>)
                set_target_properties(hs_exec_avx512vbmi PROPERTIES
                    COMPILE_FLAGS "${ICELAKE_FLAG}"
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512vbmi ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX512VBMI)

            add_library(hs_exec_common OBJECT
                ${hs_exec_common_SRCS}
                src/dispatcher.c
                )

            # hs_version.c is added explicitly to avoid some build systems that refuse to
            # create a lib without any src (I'm looking at you Xcode)

            add_library(hs_runtime STATIC src/hs_version.c
                $<TARGET_OBJECTS:hs_exec_common>
                ${RUNTIME_LIBS})
            set_target_properties(hs_runtime PROPERTIES LINKER_LANGUAGE C)
            add_library(hs_compile OBJECT ${hs_compile_SRCS})

            # we want the static lib for testing
            add_library(hs STATIC src/hs_version.c src/hs_valid_platform.c
                $<TARGET_OBJECTS:hs_compile>
                $<TARGET_OBJECTS:hs_exec_common>
                ${RUNTIME_LIBS})

            if (BUILD_SSE2_SIMDE)
                set_target_properties(hs_compile PROPERTIES
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                  COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2"
                )
                set_target_properties(hs PROPERTIES
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                  COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2"
                )
            endif (BUILD_SSE2_SIMDE)

        endif (BUILD_STATIC_LIBS)

        if (BUILD_SHARED_LIBS)
            # build shared libs
            add_library(hs_compile_shared OBJECT ${hs_compile_SRCS})
            set_target_properties(hs_compile_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

            if (BUILD_SSE2_SIMDE)
              message("Building SIMDE SSE2 version..")
              add_library(hs_exec_shared_core2 OBJECT ${hs_exec_SRCS})
              list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_core2>)

              include_directories(${PROJECT_SOURCE_DIR}/simde)
              if (CMAKE_COMPILER_IS_CLANG)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSIMDE_NO_CHECK_IMMEDIATE_CONSTANT")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSIMDE_NO_CHECK_IMMEDIATE_CONSTANT")
              endif()
              set_target_properties(hs_exec_shared_core2 PROPERTIES
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                  COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2"
                  POSITION_INDEPENDENT_CODE TRUE
                  RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                  )
              set_target_properties(hs_compile_shared PROPERTIES
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
                  string(REGEX REPLACE "-msse4.2" "-msse2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
                  COMPILE_FLAGS "-DVS_SIMDE_BACKEND -DVS_SIMDE_NATIVE -march=core2 -msse2"
                  )
            else()
              add_library(hs_exec_shared_core2 OBJECT ${hs_exec_SRCS})
              list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_core2>)
              set_target_properties(hs_exec_shared_core2 PROPERTIES
                  COMPILE_FLAGS "-march=core2 -msse4.2"
                  POSITION_INDEPENDENT_CODE TRUE
                  RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                  )
            endif (BUILD_SSE2_SIMDE)

            add_library(hs_exec_shared_corei7 OBJECT ${hs_exec_SRCS})
            list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_corei7>)
            set_target_properties(hs_exec_shared_corei7 PROPERTIES
                COMPILE_FLAGS "-march=corei7 -msse4.2"
                POSITION_INDEPENDENT_CODE TRUE
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} corei7 ${CMAKE_MODULE_PATH}/keep.syms.in"
                )

            if (BUILD_AVX2)
                add_library(hs_exec_shared_avx2 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_avx2>)
                set_target_properties(hs_exec_shared_avx2 PROPERTIES
                    COMPILE_FLAGS "-march=core-avx2 -mavx2"
                    POSITION_INDEPENDENT_CODE TRUE
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX2)
            if (BUILD_AVX512)
                add_library(hs_exec_shared_avx512 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_avx512>)
                set_target_properties(hs_exec_shared_avx512 PROPERTIES
                    COMPILE_FLAGS "${SKYLAKE_FLAG}"
                    POSITION_INDEPENDENT_CODE TRUE
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512 ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX512)
            if (BUILD_AVX512VBMI)
                add_library(hs_exec_shared_avx512vbmi OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
                list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_avx512vbmi>)
                set_target_properties(hs_exec_shared_avx512vbmi PROPERTIES
                    COMPILE_FLAGS "${ICELAKE_FLAG}"
                    POSITION_INDEPENDENT_CODE TRUE
                    RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512vbmi ${CMAKE_MODULE_PATH}/keep.syms.in"
                    )
            endif (BUILD_AVX512VBMI)
            add_library(hs_exec_common_shared OBJECT
            ${hs_exec_common_SRCS}
            src/dispatcher.c
            )
            set_target_properties(hs_exec_common_shared PROPERTIES
                OUTPUT_NAME hs_exec_common
                POSITION_INDEPENDENT_CODE TRUE)

        endif() # SHARED
    endif (ARCH_IA32 OR ARCH_X86_64)
    if (ARCH_AARCH64)
        set(BUILD_WRAPPER "${PROJECT_SOURCE_DIR}/cmake/build_wrapper.sh")
        if (BUILD_STATIC_LIBS)
            add_library(hs_exec_neon OBJECT ${hs_exec_SRCS} ${hs_exec_neon_SRCS})
            list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_neon>)
            set_target_properties(hs_exec_neon PROPERTIES
                COMPILE_FLAGS "-march=${ARMV8_ARCH}"
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} neon ${CMAKE_MODULE_PATH}/keep.syms.in"
                )

            add_library(hs_exec_sve OBJECT ${hs_exec_SRCS} ${hs_exec_sve_SRCS})
            list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_sve>)
            set_target_properties(hs_exec_sve PROPERTIES
                COMPILE_FLAGS "-march=${SVE_ARCH}"
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} sve ${CMAKE_MODULE_PATH}/keep.syms.in"
                )
            add_library(hs_exec_sve2 OBJECT ${hs_exec_SRCS} ${hs_exec_sve2_SRCS})
            list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_sve2>)
            set_target_properties(hs_exec_sve2 PROPERTIES
                COMPILE_FLAGS "-march=${SVE2_BITPERM_ARCH}"
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} sve2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                )

            add_library(hs_exec_common OBJECT
                ${hs_exec_common_SRCS}
                src/dispatcher.c
                )

            # hs_version.c is added explicitly to avoid some build systems that refuse to
            # create a lib without any src (I'm looking at you Xcode)

            add_library(hs_runtime STATIC src/hs_version.c
                $<TARGET_OBJECTS:hs_exec_common>
                ${RUNTIME_LIBS})
            set_target_properties(hs_runtime PROPERTIES LINKER_LANGUAGE C)
            add_library(hs_compile OBJECT ${hs_compile_SRCS})

            # we want the static lib for testing
            add_library(hs STATIC src/hs_version.c src/hs_valid_platform.c
                $<TARGET_OBJECTS:hs_compile>
                $<TARGET_OBJECTS:hs_exec_common>
                ${RUNTIME_LIBS})
        endif (BUILD_STATIC_LIBS)

        if (BUILD_SHARED_LIBS)
            # build shared libs
            add_library(hs_compile_shared OBJECT ${hs_compile_SRCS})
            set_target_properties(hs_compile_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
	          add_library(hs_exec_shared_neon OBJECT ${hs_exec_SRCS} ${hs_exec_neon_SRCS})
            list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_neon>)
            set_target_properties(hs_exec_shared_neon PROPERTIES
                COMPILE_FLAGS "-march=${ARMV8_ARCH}"
                POSITION_INDEPENDENT_CODE TRUE
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} neon ${CMAKE_MODULE_PATH}/keep.syms.in"
                )

            add_library(hs_exec_shared_sve OBJECT ${hs_exec_SRCS} ${hs_exec_sve_SRCS})
            list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_sve>)
            set_target_properties(hs_exec_shared_sve PROPERTIES
		            COMPILE_FLAGS "-march=${SVE_ARCH}"
                POSITION_INDEPENDENT_CODE TRUE
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} sve ${CMAKE_MODULE_PATH}/keep.syms.in"
                )
            add_library(hs_exec_shared_sve2 OBJECT ${hs_exec_SRCS} ${hs_exec_sve2_SRCS})
            list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_sve2>)
            set_target_properties(hs_exec_shared_sve2 PROPERTIES
                COMPILE_FLAGS "-march=${SVE2_BITPERM_ARCH}"
                POSITION_INDEPENDENT_CODE TRUE
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} sve2 ${CMAKE_MODULE_PATH}/keep.syms.in"
                )
            add_library(hs_exec_common_shared OBJECT
            ${hs_exec_common_SRCS}
            src/dispatcher.c
            )
            set_target_properties(hs_exec_common_shared PROPERTIES
                OUTPUT_NAME hs_exec_common
                POSITION_INDEPENDENT_CODE TRUE)
        endif() # SHARED
    endif (ARCH_AARCH64)
endif (NOT FAT_RUNTIME)

if (BUILD_STATIC_LIBS)
    install(TARGETS hs_runtime DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (BUILD_SHARED_LIBS)
    if (NOT FAT_RUNTIME)
        add_library(hs_runtime_shared SHARED src/hs_version.c
            src/hs_valid_platform.c $<TARGET_OBJECTS:hs_exec_shared>
            hs_runtime.def)
    else()
        add_library(hs_runtime_shared SHARED src/hs_version.c
            src/hs_valid_platform.c
            $<TARGET_OBJECTS:hs_exec_common_shared>
            ${RUNTIME_SHLIBS}
            hs_runtime.def)
    endif()
    set_target_properties(hs_runtime_shared PROPERTIES
        VERSION ${LIB_VERSION}
        SOVERSION ${LIB_SOVERSION}
        OUTPUT_NAME hs_runtime
        MACOSX_RPATH ON
        LINKER_LANGUAGE C)
    install(TARGETS hs_runtime_shared
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (BUILD_STATIC_LIBS)
    add_dependencies(hs ragel_Parser)
endif ()

if (BUILD_STATIC_LIBS)
    install(TARGETS hs DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (BUILD_SHARED_LIBS)
    set(hs_shared_SRCS
        src/hs_version.c
        src/hs_valid_platform.c
        $<TARGET_OBJECTS:hs_compile_shared>)

    if (NOT FAT_RUNTIME)
        set(hs_shared_SRCS
            ${hs_shared_SRCS}
            $<TARGET_OBJECTS:hs_exec_shared>)
    else ()
        set(hs_shared_SRCS
            ${hs_shared_SRCS}
            $<TARGET_OBJECTS:hs_exec_common_shared>
            ${RUNTIME_SHLIBS})
    endif ()

    add_library(hs_shared SHARED ${hs_shared_SRCS} hs.def)

    add_dependencies(hs_shared ragel_Parser)
    set_target_properties(hs_shared PROPERTIES
        OUTPUT_NAME hs
        VERSION ${LIB_VERSION}
        SOVERSION ${LIB_SOVERSION}
        MACOSX_RPATH ON)

    install(TARGETS hs_shared
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

# used by tools and other targets
if (NOT BUILD_STATIC_LIBS)
    # use shared lib without having to change all the targets
    add_library(hs ALIAS hs_shared)
endif ()

add_subdirectory(util)

option(BUILD_UNIT "Build Hyperscan unit tests (default TRUE)" TRUE)
if(BUILD_UNIT)
    add_subdirectory(unit)
endif()

option(BUILD_TOOLS "Build Hyperscan tools (default TRUE)" TRUE)
if(EXISTS ${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt AND BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if (EXISTS ${CMAKE_SOURCE_DIR}/chimera/CMakeLists.txt AND BUILD_CHIMERA)
    add_subdirectory(chimera)
endif()

option(BUILD_EXAMPLES "Build Hyperscan example code (default TRUE)" TRUE)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

option(BUILD_BENCHMARKS "Build benchmarks (default TRUE)" TRUE)
if(BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

option(BUILD_DOC "Build the Hyperscan documentation (default TRUE)" TRUE)
if(BUILD_DOC)
    add_subdirectory(doc/dev-reference)
endif()
