cmake_minimum_required (VERSION 2.8.11)
project (Hyperscan C CXX)

set (HS_MAJOR_VERSION 4)
set (HS_MINOR_VERSION 2)
set (HS_PATCH_VERSION 0)
set (HS_VERSION ${HS_MAJOR_VERSION}.${HS_MINOR_VERSION}.${HS_PATCH_VERSION})

string (TIMESTAMP BUILD_DATE "%Y-%m-%d")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
INCLUDE (CheckFunctionExists)
INCLUDE (CheckIncludeFiles)
INCLUDE (CheckIncludeFileCXX)
INCLUDE (CheckLibraryExists)
INCLUDE (CheckSymbolExists)
include (CMakeDependentOption)
include (${CMAKE_MODULE_PATH}/platform.cmake)
include (${CMAKE_MODULE_PATH}/ragel.cmake)

find_package(PkgConfig QUIET)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Default build type 'Release with debug info'")
    set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
else()
    string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
    message(STATUS "Build type ${CMAKE_BUILD_TYPE}")
endif()

if(CMAKE_BUILD_TYPE MATCHES RELEASE|RELWITHDEBINFO)
    set(RELEASE_BUILD TRUE)
else()
    set(RELEASE_BUILD FALSE)
endif()

set(BINDIR "${PROJECT_BINARY_DIR}/bin")
set(LIBDIR "${PROJECT_BINARY_DIR}/lib")

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


if(CMAKE_GENERATOR STREQUAL Xcode)
    set(XCODE TRUE)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR 1)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})
include_directories(SYSTEM include)

set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED OFF)
set(BOOST_USE_STATIC_RUNTIME OFF)
set(BOOST_MINVERSION 1.57.0)
set(BOOST_NO_BOOST_CMAKE ON)

# first check for Boost installed on the system
find_package(Boost ${BOOST_MINVERSION})
if(NOT Boost_FOUND)
    # we might have boost in tree, so provide a hint and try again
    message(STATUS "trying include dir for boost")
    set(BOOST_INCLUDEDIR "${PROJECT_SOURCE_DIR}/include")
    find_package(Boost ${BOOST_MINVERSION})
    if(NOT Boost_FOUND)
        message(FATAL_ERROR "Boost ${BOOST_MINVERSION} or later not found. Either install system packages if available, extract Boost headers to ${CMAKE_SOURCE_DIR}/include, or set the CMake BOOST_ROOT variable.")
    endif()
endif()


# -- make this work? set(python_ADDITIONAL_VERSIONS 2.7 2.6)
find_package(PythonInterp)
find_program(RAGEL ragel)

if(PYTHONINTERP_FOUND)
    set(PYTHON ${PYTHON_EXECUTABLE})
else()
    message(FATAL_ERROR "No python interpreter found")
endif()

if(${RAGEL} STREQUAL "RAGEL-NOTFOUND")
    message(FATAL_ERROR "Ragel state machine compiler not found")
endif()

option(OPTIMISE "Turns off compiler optimizations (on by default unless debug output enabled or coverage testing)" TRUE)

option(DEBUG_OUTPUT "Enable debug output (warning: very verbose)" FALSE)

if(DEBUG_OUTPUT)
    add_definitions(-DDEBUG)
    set(OPTIMISE FALSE)
endif(DEBUG_OUTPUT)

option(BUILD_SHARED_LIBS "Build shared libs instead of static" OFF)
option(BUILD_STATIC_AND_SHARED "Build shared libs as well as static" OFF)

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
    if (WIN32)
        message(FATAL_ERROR "Windows DLLs currently not supported")
    else()
        message(STATUS "Building shared libraries")
    endif()
endif()

#for config
if (OPTIMISE)
    set(HS_OPTIMIZE ON)
endif()

CMAKE_DEPENDENT_OPTION(DUMP_SUPPORT "Dump code support; normally on, except in release builds" ON "NOT RELEASE_BUILD" OFF)

option(DISABLE_ASSERTS "Disable assert(); enabled in debug builds, disabled in release builds" FALSE)

if (DISABLE_ASSERTS)
    if (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
        add_definitions(-DNDEBUG)
    endif()
endif()

option(WINDOWS_ICC "Use Intel C++ Compiler on Windows, default off, requires ICC to be set in project" OFF)

# TODO: per platform config files?

# TODO: windows generator on cmake always uses msvc, even if we plan to build with icc
if(MSVC OR MSVC_IDE)
    message(STATUS "Building for Windows")
    if (MSVC_VERSION LESS 1700)
        message(FATAL_ERROR "The project requires C++11 features.")
    else()
        if (WINDOWS_ICC)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Qstd=c99 /Qrestrict /QxHost /O3 /wd4267 /Qdiag-disable:remark")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Qstd=c++11 /Qrestrict /QxHost /O2 /wd4267 /wd4800 /Qdiag-disable:remark -DBOOST_DETAIL_NO_CONTAINER_FWD -D_SCL_SECURE_NO_WARNINGS")
        else()
            #TODO: don't hardcode arch
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX /O2 /wd4267")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX /O2 /wd4244 /wd4267 /wd4800 /wd2586 /wd1170 -DBOOST_DETAIL_NO_CONTAINER_FWD -D_SCL_SECURE_NO_WARNINGS")
        endif()
        string(REGEX REPLACE "/RTC1" ""
            CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}" )
        string(REGEX REPLACE "/RTC1" ""
            CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}" )

    endif()

else()

    # compiler version checks TODO: test more compilers
    if (CMAKE_COMPILER_IS_GNUCXX)
        set (GNUCXX_MINVER "4.8.1")
        exec_program(${CMAKE_CXX_COMPILER}
                     ARGS ${CMAKE_CXX_COMPILER_ARG1} --version
                     OUTPUT_VARIABLE _GXX_OUTPUT)
        # is the following too fragile?
        string(REGEX REPLACE ".* ([0-9]\\.[0-9](\\.[0-9])?)( |\n).*" "\\1"
               GNUCXX_VERSION "${_GXX_OUTPUT}")
        message(STATUS "g++ version ${GNUCXX_VERSION}")
        if (GNUCXX_VERSION VERSION_LESS ${GNUCXX_MINVER})
            message(FATAL_ERROR "A minimum of g++ ${GNUCXX_MINVER} is required for C++11 support")
        endif()
        unset(_GXX_OUTPUT)
    endif()

    # set compiler flags - more are tested and added later
    set(EXTRA_C_FLAGS "-std=c99 -Wall -Wextra -Wshadow -Wcast-qual")
    set(EXTRA_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wno-shadow -Wswitch -Wreturn-type -Wcast-qual -Wno-deprecated -Wnon-virtual-dtor")
    if (NOT RELEASE_BUILD)
        # -Werror is most useful during development, don't potentially break
        # release builds
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Werror")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Werror")
    endif()

    if (NOT CMAKE_C_FLAGS MATCHES .*march.*)
        message(STATUS "Building for current host CPU")
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -march=native -mtune=native")
    endif()
    if (NOT CMAKE_CXX_FLAGS MATCHES .*march.*)
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -march=native -mtune=native")
    endif()

    if(CMAKE_COMPILER_IS_GNUCC)
        # spurious warnings?
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-array-bounds -Wno-maybe-uninitialized")
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX)
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fabi-version=0 -Wno-unused-local-typedefs -Wno-maybe-uninitialized")
    endif()

    if(OPTIMISE)
        set(EXTRA_C_FLAGS "-O3 ${EXTRA_C_FLAGS}")
        set(EXTRA_CXX_FLAGS "-O2 ${EXTRA_CXX_FLAGS}")
    else()
        set(EXTRA_C_FLAGS "-O0 ${EXTRA_C_FLAGS}")
        set(EXTRA_CXX_FLAGS "-O0 ${EXTRA_CXX_FLAGS}")
    endif(OPTIMISE)

    if(NOT RELEASE_BUILD)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -fno-omit-frame-pointer")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fno-omit-frame-pointer")
    endif()

endif()

CHECK_INCLUDE_FILES(unistd.h HAVE_UNISTD_H)
CHECK_INCLUDE_FILES(intrin.h HAVE_C_INTRIN_H)
CHECK_INCLUDE_FILE_CXX(intrin.h HAVE_CXX_INTRIN_H)
CHECK_INCLUDE_FILES(tmmintrin.h HAVE_TMMINTRIN_H)
CHECK_INCLUDE_FILES(x86intrin.h HAVE_C_X86INTRIN_H)
CHECK_INCLUDE_FILE_CXX(x86intrin.h HAVE_CXX_X86INTRIN_H)

CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)
CHECK_FUNCTION_EXISTS(_aligned_malloc HAVE__ALIGNED_MALLOC)

# these end up in the config file
CHECK_C_COMPILER_FLAG(-fvisibility=hidden HAS_C_HIDDEN)
CHECK_CXX_COMPILER_FLAG(-fvisibility=hidden HAS_CXX_HIDDEN)

if (RELEASE_BUILD)
    if (HAS_C_HIDDEN)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -fvisibility=hidden")
    endif()
    if (HAS_CXX_HIDDEN)
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fvisibility=hidden")
    endif()
endif()

# ensure we are building for the right target arch
include (${CMAKE_MODULE_PATH}/arch.cmake)

# testing a builtin takes a little more work
CHECK_C_SOURCE_COMPILES("void *aa_test(void *x) { return __builtin_assume_aligned(x, 16);}\nint main(void) { return 0; }" HAVE_CC_BUILTIN_ASSUME_ALIGNED)
CHECK_CXX_SOURCE_COMPILES("void *aa_test(void *x) { return __builtin_assume_aligned(x, 16);}\nint main(void) { return 0; }" HAVE_CXX_BUILTIN_ASSUME_ALIGNED)

if (NOT WIN32)
set(C_FLAGS_TO_CHECK
# Variable length arrays are way bad, most especially at run time
"-Wvla"
# Pointer arith on void pointers is doing it wong.
 "-Wpointer-arith"
# Build our C code with -Wstrict-prototypes -Wmissing-prototypes
 "-Wstrict-prototypes"
 "-Wmissing-prototypes"
)
foreach (FLAG ${C_FLAGS_TO_CHECK})
    # munge the name so it doesn't break things
    string(REPLACE "-" "_" FNAME C_FLAG${FLAG})
    CHECK_C_COMPILER_FLAG("${FLAG}" ${FNAME})
    if (${FNAME})
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} ${FLAG}")
    endif()
endforeach()

set(CXX_FLAGS_TO_CHECK
"-Wvla"
"-Wpointer-arith"
)
foreach (FLAG ${CXX_FLAGS_TO_CHECK})
    string(REPLACE "-" "_" FNAME CXX_FLAG${FLAG})
    CHECK_CXX_COMPILER_FLAG("${FLAG}" ${FNAME})
    if (${FNAME})
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} ${FLAG}")
    endif()
endforeach()

# self-assign should be thrown away, but clang whinges
CHECK_C_COMPILER_FLAG("-Wself-assign" CC_SELF_ASSIGN)
if (CC_SELF_ASSIGN)
    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-self-assign")
endif()
CHECK_CXX_COMPILER_FLAG("-Wself-assign" CXX_SELF_ASSIGN)
if (CXX_SELF_ASSIGN)
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wno-self-assign")
endif()

# clang gets up in our face for going paren crazy with macros
CHECK_C_COMPILER_FLAG("-Wparentheses-equality" CC_PAREN_EQUALITY)
if (CC_PAREN_EQUALITY)
    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-parentheses-equality")
endif()

# clang compains about unused const vars in our Ragel-generated code.
CHECK_CXX_COMPILER_FLAG("-Wunused-const-variable" CXX_UNUSED_CONST_VAR)
if (CXX_UNUSED_CONST_VAR)
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wno-unused-const-variable")
endif()


# note this for later
# g++ doesn't have this flag but clang does
CHECK_CXX_COMPILER_FLAG("-Wweak-vtables" CXX_WEAK_VTABLES)
if (CXX_WEAK_VTABLES)
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wweak-vtables")
endif()

CHECK_CXX_COMPILER_FLAG("-Wmissing-declarations" CXX_MISSING_DECLARATIONS)
if (CXX_MISSING_DECLARATIONS)
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wmissing-declarations")
endif()

# gcc5 complains about this
CHECK_CXX_COMPILER_FLAG("-Wunused-variable" CXX_WUNUSED_VARIABLE)

endif()

if (NOT XCODE)
    include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
else()
    # cmake doesn't think Xcode supports isystem
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -isystem ${Boost_INCLUDE_DIRS}")
endif()


if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(LINUX TRUE)
endif(CMAKE_SYSTEM_NAME MATCHES "Linux")

if(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    set(FREEBSD true)
endif(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")

if(NOT WIN32)
if(CMAKE_C_COMPILER_ID MATCHES "Intel")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -diag-error 10006 -diag-disable 177 -diag-disable 2304 -diag-disable 2305 -diag-disable 2338 -diag-disable 1418 -diag-disable=remark")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -diag-error 10006 -diag-disable 177 -diag-disable 2304 -diag-disable 2305 -diag-disable 2338 -diag-disable 1418 -diag-disable 1170 -diag-disable 3373 -diag-disable=remark")
endif()
endif()

add_subdirectory(util)
add_subdirectory(unit)
add_subdirectory(doc/dev-reference)
if (EXISTS ${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()

# do substitutions
configure_file(${CMAKE_MODULE_PATH}/config.h.in ${PROJECT_BINARY_DIR}/config.h)
configure_file(src/hs_version.h.in ${PROJECT_BINARY_DIR}/hs_version.h)

if (NOT WIN32)
    # expand out library names for pkgconfig static link info
    foreach (LIB ${CMAKE_CXX_IMPLICIT_LINK_LIBRARIES})
        # this is fragile, but protects us from toolchain specific files
        if (NOT EXISTS ${LIB})
            set(PRIVATE_LIBS "${PRIVATE_LIBS} -l${LIB}")
        endif()
    endforeach()

    configure_file(libhs.pc.in libhs.pc @ONLY) # only replace @ quoted vars
    install(FILES ${CMAKE_BINARY_DIR}/libhs.pc
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
endif()

# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")


if(NOT WIN32)
set(RAGEL_C_FLAGS "-Wno-unused")
endif()

set_source_files_properties(
    ${CMAKE_BINARY_DIR}/src/parser/Parser.cpp
    PROPERTIES
        COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(src/parser/Parser.rl)

SET(hs_HEADERS
    src/hs.h
    src/hs_common.h
    src/hs_compile.h
    src/hs_runtime.h
)
install(FILES ${hs_HEADERS} DESTINATION include/hs)

set (hs_exec_SRCS
    ${hs_HEADERS}
    src/hs_version.h
    src/ue2common.h
    src/alloc.c
    src/allocator.h
    src/report.h
    src/runtime.c
    src/fdr/fdr.c
    src/fdr/fdr.h
    src/fdr/fdr_internal.h
    src/fdr/fdr_confirm.h
    src/fdr/fdr_confirm_runtime.h
    src/fdr/fdr_streaming_runtime.h
    src/fdr/flood_runtime.h
    src/fdr/fdr_loadval.h
    src/fdr/teddy.c
    src/fdr/teddy.h
    src/fdr/teddy_internal.h
    src/fdr/teddy_runtime_common.h
    src/hwlm/hwlm.c
    src/hwlm/hwlm.h
    src/hwlm/hwlm_internal.h
    src/hwlm/noodle_engine.c
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
    src/nfa/mcclellan.c
    src/nfa/mcclellan.h
    src/nfa/mcclellan_common_impl.h
    src/nfa/mcclellan_internal.h
    src/nfa/limex_accel.c
    src/nfa/limex_accel.h
    src/nfa/limex_exceptional.h
    src/nfa/limex_native.c
    src/nfa/limex_ring.h
    src/nfa/limex_simd128.c
    src/nfa/limex_simd256.c
    src/nfa/limex_simd384.c
    src/nfa/limex_simd512a.c
    src/nfa/limex_simd512b.c
    src/nfa/limex_simd512c.c
    src/nfa/limex.h
    src/nfa/limex_common_impl.h
    src/nfa/limex_context.h
    src/nfa/limex_internal.h
    src/nfa/limex_runtime.h
    src/nfa/limex_runtime_impl.h
    src/nfa/limex_state_impl.h
    src/nfa/mpv.h
    src/nfa/mpv.c
    src/nfa/mpv_internal.h
    src/nfa/multiaccel_common.h
    src/nfa/multiaccel_doubleshift.h
    src/nfa/multiaccel_doubleshiftgrab.h
    src/nfa/multiaccel_long.h
    src/nfa/multiaccel_longgrab.h
    src/nfa/multiaccel_shift.h
    src/nfa/multiaccel_shiftgrab.h
    src/nfa/multishufti.c
    src/nfa/multishufti_avx2.h
    src/nfa/multishufti_sse.h
    src/nfa/multishufti.h
    src/nfa/multitruffle.c
    src/nfa/multitruffle_avx2.h
    src/nfa/multitruffle_sse.h
    src/nfa/multitruffle.h
    src/nfa/multivermicelli.c
    src/nfa/multivermicelli.h
    src/nfa/multivermicelli_sse.h
    src/nfa/multivermicelli_avx2.h
    src/nfa/nfa_api.h
    src/nfa/nfa_api_dispatch.c
    src/nfa/nfa_internal.h
    src/nfa/nfa_rev_api.h
    src/nfa/repeat.c
    src/nfa/repeat.h
    src/nfa/repeat_internal.h
    src/nfa/shufti_common.h
    src/nfa/shufti.c
    src/nfa/shufti.h
    src/nfa/truffle_common.h
    src/nfa/truffle.c
    src/nfa/truffle.h
    src/nfa/vermicelli.h
    src/nfa/vermicelli_run.h
    src/nfa/vermicelli_sse.h
    src/som/som.h
    src/som/som_operation.h
    src/som/som_runtime.h
    src/som/som_runtime.c
    src/som/som_stream.c
    src/som/som_stream.h
    src/rose/block.c
    src/rose/catchup.h
    src/rose/catchup.c
    src/rose/eod.c
    src/rose/infix.h
    src/rose/init.h
    src/rose/init.c
    src/rose/stream.c
    src/rose/match.h
    src/rose/match.c
    src/rose/miracle.h
    src/rose/program_runtime.h
    src/rose/runtime.h
    src/rose/rose.h
    src/rose/rose_internal.h
    src/rose/rose_program.h
    src/rose/rose_types.h
    src/rose/rose_common.h
    src/util/bitutils.h
    src/util/exhaust.h
    src/util/fatbit.h
    src/util/fatbit.c
    src/util/join.h
    src/util/masked_move.h
    src/util/multibit.h
    src/util/multibit_internal.h
    src/util/multibit.c
    src/util/pack_bits.h
    src/util/popcount.h
    src/util/pqueue.h
    src/util/scatter.h
    src/util/scatter_runtime.h
    src/util/shuffle.h
    src/util/shuffle_ssse3.h
    src/util/simd_utils.h
    src/util/simd_utils_ssse3.h
    src/util/simd_utils_ssse3.c
    src/util/state_compress.h
    src/util/state_compress.c
    src/util/unaligned.h
    src/util/uniform_ops.h
    src/scratch.h
    src/scratch.c
    src/crc32.c
    src/crc32.h
    src/database.c
    src/database.h
)

if (HAVE_AVX2)
    set (hs_exec_SRCS
        ${hs_exec_SRCS}
        src/fdr/teddy_avx2.c
        src/util/masked_move.c
        )
endif ()


SET (hs_SRCS
    ${hs_HEADERS}
    src/crc32.h
    src/database.h
    src/grey.cpp
    src/grey.h
    src/hs.cpp
    src/hs_internal.h
    src/hs_version.c
    src/hs_version.h
    src/scratch.h
    src/state.h
    src/ue2common.h
    src/compiler/asserts.cpp
    src/compiler/asserts.h
    src/compiler/compiler.cpp
    src/compiler/compiler.h
    src/compiler/error.cpp
    src/compiler/error.h
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
    src/fdr/fdr_streaming_compile.cpp
    src/fdr/fdr_streaming_internal.h
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
    src/nfa/accelcompile.cpp
    src/nfa/accelcompile.h
    src/nfa/callback.h
    src/nfa/castlecompile.cpp
    src/nfa/castlecompile.h
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
    src/nfa/mcclellancompile_accel.cpp
    src/nfa/mcclellancompile_accel.h
    src/nfa/mcclellancompile_util.cpp
    src/nfa/mcclellancompile_util.h
    src/nfa/limex_compile.cpp
    src/nfa/limex_compile.h
    src/nfa/limex_accel.h
    src/nfa/limex_internal.h
    src/nfa/mpv_internal.h
    src/nfa/mpvcompile.cpp
    src/nfa/mpvcompile.h
    src/nfa/multiaccel_compilehelper.cpp
    src/nfa/multiaccel_compilehelper.h
    src/nfa/nfa_api.h
    src/nfa/nfa_api_queue.h
    src/nfa/nfa_api_util.h
    src/nfa/nfa_build_util.cpp
    src/nfa/nfa_build_util.h
    src/nfa/nfa_internal.h
    src/nfa/nfa_kind.h
    src/nfa/rdfa.h
    src/nfa/rdfa_merge.cpp
    src/nfa/rdfa_merge.h
    src/nfa/repeat_internal.h
    src/nfa/repeatcompile.cpp
    src/nfa/repeatcompile.h
    src/nfa/shufticompile.cpp
    src/nfa/shufticompile.h
    src/nfa/trufflecompile.cpp
    src/nfa/trufflecompile.h
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
    src/nfagraph/ng_graph.h
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
    src/nfagraph/ng_rose.cpp
    src/nfagraph/ng_rose.h
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
    src/nfagraph/ng_undirected.h
    src/nfagraph/ng_utf8.cpp
    src/nfagraph/ng_utf8.h
    src/nfagraph/ng_util.cpp
    src/nfagraph/ng_util.h
    src/nfagraph/ng_vacuous.cpp
    src/nfagraph/ng_vacuous.h
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
    src/rose/rose_build_impl.h
    src/rose/rose_build_infix.cpp
    src/rose/rose_build_infix.h
    src/rose/rose_build_lookaround.cpp
    src/rose/rose_build_lookaround.h
    src/rose/rose_build_matchers.cpp
    src/rose/rose_build_matchers.h
    src/rose/rose_build_merge.cpp
    src/rose/rose_build_merge.h
    src/rose/rose_build_misc.cpp
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
    src/util/compare.h
    src/util/compile_context.cpp
    src/util/compile_context.h
    src/util/compile_error.cpp
    src/util/compile_error.h
    src/util/container.h
    src/util/cpuid_flags.c
    src/util/cpuid_flags.h
    src/util/depth.cpp
    src/util/depth.h
    src/util/determinise.h
    src/util/dump_mask.cpp
    src/util/dump_mask.h
    src/util/graph.h
    src/util/multibit_build.cpp
    src/util/multibit_build.h
    src/util/order_check.h
    src/util/partial_store.h
    src/util/partitioned_set.h
    src/util/popcount.h
    src/util/queue_index_factory.h
    src/util/report.h
    src/util/report_manager.cpp
    src/util/report_manager.h
    src/util/simd_utils.h
    src/util/simd_utils_ssse3.h
    src/util/target_info.cpp
    src/util/target_info.h
    src/util/ue2_containers.h
    src/util/ue2string.cpp
    src/util/ue2string.h
    src/util/unaligned.h
    src/util/unicode_def.h
    src/util/unicode_set.h
    src/util/uniform_ops.h
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
    src/nfa/mpv_dump.cpp
    src/nfa/nfa_dump_api.h
    src/nfa/nfa_dump_dispatch.cpp
    src/nfa/nfa_dump_internal.cpp
    src/nfa/nfa_dump_internal.h
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
    src/rose/rose_dump.cpp
    src/rose/rose_dump.h
    src/util/dump_charclass.cpp
    src/util/dump_charclass.h
)

if (DUMP_SUPPORT)
    set(hs_SRCS ${hs_SRCS} ${hs_dump_SRCS})
endif()

# we group things by sublibraries, specifying shared and static and then
# choose which ones to build

set (LIB_VERSION ${HS_VERSION})
set (LIB_SOVERSION ${HS_MAJOR_VERSION}.${HS_MINOR_VERSION})

add_library(hs_exec OBJECT ${hs_exec_SRCS})

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
add_library(hs_exec_shared OBJECT ${hs_exec_SRCS})
set_target_properties(hs_exec_shared PROPERTIES
    POSITION_INDEPENDENT_CODE TRUE)
endif()

# hs_version.c is added explicitly to avoid some build systems that refuse to
# create a lib without any src (I'm looking at you Xcode)

add_library(hs_runtime STATIC src/hs_version.c $<TARGET_OBJECTS:hs_exec>)

set_target_properties(hs_runtime PROPERTIES
    LINKER_LANGUAGE C)
if (NOT BUILD_SHARED_LIBS)
    install(TARGETS hs_runtime DESTINATION lib)
endif()

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
    add_library(hs_runtime_shared SHARED src/hs_version.c $<TARGET_OBJECTS:hs_exec_shared>)
    set_target_properties(hs_runtime_shared PROPERTIES
        VERSION ${LIB_VERSION}
        SOVERSION ${LIB_SOVERSION}
        OUTPUT_NAME hs_runtime
        MACOSX_RPATH ON
        LINKER_LANGUAGE C)
    install(TARGETS hs_runtime_shared
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
endif()

# we want the static lib for testing
add_library(hs STATIC ${hs_SRCS} $<TARGET_OBJECTS:hs_exec>)

add_dependencies(hs ragel_Parser)

if (NOT BUILD_SHARED_LIBS)
install(TARGETS hs DESTINATION lib)
endif()

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
    add_library(hs_shared SHARED ${hs_SRCS} $<TARGET_OBJECTS:hs_exec_shared>)
    add_dependencies(hs_shared ragel_Parser)
    set_target_properties(hs_shared PROPERTIES
        OUTPUT_NAME hs
        VERSION ${LIB_VERSION}
        SOVERSION ${LIB_SOVERSION}
        MACOSX_RPATH ON)
install(TARGETS hs_shared
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
endif()

if(NOT WIN32)
    add_subdirectory(examples)
endif()
