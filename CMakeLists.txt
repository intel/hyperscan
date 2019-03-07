cmake_minimum_required (VERSION 2.8.11)
project (hyperscan C CXX)

set (HS_MAJOR_VERSION 5)
set (HS_MINOR_VERSION 1)
set (HS_PATCH_VERSION 0)
set (HS_VERSION ${HS_MAJOR_VERSION}.${HS_MINOR_VERSION}.${HS_PATCH_VERSION})

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
include (${CMAKE_MODULE_PATH}/platform.cmake)
include (${CMAKE_MODULE_PATH}/ragel.cmake)

find_package(PkgConfig QUIET)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Default build type 'Release with debug info'")
    set(CMAKE_BUILD_TYPE RELWITHDEBINFO CACHE STRING "" FORCE )
else()
    string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
    message(STATUS "Build type ${CMAKE_BUILD_TYPE}")
endif()

if(CMAKE_BUILD_TYPE MATCHES RELEASE|RELWITHDEBINFO|MINSIZEREL)
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


if(CMAKE_GENERATOR STREQUAL Xcode)
    set(XCODE TRUE)
endif()

# older versions of cmake don't know things support isystem
if (XCODE OR CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem")
endif ()

set(CMAKE_INCLUDE_CURRENT_DIR 1)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})
include_directories(SYSTEM include)

include (${CMAKE_MODULE_PATH}/boost.cmake)

# -- make this work? set(python_ADDITIONAL_VERSIONS 2.7 2.6)
find_package(PythonInterp)
find_program(RAGEL ragel)

if(PYTHONINTERP_FOUND)
    set(PYTHON ${PYTHON_EXECUTABLE})
else()
    message(FATAL_ERROR "No python interpreter found")
endif()

# allow for reproducible builds - python for portability
if (DEFINED ENV{SOURCE_DATE_EPOCH})
      execute_process(
          COMMAND "${PYTHON}" "${CMAKE_MODULE_PATH}/formatdate.py" "$ENV{SOURCE_DATE_EPOCH}"
          OUTPUT_VARIABLE BUILD_DATE
          OUTPUT_STRIP_TRAILING_WHITESPACE)
else ()
    string (TIMESTAMP BUILD_DATE "%Y-%m-%d")
endif ()
message(STATUS "Build date: ${BUILD_DATE}")


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

if (NOT BUILD_SHARED_LIBS)
    # build static libs
    set(BUILD_STATIC_LIBS ON)
    mark_as_advanced(BUILD_STATIC_LIBS)
endif ()

#for config
if (OPTIMISE)
    set(HS_OPTIMIZE ON)
endif()

CMAKE_DEPENDENT_OPTION(DUMP_SUPPORT "Dump code support; normally on, except in release builds" ON "NOT RELEASE_BUILD" OFF)

CMAKE_DEPENDENT_OPTION(DISABLE_ASSERTS "Disable assert(); Asserts are enabled in debug builds, disabled in release builds" OFF "NOT RELEASE_BUILD" ON)

option(BUILD_AVX512 "Experimental: support avx512 in the fat runtime"
    OFF)

option(WINDOWS_ICC "Use Intel C++ Compiler on Windows, default off, requires ICC to be set in project" OFF)

# TODO: per platform config files?

# TODO: windows generator on cmake always uses msvc, even if we plan to build with icc
if(MSVC OR MSVC_IDE)
    message(STATUS "Building for Windows")

    if (MSVC_VERSION LESS 1700)
        message(FATAL_ERROR "The project requires C++11 features.")
    else()
        if (WINDOWS_ICC)
            set(ARCH_C_FLAGS "/QxHost")
            set(ARCH_CXX_FLAGS "/QxHost")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /O3 /Qstd=c99 /Qrestrict /wd4267 /Qdiag-disable:remark")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /Qstd=c++11 /Qrestrict /QxHost /wd4267 /wd4800 /Qdiag-disable:remark -DBOOST_DETAIL_NO_CONTAINER_FWD -D_SCL_SECURE_NO_WARNINGS")
        else()
            # todo: change these as required
            set(ARCH_C_FLAGS "/arch:AVX2")
            set(ARCH_CXX_FLAGS "/arch:AVX2")
            set(MSVC_WARNS "/wd4101 /wd4146 /wd4172 /wd4200 /wd4244 /wd4267 /wd4307 /wd4334 /wd4805 /wd4996 -D_CRT_SECURE_NO_WARNINGS")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /O2 ${MSVC_WARNS}")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 ${MSVC_WARNS} /wd4800 -DBOOST_DETAIL_NO_CONTAINER_FWD")
        endif()
        string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        string(REPLACE "/RTC1" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

        if (DISABLE_ASSERTS)
            set(CMAKE_C_FLAGS_DEBUG "/DNDEBUG ${CMAKE_C_FLAGS_DEBUG}")
            set(CMAKE_CXX_FLAGS_DEBUG "/DNDEBUG ${CMAKE_CXX_FLAGS_DEBUG}")
        endif ()

        # flags only used to build hs libs
        set(HS_C_FLAGS "/Gv")
        set(HS_CXX_FLAGS "/Gv")
    endif()

else()

    # remove CMake's idea of optimisation
    foreach (CONFIG ${CMAKE_BUILD_TYPE} ${CMAKE_CONFIGURATION_TYPES})
        string(REGEX REPLACE "-O[^ ]*" "" CMAKE_C_FLAGS_${CONFIG} "${CMAKE_C_FLAGS_${CONFIG}}")
        string(REGEX REPLACE "-O[^ ]*" "" CMAKE_CXX_FLAGS_${CONFIG} "${CMAKE_CXX_FLAGS_${CONFIG}}")
    endforeach ()

    if (CMAKE_COMPILER_IS_GNUCC)
        message(STATUS "gcc version ${CMAKE_C_COMPILER_VERSION}")
        # If gcc doesn't recognise the host cpu, then mtune=native becomes
        # generic, which isn't very good in some cases. march=native looks at
        # cpuid info and then chooses the best microarch it can (and replaces
        # the flag), so use that for tune.

        # arg1 might exist if using ccache
        string (STRIP "${CMAKE_C_COMPILER_ARG1}" CC_ARG1)
        set (EXEC_ARGS ${CC_ARG1} -c -Q --help=target -march=native -mtune=native)
        execute_process(COMMAND ${CMAKE_C_COMPILER} ${EXEC_ARGS}
            OUTPUT_VARIABLE _GCC_OUTPUT)
        string(REGEX REPLACE ".*march=[ \t]*([^ \n]*)[ \n].*" "\\1"
            GNUCC_ARCH "${_GCC_OUTPUT}")

        # test the parsed flag
        set (EXEC_ARGS ${CC_ARG1} -E - -mtune=${GNUCC_ARCH})
        execute_process(COMMAND ${CMAKE_C_COMPILER} ${EXEC_ARGS}
            OUTPUT_QUIET ERROR_QUIET
            INPUT_FILE /dev/null
            RESULT_VARIABLE GNUCC_TUNE_TEST)
        if (NOT GNUCC_TUNE_TEST EQUAL 0)
            message(SEND_ERROR "Something went wrong determining gcc tune: -mtune=${GNUCC_ARCH} not valid")
        endif()
        set(TUNE_FLAG ${GNUCC_ARCH})
    else ()
        set(TUNE_FLAG native)
    endif()

    # compiler version checks TODO: test more compilers
    if (CMAKE_COMPILER_IS_GNUCXX)
        set(GNUCXX_MINVER "4.8.1")
        message(STATUS "g++ version ${CMAKE_CXX_COMPILER_VERSION}")
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS GNUCXX_MINVER)
            message(FATAL_ERROR "A minimum of g++ ${GNUCXX_MINVER} is required for C++11 support")
        endif()
    endif()

    if(OPTIMISE)
        if (NOT CMAKE_BUILD_TYPE MATCHES MINSIZEREL)
            set(OPT_C_FLAG "-O3")
            set(OPT_CXX_FLAG "-O2")
        else ()
            set(OPT_C_FLAG "-Os")
            set(OPT_CXX_FLAG "-Os")
        endif ()
    else()
        set(OPT_C_FLAG "-O0")
        set(OPT_CXX_FLAG "-O0")
    endif(OPTIMISE)

    # set compiler flags - more are tested and added later
    set(EXTRA_C_FLAGS "${OPT_C_FLAG} -std=c99 -Wall -Wextra -Wshadow -Wcast-qual -fno-strict-aliasing")
    set(EXTRA_CXX_FLAGS "${OPT_CXX_FLAG} -std=c++11 -Wall -Wextra -Wshadow -Wswitch -Wreturn-type -Wcast-qual -Wno-deprecated -Wnon-virtual-dtor -fno-strict-aliasing")

    if (NOT RELEASE_BUILD)
        # -Werror is most useful during development, don't potentially break
        # release builds
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Werror")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Werror")
    endif()

    if (DISABLE_ASSERTS)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -DNDEBUG")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -DNDEBUG")
    endif()

    if (NOT CMAKE_C_FLAGS MATCHES .*march.* AND NOT CMAKE_C_FLAGS MATCHES .*mtune.*)
        set(ARCH_C_FLAGS "-march=native -mtune=${TUNE_FLAG}")
    endif()

    if (NOT CMAKE_CXX_FLAGS MATCHES .*march.* AND NOT CMAKE_CXX_FLAGS MATCHES .*mtune.*)
        set(ARCH_CXX_FLAGS "-march=native -mtune=${TUNE_FLAG}")
    endif()

    if(CMAKE_COMPILER_IS_GNUCC)
        # spurious warnings?
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-array-bounds -Wno-maybe-uninitialized")
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX)
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wno-maybe-uninitialized")
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)
            set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fabi-version=0")
        endif ()
        # don't complain about abi
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-abi")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wno-abi")
    endif()

    if (NOT(ARCH_IA32 AND RELEASE_BUILD))
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -fno-omit-frame-pointer")
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fno-omit-frame-pointer")
    endif()


    if (CMAKE_C_COMPILER_ID MATCHES "Intel")
        set(SKYLAKE_FLAG "-xCORE-AVX512")
    else ()
        set(SKYLAKE_FLAG "-march=skylake-avx512")
    endif ()
endif()

CHECK_INCLUDE_FILES(unistd.h HAVE_UNISTD_H)
CHECK_INCLUDE_FILES(intrin.h HAVE_C_INTRIN_H)
CHECK_INCLUDE_FILE_CXX(intrin.h HAVE_CXX_INTRIN_H)
CHECK_INCLUDE_FILES(x86intrin.h HAVE_C_X86INTRIN_H)
CHECK_INCLUDE_FILE_CXX(x86intrin.h HAVE_CXX_X86INTRIN_H)

CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)
CHECK_FUNCTION_EXISTS(_aligned_malloc HAVE__ALIGNED_MALLOC)

# these end up in the config file
CHECK_C_COMPILER_FLAG(-fvisibility=hidden HAS_C_HIDDEN)
CHECK_CXX_COMPILER_FLAG(-fvisibility=hidden HAS_CXX_HIDDEN)

# are we using libc++
CHECK_CXX_SYMBOL_EXISTS(_LIBCPP_VERSION ciso646 HAVE_LIBCPP)

if (RELEASE_BUILD)
    if (HAS_C_HIDDEN)
        set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -fvisibility=hidden")
    endif()
    if (HAS_CXX_HIDDEN)
        set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -fvisibility=hidden")
    endif()
endif()

if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    # This is a Linux-only feature for now - requires platform support
    # elsewhere
    message(STATUS "generator is ${CMAKE_GENERATOR}")
    if (CMAKE_C_COMPILER_ID MATCHES "Clang" AND
        CMAKE_C_COMPILER_VERSION VERSION_LESS "3.9")
        message (STATUS "Clang v3.9 or higher required for fat runtime, cannot build fat runtime")
        set (FAT_RUNTIME_REQUISITES FALSE)
    elseif (NOT (CMAKE_GENERATOR MATCHES "Unix Makefiles" OR
            (CMAKE_VERSION VERSION_GREATER "3.0" AND CMAKE_GENERATOR MATCHES "Ninja")))
        message (STATUS "Building the fat runtime requires the Unix Makefiles generator, or Ninja with CMake v3.0 or higher")
        set (FAT_RUNTIME_REQUISITES FALSE)
    else()
        include (${CMAKE_MODULE_PATH}/attrib.cmake)
        if (NOT HAS_C_ATTR_IFUNC)
            message(STATUS "Compiler does not support ifunc attribute, cannot build fat runtime")
            set (FAT_RUNTIME_REQUISITES FALSE)
        else ()
            set (FAT_RUNTIME_REQUISITES TRUE)
        endif()
    endif()
    CMAKE_DEPENDENT_OPTION(FAT_RUNTIME "Build a library that supports multiple microarchitecures" ${RELEASE_BUILD} "FAT_RUNTIME_REQUISITES" OFF)
endif ()

include (${CMAKE_MODULE_PATH}/arch.cmake)

# testing a builtin takes a little more work
CHECK_C_SOURCE_COMPILES("void *aa_test(void *x) { return __builtin_assume_aligned(x, 16);}\nint main(void) { return 0; }" HAVE_CC_BUILTIN_ASSUME_ALIGNED)
CHECK_CXX_SOURCE_COMPILES("void *aa_test(void *x) { return __builtin_assume_aligned(x, 16);}\nint main(void) { return 0; }" HAVE_CXX_BUILTIN_ASSUME_ALIGNED)
CHECK_C_SOURCE_COMPILES("int main(void) { __builtin_constant_p(0); }" HAVE__BUILTIN_CONSTANT_P)

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

# gcc 6 complains about type attributes that get ignored, like alignment
CHECK_CXX_COMPILER_FLAG("-Wignored-attributes" CXX_IGNORED_ATTR)
if (CXX_IGNORED_ATTR)
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wno-ignored-attributes")
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

CHECK_CXX_COMPILER_FLAG("-Wunused-local-typedefs" CXX_UNUSED_LOCAL_TYPEDEFS)

# gcc5 complains about this
CHECK_CXX_COMPILER_FLAG("-Wunused-variable" CXX_WUNUSED_VARIABLE)

endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(LINUX TRUE)
endif(CMAKE_SYSTEM_NAME MATCHES "Linux")

if(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    set(FREEBSD true)
endif(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")

if(NOT WIN32)
if(CMAKE_C_COMPILER_ID MATCHES "Intel")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -diag-error 10006 -diag-disable 68 -diag-disable 177 -diag-disable 186 -diag-disable 2304 -diag-disable 2305 -diag-disable 2338 -diag-disable 1418 -diag-disable 279 -diag-disable=remark")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -diag-error 10006 -diag-disable 68 -diag-disable 177 -diag-disable 186 -diag-disable 2304 -diag-disable 2305 -diag-disable 2338 -diag-disable 1418 -diag-disable 1170 -diag-disable 3373 -diag-disable 279 -diag-disable=remark")
endif()
endif()

if (NOT FAT_RUNTIME)
    message(STATUS "Building for current host CPU: ${ARCH_C_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ARCH_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARCH_CXX_FLAGS}")
else()
    message(STATUS "Building runtime for multiple microarchitectures")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

add_subdirectory(util)
add_subdirectory(doc/dev-reference)

if (NOT WIN32)
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

add_subdirectory(unit)
if (EXISTS ${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()
if (EXISTS ${CMAKE_SOURCE_DIR}/chimera/CMakeLists.txt AND BUILD_CHIMERA)
    add_subdirectory(chimera)
endif()
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
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
endif()

# only set these after all tests are done
if (NOT FAT_RUNTIME)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS} ${HS_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS} ${HS_CXX_FLAGS}")
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")
endif()

if (WIN32)
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

add_subdirectory(unit)
if (EXISTS ${CMAKE_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()
if (EXISTS ${CMAKE_SOURCE_DIR}/chimera/CMakeLists.txt AND BUILD_CHIMERA)
    add_subdirectory(chimera)
endif()
endif()

if(NOT WIN32)
set(RAGEL_C_FLAGS "-Wno-unused")
endif()

set_source_files_properties(
    ${CMAKE_BINARY_DIR}/src/parser/Parser.cpp
    PROPERTIES
        COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(src/parser/Parser.rl)

set_source_files_properties(
    ${CMAKE_BINARY_DIR}/src/parser/control_verbs.cpp
    PROPERTIES
        COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(src/parser/control_verbs.rl)

SET(hs_HEADERS
    src/hs.h
    src/hs_common.h
    src/hs_compile.h
    src/hs_runtime.h
)
install(FILES ${hs_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hs")

set (hs_exec_common_SRCS
    src/alloc.c
    src/scratch.c
    src/util/cpuid_flags.c
    src/util/cpuid_flags.h
    src/util/multibit.c
    )

set (hs_exec_SRCS
    ${hs_HEADERS}
    src/hs_version.h
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
    src/nfa/shufti.c
    src/nfa/shufti.h
    src/nfa/tamarama.c
    src/nfa/tamarama.h
    src/nfa/tamarama_internal.h
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
    src/util/masked_move.h
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
    src/util/simd_utils.c
    src/util/state_compress.h
    src/util/state_compress.c
    src/util/unaligned.h
    src/util/uniform_ops.h
    src/database.c
    src/database.h
)

set (hs_exec_avx2_SRCS
    src/fdr/teddy_avx2.c
    src/util/masked_move.c
)


SET (hs_compile_SRCS
    ${hs_HEADERS}
    src/crc32.h
    src/database.h
    src/grey.cpp
    src/grey.h
    src/hs.cpp
    src/hs_internal.h
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

    if (HAVE_AVX2)
        set(hs_exec_SRCS ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
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

    if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
        add_library(hs_exec_shared OBJECT ${hs_exec_SRCS})
        set_target_properties(hs_exec_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
        add_library(hs_compile_shared OBJECT ${hs_compile_SRCS})
        set_target_properties(hs_compile_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
    endif()

else (FAT_RUNTIME)

    set(BUILD_WRAPPER "${PROJECT_SOURCE_DIR}/cmake/build_wrapper.sh")
    if (NOT BUILD_AVX512)
        set (DISPATCHER_DEFINE "-DDISABLE_AVX512_DISPATCH")
    endif (NOT BUILD_AVX512)
    set_source_files_properties(src/dispatcher.c PROPERTIES
        COMPILE_FLAGS "-Wno-unused-parameter -Wno-unused-function ${DISPATCHER_DEFINE}")

    if (BUILD_STATIC_LIBS)
       add_library(hs_exec_core2 OBJECT ${hs_exec_SRCS})
       list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_core2>)
       set_target_properties(hs_exec_core2 PROPERTIES
           COMPILE_FLAGS "-march=core2"
           RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
           )

       add_library(hs_exec_corei7 OBJECT ${hs_exec_SRCS})
       list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_corei7>)
       set_target_properties(hs_exec_corei7 PROPERTIES
           COMPILE_FLAGS "-march=corei7"
           RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} corei7 ${CMAKE_MODULE_PATH}/keep.syms.in"
           )

       add_library(hs_exec_avx2 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
       list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_avx2>)
       set_target_properties(hs_exec_avx2 PROPERTIES
           COMPILE_FLAGS "-march=core-avx2"
           RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx2 ${CMAKE_MODULE_PATH}/keep.syms.in"
           )
       if (BUILD_AVX512)
           add_library(hs_exec_avx512 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
           list(APPEND RUNTIME_LIBS $<TARGET_OBJECTS:hs_exec_avx512>)
           set_target_properties(hs_exec_avx512 PROPERTIES
               COMPILE_FLAGS "${SKYLAKE_FLAG}"
               RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512 ${CMAKE_MODULE_PATH}/keep.syms.in"
               )
       endif (BUILD_AVX512)

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

    if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
        # build shared libs
        add_library(hs_compile_shared OBJECT ${hs_compile_SRCS})
        set_target_properties(hs_compile_shared PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
        add_library(hs_exec_shared_core2 OBJECT ${hs_exec_SRCS})
        list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_core2>)
        set_target_properties(hs_exec_shared_core2 PROPERTIES
            COMPILE_FLAGS "-march=core2"
            POSITION_INDEPENDENT_CODE TRUE
            RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} core2 ${CMAKE_MODULE_PATH}/keep.syms.in"
            )
        add_library(hs_exec_shared_corei7 OBJECT ${hs_exec_SRCS})
        list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_corei7>)
        set_target_properties(hs_exec_shared_corei7 PROPERTIES
            COMPILE_FLAGS "-march=corei7"
            POSITION_INDEPENDENT_CODE TRUE
            RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} corei7 ${CMAKE_MODULE_PATH}/keep.syms.in"
            )
        add_library(hs_exec_shared_avx2 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
        list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_avx2>)
        set_target_properties(hs_exec_shared_avx2 PROPERTIES
            COMPILE_FLAGS "-march=core-avx2"
            POSITION_INDEPENDENT_CODE TRUE
            RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx2 ${CMAKE_MODULE_PATH}/keep.syms.in"
            )

        if (BUILD_AVX512)
            add_library(hs_exec_shared_avx512 OBJECT ${hs_exec_SRCS} ${hs_exec_avx2_SRCS})
            list(APPEND RUNTIME_SHLIBS $<TARGET_OBJECTS:hs_exec_shared_avx512>)
            set_target_properties(hs_exec_shared_avx512 PROPERTIES
                COMPILE_FLAGS "${SKYLAKE_FLAG}"
                POSITION_INDEPENDENT_CODE TRUE
                RULE_LAUNCH_COMPILE "${BUILD_WRAPPER} avx512 ${CMAKE_MODULE_PATH}/keep.syms.in"
                )
        endif (BUILD_AVX512)
        add_library(hs_exec_common_shared OBJECT
        ${hs_exec_common_SRCS}
        src/dispatcher.c
        )
        set_target_properties(hs_exec_common_shared PROPERTIES
            OUTPUT_NAME hs_exec_common
            POSITION_INDEPENDENT_CODE TRUE)
    endif() # SHARED


endif (NOT FAT_RUNTIME)

if (NOT BUILD_SHARED_LIBS)
    install(TARGETS hs_runtime DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
    if (NOT FAT_RUNTIME)
        add_library(hs_runtime_shared SHARED src/hs_version.c
            src/hs_valid_platform.c $<TARGET_OBJECTS:hs_exec_shared>)
    else()
        add_library(hs_runtime_shared SHARED src/hs_version.c
            src/hs_valid_platform.c
            $<TARGET_OBJECTS:hs_exec_common_shared>
            ${RUNTIME_SHLIBS})
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

if (NOT BUILD_SHARED_LIBS)
    install(TARGETS hs DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
    set(hs_shared_SRCS
        src/hs_version.c
        src/hs_valid_platform.c
        $<TARGET_OBJECTS:hs_compile_shared>)

    if (XCODE)
        # force this lib to use C++ linkage
        add_custom_command(OUTPUT empty.cxx
            COMMAND ${CMAKE_COMMAND} -E touch empty.cxx)
        set (hs_shared_SRCS ${hs_shared_SRCS} empty.cxx)
    endif (XCODE)

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

    add_library(hs_shared SHARED ${hs_shared_SRCS})

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


if(NOT WIN32)
    add_subdirectory(examples)
endif()
