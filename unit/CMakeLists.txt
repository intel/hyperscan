set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

if(CMAKE_C_FLAGS MATCHES "/Gv" )
    string(REPLACE "/Gv" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
endif()
if(CMAKE_CXX_FLAGS MATCHES "/Gv" )
    string(REPLACE "/Gv" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

set(gtest_SOURCES gtest/gtest-all.cc gtest/gtest.h)
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# remove some warnings
# cmake's scope means these only apply here

if (CXX_MISSING_DECLARATIONS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-declarations")
endif()

if(CXX_WEAK_VTABLES)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-weak-vtables")
endif()

if(CXX_WUNUSED_VARIABLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
endif()

if (CXX_UNUSED_LOCAL_TYPEDEFS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
endif()

if(CMAKE_COMPILER_IS_GNUCC)
    # spurious warnings?
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-array-bounds")
endif()

add_definitions(-DGTEST_HAS_PTHREAD=0 -DSRCDIR=${PROJECT_SOURCE_DIR})

if (WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4309 /wd4018")
endif()

set(unit_hyperscan_SOURCES
    ${gtest_SOURCES}
    hyperscan/allocators.cpp
    hyperscan/arg_checks.cpp
    hyperscan/bad_patterns.cpp
    hyperscan/bad_patterns.txt
    hyperscan/behaviour.cpp
    hyperscan/expr_info.cpp
    hyperscan/extparam.cpp
    hyperscan/identical.cpp
    hyperscan/literals.cpp
    hyperscan/logical_combination.cpp
    hyperscan/main.cpp
    hyperscan/multi.cpp
    hyperscan/order.cpp
    hyperscan/scratch_op.cpp
    hyperscan/scratch_in_use.cpp
    hyperscan/serialize.cpp
    hyperscan/single.cpp
    hyperscan/som.cpp
    hyperscan/stream_op.cpp
    hyperscan/test_util.cpp
    hyperscan/test_util.h
    )
add_executable(unit-hyperscan ${unit_hyperscan_SOURCES})
if (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS)
target_link_libraries(unit-hyperscan hs_shared expressionutil)
else()
target_link_libraries(unit-hyperscan hs expressionutil)
endif()


if (NOT (RELEASE_BUILD OR FAT_RUNTIME))
set(unit_internal_SOURCES
    ${gtest_SOURCES}
    internal/bitfield.cpp
    internal/bitutils.cpp
    internal/charreach.cpp
    internal/compare.cpp
    internal/database.cpp
    internal/depth.cpp
    internal/fdr.cpp
    internal/fdr_flood.cpp
    internal/fdr_loadval.cpp
    internal/flat_set.cpp
    internal/flat_map.cpp
    internal/graph.cpp
    internal/graph_undirected.cpp
    internal/insertion_ordered.cpp
    internal/lbr.cpp
    internal/limex_nfa.cpp
    internal/masked_move.cpp
    internal/multi_bit.cpp
    internal/multi_bit_compress.cpp
    internal/nfagraph_common.h
    internal/nfagraph_comp.cpp
    internal/nfagraph_equivalence.cpp
    internal/nfagraph_find_matches.cpp
    internal/nfagraph_literal_analysis.cpp
    internal/nfagraph_redundancy.cpp
    internal/nfagraph_repeat.cpp
    internal/nfagraph_util.cpp
    internal/nfagraph_width.cpp
    internal/noodle.cpp
    internal/pack_bits.cpp
    internal/parser.cpp
    internal/partial.cpp
    internal/pqueue.cpp
    internal/repeat.cpp
    internal/rose_build_merge.cpp
    internal/rose_mask.cpp
    internal/rose_mask_32.cpp
    internal/rvermicelli.cpp
    internal/simd_utils.cpp
    internal/shuffle.cpp
    internal/shufti.cpp
    internal/state_compress.cpp
    internal/truffle.cpp
    internal/unaligned.cpp
    internal/unicode_set.cpp
    internal/uniform_ops.cpp
    internal/utf8_validate.cpp
    internal/util_string.cpp
    internal/vermicelli.cpp
    internal/main.cpp
    )

if(WIN32 AND (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS))
add_executable(unit-internal ${unit_internal_SOURCES} $<TARGET_OBJECTS:hs_compile_shared> $<TARGET_OBJECTS:hs_exec_shared>)
else()
add_executable(unit-internal ${unit_internal_SOURCES})
endif()
set_target_properties(unit-internal PROPERTIES COMPILE_FLAGS "${HS_CXX_FLAGS}")
target_link_libraries(unit-internal hs corpusomatic)
endif(NOT (RELEASE_BUILD OR FAT_RUNTIME))

if (BUILD_CHIMERA)
    # enable Chimera unit tests
    set(unit_chimera_SOURCES
        ${gtest_SOURCES}
        chimera/allocators.cpp
        chimera/arg_checks.cpp
        chimera/bad_patterns.cpp
        chimera/compat.cpp
        chimera/main.cpp
        chimera/scan.cpp
        )
    add_executable(unit-chimera ${unit_chimera_SOURCES})
    target_link_libraries(unit-chimera chimera hs pcre)
    #
    # build target to run unit tests
    #
    if (NOT RELEASE_BUILD)
    add_custom_target(
        unit
        COMMAND bin/unit-internal
        COMMAND bin/unit-hyperscan
        COMMAND bin/unit-chimera
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS unit-internal unit-hyperscan unit-chimera
    )
    else ()
    add_custom_target(
        unit
        COMMAND bin/unit-hyperscan
        COMMAND bin/unit-chimera
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS unit-hyperscan unit-chimera
    )
    endif()
else()
    #
    # build target to run unit tests
    #
    if (NOT RELEASE_BUILD)
    add_custom_target(
        unit
        COMMAND bin/unit-internal
        COMMAND bin/unit-hyperscan
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS unit-internal unit-hyperscan
    )
    else ()
    add_custom_target(
        unit
        COMMAND bin/unit-hyperscan
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS unit-hyperscan
    )
    endif()
endif()
