# dump support is required
if (NOT DUMP_SUPPORT)
    return()
endif ()

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/util)

# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

if(WIN32 AND (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS))
    add_executable(hsdump main.cpp $<TARGET_OBJECTS:hs_compile_shared> $<TARGET_OBJECTS:hs_exec_shared>)
else()
    add_executable(hsdump main.cpp)
endif()
target_link_libraries(hsdump hs expressionutil crosscompileutil)

