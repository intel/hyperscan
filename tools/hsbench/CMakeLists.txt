include (${CMAKE_MODULE_PATH}/sqlite3.cmake)
if (NOT SQLITE3_FOUND)
    message(STATUS "sqlite3 not found, not building hsbench")
    return()
endif()

if (NOT XCODE)
    include_directories(SYSTEM ${SQLITE3_INCLUDE_DIRS})
else()
    # cmake doesn't think Xcode supports isystem
    set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -isystem ${SQLITE3_INCLUDE_DIRS}")
endif()

CHECK_FUNCTION_EXISTS(malloc_info HAVE_MALLOC_INFO)
CHECK_FUNCTION_EXISTS(shmget HAVE_SHMGET)
set(HAVE_SHMGET ${HAVE_SHMGET} CACHE BOOL "shmget()")

# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")


SET(hsbench_SOURCES
    common.h
    data_corpus.cpp
    data_corpus.h
    engine_hyperscan.cpp
    engine_hyperscan.h
    heapstats.cpp
    heapstats.h
    huge.cpp
    huge.h
    main.cpp
    thread_barrier.h
    timer.h
)

add_executable(hsbench ${hsbench_SOURCES})
target_link_libraries(hsbench hs databaseutil expressionutil ${SQLITE3_LDFLAGS}
    ${CMAKE_THREAD_LIBS_INIT})
