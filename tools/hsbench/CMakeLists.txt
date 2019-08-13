include (${CMAKE_MODULE_PATH}/sqlite3.cmake)
if (NOT SQLITE3_FOUND)
    message(STATUS "sqlite3 not found, not building hsbench")
    return()
endif()

include_directories(SYSTEM ${SQLITE3_INCLUDE_DIRS})

# BSD has the _np funcs in a _np header
CHECK_INCLUDE_FILE_CXX(pthread_np.h HAVE_PTHREAD_NP_H)
if (HAVE_PTHREAD_NP_H)
    set (PTHREAD_NP_INC pthread_np.h)
else ()
    set (PTHREAD_NP_INC pthread.h)
endif ()

set (CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -D_GNU_SOURCE")
set (CMAKE_REQUIRED_LIBRARIES pthread)
CHECK_CXX_SYMBOL_EXISTS(pthread_setaffinity_np ${PTHREAD_NP_INC} HAVE_DECL_PTHREAD_SETAFFINITY_NP)

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
    engine.cpp
    engine.h
    engine_hyperscan.cpp
    engine_hyperscan.h
    heapstats.cpp
    heapstats.h
    huge.cpp
    huge.h
    main.cpp
    sqldb.cpp
    sqldb.h
    sqldb_bind.h
    thread_barrier.h
    timer.h
)

if (BUILD_CHIMERA)
    add_definitions(-DHS_HYBRID)
    SET(hsbench_SOURCES
        ${hsbench_SOURCES}
        engine_chimera.cpp
        engine_chimera.h
        engine_pcre.cpp
        engine_pcre.h
    )
    add_executable(hsbench ${hsbench_SOURCES})
    include_directories(${PCRE_INCLUDE_DIRS})
    if(NOT WIN32)
        target_link_libraries(hsbench hs chimera ${PCRE_LDFLAGS} databaseutil
            expressionutil ${SQLITE3_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})
    else()
        target_link_libraries(hsbench hs chimera pcre databaseutil
            expressionutil ${SQLITE3_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})
    endif()
else()
    if(WIN32 AND (BUILD_STATIC_AND_SHARED OR BUILD_SHARED_LIBS))
        add_executable(hsbench ${hsbench_SOURCES} $<TARGET_OBJECTS:hs_compile_shared> $<TARGET_OBJECTS:hs_exec_shared>)
    else()
        add_executable(hsbench ${hsbench_SOURCES})
    endif()
    target_link_libraries(hsbench hs databaseutil expressionutil
        ${SQLITE3_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})
endif()
