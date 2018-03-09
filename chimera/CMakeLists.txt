# Chimera lib

include_directories(${PCRE_INCLUDE_DIRS})

# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

SET(chimera_HEADERS
    ch.h
    ch_common.h
    ch_compile.h
    ch_runtime.h
)
install(FILES ${chimera_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hs")

SET(chimera_SRCS
    ${chimera_HEADERS}
    ch_alloc.c
    ch_alloc.h
    ch_compile.cpp
    ch_database.c
    ch_database.h
    ch_internal.h
    ch_runtime.c
    ch_scratch.h
    ch_scratch.c
)

add_library(chimera STATIC ${chimera_SRCS})
add_dependencies(chimera hs pcre)
target_link_libraries(chimera hs pcre)
