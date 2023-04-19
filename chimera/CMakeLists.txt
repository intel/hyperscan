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
message(STATUS "PCRE_LINK_LIBRARIES CHECK: ${PCRE_LINK_LIBRARIES}")
target_link_libraries(chimera hs ${PCRE_LINK_LIBRARIES})

install(TARGETS chimera DESTINATION ${CMAKE_INSTALL_LIBDIR})

if (NOT WIN32)
    # expand out library names for pkgconfig static link info
    foreach (LIB ${CMAKE_CXX_IMPLICIT_LINK_LIBRARIES})
        # this is fragile, but protects us from toolchain specific files
        if (NOT EXISTS ${LIB})
            set(PRIVATE_LIBS "${PRIVATE_LIBS} -l${LIB}")
        endif()
    endforeach()
    set(PRIVATE_LIBS "${PRIVATE_LIBS} -L${LIBDIR} -lpcre")

    configure_file(libch.pc.in libch.pc @ONLY) # only replace @ quoted vars
    install(FILES ${CMAKE_BINARY_DIR}/chimera/libch.pc
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
endif()
