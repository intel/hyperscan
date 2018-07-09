# first look in pcre-$version or pcre subdirs
if (PCRE_SOURCE)
    # either provided on cmdline or we've seen it already
    set (PCRE_BUILD_SOURCE TRUE)
elseif (EXISTS ${PROJECT_SOURCE_DIR}/pcre-${PCRE_REQUIRED_VERSION})
    set (PCRE_SOURCE ${PROJECT_SOURCE_DIR}/pcre-${PCRE_REQUIRED_VERSION})
    set (PCRE_BUILD_SOURCE TRUE)
elseif (EXISTS ${PROJECT_SOURCE_DIR}/pcre)
    set (PCRE_SOURCE ${PROJECT_SOURCE_DIR}/pcre)
    set (PCRE_BUILD_SOURCE TRUE)
endif()

if (PCRE_BUILD_SOURCE)
    if (NOT IS_ABSOLUTE ${PCRE_SOURCE})
        set(PCRE_SOURCE "${CMAKE_BINARY_DIR}/${PCRE_SOURCE}")
    endif ()
    set (saved_INCLUDES "${CMAKE_REQUIRED_INCLUDES}")
    set (CMAKE_REQUIRED_INCLUDES "${CMAKE_REQUIRED_INCLUDES} ${PCRE_SOURCE}")

    if (PCRE_CHECKED)
        set(PCRE_INCLUDE_DIRS ${PCRE_SOURCE} ${PROJECT_BINARY_DIR}/pcre)
        set(PCRE_LDFLAGS -L"${LIBDIR}" -lpcre)

        # already processed this file and set up pcre building
        return()
    endif ()

    # first, check version number
    CHECK_C_SOURCE_COMPILES("#include <pcre.h.generic>
    #if PCRE_MAJOR != ${PCRE_REQUIRED_MAJOR_VERSION} || PCRE_MINOR != ${PCRE_REQUIRED_MINOR_VERSION}
    #error Incorrect pcre version
    #endif
    main() {}" CORRECT_PCRE_VERSION)
    set (CMAKE_REQUIRED_INCLUDES "${saved_INCLUDES}")

    if (NOT CORRECT_PCRE_VERSION)
        unset(CORRECT_PCRE_VERSION CACHE)
        message(STATUS "Incorrect version of pcre - version ${PCRE_REQUIRED_VERSION} is required")
        return ()
    else()
        message(STATUS "PCRE version ${PCRE_REQUIRED_VERSION} - building from source.")
    endif()

    # PCRE compile options
    option(PCRE_BUILD_PCRECPP OFF)
    option(PCRE_BUILD_PCREGREP OFF)
    option(PCRE_SHOW_REPORT OFF)
    set(PCRE_SUPPORT_UNICODE_PROPERTIES ON CACHE BOOL "Build pcre with unicode")
    add_subdirectory(${PCRE_SOURCE} ${PROJECT_BINARY_DIR}/pcre EXCLUDE_FROM_ALL)
    set(PCRE_INCLUDE_DIRS ${PCRE_SOURCE} ${PROJECT_BINARY_DIR}/pcre)
    set(PCRE_LDFLAGS -L"${LIBDIR}" -lpcre)
else ()
    # pkgconf should save us
    find_package(PkgConfig)
    pkg_check_modules(PCRE libpcre=${PCRE_REQUIRED_VERSION})
    if (PCRE_FOUND)
        set(CORRECT_PCRE_VERSION TRUE)
        message(STATUS "PCRE version ${PCRE_REQUIRED_VERSION}")
    else ()
        message(STATUS "PCRE version ${PCRE_REQUIRED_VERSION} not found")
        return ()
    endif ()
endif (PCRE_BUILD_SOURCE)
