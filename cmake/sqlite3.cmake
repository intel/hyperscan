#
# a lot of noise to find sqlite
#

option(SQLITE_PREFER_STATIC "Build sqlite3 statically instead of using an installed lib" OFF)

if(NOT WIN32 AND NOT SQLITE_PREFER_STATIC)
find_package(PkgConfig QUIET)

# first check for sqlite on the system
pkg_check_modules(SQLITE3 sqlite3)
endif()

if (NOT SQLITE3_FOUND)
    message(STATUS "looking for sqlite3 in source tree")
    # look in the source tree
    if (EXISTS "${PROJECT_SOURCE_DIR}/sqlite3/sqlite3.h" AND
            EXISTS "${PROJECT_SOURCE_DIR}/sqlite3/sqlite3.c")
        message(STATUS "  found sqlite3 in source tree")
        set(SQLITE3_FOUND TRUE)
        set(SQLITE3_BUILD_SOURCE TRUE)
        set(SQLITE3_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/sqlite3")
        set(SQLITE3_LDFLAGS sqlite3_static)
    else()
        message(STATUS "  no sqlite3 in source tree")
    endif()
endif()

# now do version checks
if (SQLITE3_FOUND)
    list(INSERT CMAKE_REQUIRED_INCLUDES 0 "${SQLITE3_INCLUDE_DIRS}")
    CHECK_C_SOURCE_COMPILES("#include <sqlite3.h>\n#if SQLITE_VERSION_NUMBER >= 3008007 && SQLITE_VERSION_NUMBER < 3008010\n#error broken sqlite\n#endif\nint main() {return 0;}" SQLITE_VERSION_OK)
    if (NOT SQLITE_VERSION_OK)
        message(FATAL_ERROR "sqlite3 is broken from 3.8.7 to 3.8.10 - please find a working version")
    endif()
if (NOT SQLITE3_BUILD_SOURCE)
    set(_SAVED_FLAGS ${CMAKE_REQUIRED_FLAGS})
    list(INSERT CMAKE_REQUIRED_LIBRARIES 0 ${SQLITE3_LDFLAGS})
    CHECK_SYMBOL_EXISTS(sqlite3_open_v2 sqlite3.h HAVE_SQLITE3_OPEN_V2)
    list(REMOVE_ITEM CMAKE_REQUIRED_INCLUDES "${SQLITE3_INCLUDE_DIRS}")
    list(REMOVE_ITEM CMAKE_REQUIRED_LIBRARIES ${SQLITE3_LDFLAGS})
else()
    if (NOT TARGET sqlite3_static)
    # build sqlite as a static lib to compile into our test programs
    add_library(sqlite3_static STATIC "${PROJECT_SOURCE_DIR}/sqlite3/sqlite3.c")
    if (NOT WIN32)
        set_target_properties(sqlite3_static PROPERTIES COMPILE_FLAGS "-Wno-error -Wno-extra -Wno-unused -Wno-cast-qual -DSQLITE_OMIT_LOAD_EXTENSION")
    endif()
    endif()
endif()
endif()

# that's enough about sqlite
