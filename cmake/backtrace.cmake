# The `backtrace' function is available on Linux via glibc, and on FreeBSD if
# the 'libexecinfo' package is installed.

CHECK_C_SOURCE_COMPILES(
    "#include <stdlib.h>\n#include <execinfo.h>\nint main () { backtrace(NULL, 0); }"
    BACKTRACE_LIBC)

if(BACKTRACE_LIBC)
    set(HAVE_BACKTRACE TRUE)
    set(BACKTRACE_CFLAGS "")
    set(BACKTRACE_LDFLAGS "")
endif()

if(NOT BACKTRACE_LIBC)
    # FreeBSD 10 has backtrace but requires libexecinfo
    list(INSERT CMAKE_REQUIRED_LIBRARIES 0 "-lexecinfo")
    CHECK_C_SOURCE_COMPILES(
        "#include <stdlib.h>\n#include <execinfo.h>\nint main () { backtrace(NULL, 0); }"
        BACKTRACE_LIBEXECINFO)
    list(REMOVE_ITEM CMAKE_REQUIRED_LIBRARIES "-lexecinfo")

    if(BACKTRACE_LIBEXECINFO)
        set(HAVE_BACKTRACE TRUE)
        set(BACKTRACE_CFLAGS "")
        set(BACKTRACE_LDFLAGS "-lexecinfo")
    else()
        # older FreeBSD requires it from ports
        list(INSERT CMAKE_REQUIRED_INCLUDES 0 "/usr/local/include")
        list(INSERT CMAKE_REQUIRED_LIBRARIES 0 "-L/usr/local/lib -lexecinfo")
        CHECK_C_SOURCE_COMPILES(
            "#include <stdlib.h>\n#include <execinfo.h>\nint main () { backtrace(NULL, 0); }"
            BACKTRACE_LIBEXECINFO_LOCAL)
        list(REMOVE_ITEM CMAKE_REQUIRED_INCLUDES 0 "/usr/local/include")
        list(REMOVE_ITEM CMAKE_REQUIRED_LIBRARIES "-L/usr/local/lib -lexecinfo")
        if(BACKTRACE_LIBEXECINFO_LOCAL)
            set(HAVE_BACKTRACE TRUE)
            set(BACKTRACE_CFLAGS "-I/usr/local/include")
            set(BACKTRACE_LDFLAGS "-L/usr/local/lib -lexecinfo")
        endif()
    endif()
endif()

if(HAVE_BACKTRACE)
    CHECK_C_COMPILER_FLAG(-rdynamic HAS_RDYNAMIC)
    if(HAS_RDYNAMIC)
        list(INSERT BACKTRACE_LDFLAGS 0 -rdynamic)
    endif()
else()
    set(BACKTRACE_CFLAGS "")
    set(BACKTRACE_LDFLAGS "")
endif()

# cmake scope fun
set(HAVE_BACKTRACE ${HAVE_BACKTRACE} CACHE BOOL INTERNAL)
set(BACKTRACE_CFLAGS ${BACKTRACE_CFLAGS} CACHE STRING INTERNAL)
set(BACKTRACE_LDFLAGS ${BACKTRACE_LDFLAGS} CACHE STRING INTERNAL)
