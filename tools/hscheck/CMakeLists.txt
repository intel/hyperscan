# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

SET(hscheck_SOURCES
    main.cpp
)

if (BUILD_CHIMERA)
    include_directories(${PCRE_INCLUDE_DIRS})
    add_definitions(-DHS_HYBRID)
    add_executable(hscheck ${hscheck_SOURCES})
    target_link_libraries(hscheck hs chimera ${PCRE_LDFLAGS} expressionutil pthread)
else()
    add_executable(hscheck ${hscheck_SOURCES})
    target_link_libraries(hscheck hs expressionutil pthread)
endif()

