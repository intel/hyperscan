# only set these after all tests are done
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

SET(hscheck_SOURCES
    main.cpp
)
add_executable(hscheck ${hscheck_SOURCES})
target_link_libraries(hscheck hs expressionutil pthread)

