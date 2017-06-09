if (WIN32)
    return()
endif()
find_package(Threads)

# remove some warnings
if(CMAKE_CXX_FLAGS MATCHES "-Wmissing-declarations" )
    string(REPLACE "-Wmissing-declarations" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/util)

# add any subdir with a cmake file
file(GLOB dirents RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(e ${dirents})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${e} AND
       EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${e}/CMakeLists.txt)
        add_subdirectory(${e})
    endif ()
endforeach ()
