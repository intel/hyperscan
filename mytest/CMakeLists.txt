add_executable(mytest test1.c)
set_source_files_properties(test1.c PROPERTIES COMPILE_FLAGS
    "-Wall -Wno-unused-parameter")
target_link_libraries(mytest hs)

install(FILES test1.c
        DESTINATION ${CMAKE_INSTALL_DOCDIR}/mytest)