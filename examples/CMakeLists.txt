find_library(PCAP_LIBRARY pcap)

if (NOT PCAP_LIBRARY)
    message(STATUS "Could not find libpcap - some examples will not be built")
endif()

add_executable(simplegrep simplegrep.c)
set_source_files_properties(simplegrep.c PROPERTIES COMPILE_FLAGS
    "-Wall -Wno-unused-parameter")
target_link_libraries(simplegrep hs)

if (PCAP_LIBRARY)
add_executable(pcapscan pcapscan.cc)
set_source_files_properties(pcapscan.cc PROPERTIES COMPILE_FLAGS
    "-Wall -Wno-unused-parameter")
target_link_libraries(pcapscan hs pcap)
endif()

if (PCAP_LIBRARY)
add_executable(patbench patbench.cc)
set_source_files_properties(patbench.cc PROPERTIES COMPILE_FLAGS
    "-Wall -Wno-unused-parameter")
target_link_libraries(patbench hs pcap)
endif()

install(FILES simplegrep.c pcapscan.cc patbench.cc README.md
        DESTINATION ${CMAKE_INSTALL_DOCDIR}/examples)
