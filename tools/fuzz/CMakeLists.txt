if(NOT DEFINED ENV{LIB_FUZZING_ENGINE})
  set(FUZZ_DRIVER_SRC onefile.c)
endif()

add_executable(fuzz_simplegrep fuzz_simplegrep.c ${FUZZ_DRIVER_SRC})
target_link_libraries(fuzz_simplegrep hs)

add_executable(fuzz_deserialize_database fuzz_deserialize_database.c ${FUZZ_DRIVER_SRC})
target_link_libraries(fuzz_deserialize_database hs)

if(DEFINED ENV{LIB_FUZZING_ENGINE})
  target_link_libraries(fuzz_simplegrep $ENV{LIB_FUZZING_ENGINE})
  target_link_libraries(fuzz_deserialize_database $ENV{LIB_FUZZING_ENGINE})
endif()
