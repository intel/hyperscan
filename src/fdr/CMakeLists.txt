# The set of rules and other nastiness for generating FDR/Teddy source

# we need to add these as explicit dependencies
set(AUTOGEN_PY_FILES
    arch.py
    autogen.py
    autogen_utils.py
    base_autogen.py
    fdr_autogen.py
    teddy_autogen.py
)

function(fdr_autogen type out)
    add_custom_command (
        COMMENT "AUTOGEN ${out}"
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${out}
        COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/autogen.py ${type} > ${CMAKE_CURRENT_BINARY_DIR}/${out}
        DEPENDS ${AUTOGEN_PY_FILES}
        )
    add_custom_target(autogen_${type} DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${out})
endfunction(fdr_autogen)

#now build the functions
fdr_autogen(runtime fdr_autogen.c)
fdr_autogen(compiler fdr_autogen_compiler.cpp)
fdr_autogen(teddy_runtime teddy_autogen.c)
fdr_autogen(teddy_compiler teddy_autogen_compiler.cpp)

set(fdr_GENERATED_SRC
${CMAKE_BINARY_DIR}/src/fdr/fdr_autogen.c
${CMAKE_BINARY_DIR}/src/fdr/fdr_autogen_compiler.cpp
${CMAKE_BINARY_DIR}/src/fdr/teddy_autogen.c
${CMAKE_BINARY_DIR}/src/fdr/teddy_autogen_compiler.cpp
PARENT_SCOPE)

set_source_files_properties(${fdr_GENERATED_SRC} PROPERTIES GENERATED TRUE)
include_directories(${CMAKE_CURRENT_BINARY_DIR})


