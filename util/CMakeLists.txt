# utility libs

CHECK_FUNCTION_EXISTS(mmap HAVE_MMAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS} ${HS_CXX_FLAGS}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR})

set_source_files_properties(
    ${CMAKE_BINARY_DIR}/tools/ExpressionParser.cpp
    PROPERTIES
    COMPILE_FLAGS "${RAGEL_C_FLAGS}")

ragelmaker(ExpressionParser.rl)

set(expressionutil_SRCS
    expressions.cpp
    expressions.h
    ExpressionParser.h
    ExpressionParser.cpp
    )
add_library(expressionutil STATIC ${expressionutil_SRCS})
add_dependencies(expressionutil ragel_ExpressionParser)

SET(corpusomatic_SRCS
    ng_corpus_editor.h
    ng_corpus_editor.cpp
    ng_corpus_generator.h
    ng_corpus_generator.cpp
    ng_corpus_properties.h
    ng_corpus_properties.cpp
    ng_find_matches.h
    ng_find_matches.cpp
)
add_library(corpusomatic STATIC ${corpusomatic_SRCS})

set(databaseutil_SRCS
    database_util.cpp
    database_util.h
)
add_library(databaseutil STATIC ${databaseutil_SRCS})

set(crosscompileutil_SRCS
    cross_compile.cpp
    cross_compile.h
    )
add_library(crosscompileutil STATIC ${crosscompileutil_SRCS})
