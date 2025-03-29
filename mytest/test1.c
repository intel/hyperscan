#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <hs.h>

#define PATTERN_COUNT 6
static int eventHandler(unsigned int id, unsigned long long from,
                        unsigned long long to, unsigned int flags, void *ctx) {
    printf("Match for pattern id = %d,  at offset %llu\n", id,
           to);
    return 0;
}
void test_depth(void);
void test_offset(void);
void test_logical_combiantaion(void);
void test_logical_combination_relative_position(void);
void test_depth(void) {
    const char *corpus = "aaaxxaaaxxxxxxxxaaa";
    const char *patterns[1] = {"aaa"};
    unsigned int ids[1] = {1};
    unsigned int flags[1] = {0};
    hs_expr_ext_t e;
    e.flags = HS_EXT_FLAG_MAX_DEPTH;
    e.max_depth = 10;
    const hs_expr_ext_t **exts =
        malloc(1 * sizeof(hs_expr_ext_t *));
    exts[0] =&e;

    hs_database_t *database;
    hs_compile_error_t *compile_err;
    if (hs_compile_ext_multi(patterns, flags, ids, exts, 1, HS_MODE_BLOCK, NULL,
                             &database, &compile_err) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to compile pattern \": %s\n",
                compile_err->message);
        hs_free_compile_error(compile_err);
        return ;
    }
    hs_scratch_t *scratch = NULL;
    if (hs_alloc_scratch(database, &scratch) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to allocate scratch space. Exiting.\n");
        hs_free_database(database);
        return ;
    }
    if (hs_scan(database, corpus, strlen(corpus), 0, scratch, eventHandler, NULL) !=
        HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to scan input buffer. Exiting.\n");
        hs_free_scratch(scratch);
    }
}
void test_offset(void){
    const char *corpus = "aaaxxaaaxxxxxxxxaaa";
    const char *patterns[1] = {"aaa"};
    unsigned int ids[1] = {1};
    unsigned int flags[1] = {0};
    hs_expr_ext_t e;
    e.flags = HS_EXT_FLAG_MIN_OFFSET | HS_EXT_FLAG_MAX_OFFSET;
    e.min_offset = 5;
    e.max_offset = 10;
    const hs_expr_ext_t **exts =
        malloc(1 * sizeof(hs_expr_ext_t *));
    exts[0] =&e;
    hs_database_t *database;
    hs_compile_error_t *compile_err;
    if (hs_compile_ext_multi(patterns, flags, ids, exts, 1, HS_MODE_BLOCK, NULL,
                             &database, &compile_err) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to compile pattern \": %s\n",
                compile_err->message);
        hs_free_compile_error(compile_err);
        return ;
    }
    hs_scratch_t *scratch = NULL;
    if (hs_alloc_scratch(database, &scratch) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to allocate scratch space. Exiting.\n");
        hs_free_database(database);
        return ;
    }
    if (hs_scan(database, corpus, strlen(corpus), 0, scratch, eventHandler, NULL) !=
        HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to scan input buffer. Exiting.\n");
        hs_free_scratch(scratch);
    }
}
void test_logical_combiantaion(void){
    const char *corpus = "aaaxxxbbbxxxxxxxxaaa";
    const char *patterns[7] = {"aaa","bbb","ccc","1 & 2","1 | 2","1 & !2","1 & !3"};
    unsigned int ids[7] = {1,2,3,4,5,6,7};
    unsigned int flags[7] = {HS_FLAG_QUIET,HS_FLAG_QUIET,HS_FLAG_QUIET,HS_FLAG_COMBINATION,HS_FLAG_COMBINATION,HS_FLAG_COMBINATION,HS_FLAG_COMBINATION};
    hs_database_t *database;
    hs_compile_error_t *compile_err;
    if (hs_compile_ext_multi(patterns, flags, ids, NULL, 7, HS_MODE_BLOCK, NULL,
                             &database, &compile_err) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to compile pattern \": %s\n",
                compile_err->message);
        hs_free_compile_error(compile_err);
        return ;
    }
    hs_scratch_t *scratch = NULL;
    if (hs_alloc_scratch(database, &scratch) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to allocate scratch space. Exiting.\n");
        hs_free_database(database);
        return ;
    }
    if (hs_scan(database, corpus, strlen(corpus), 0, scratch, eventHandler, NULL) !=
        HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to scan input buffer. Exiting.\n");
        hs_free_scratch(scratch);
    }
}
//逻辑组合相对位置判断
void test_logical_combination_relative_position(void){
    const char *corpus = "aaaxxxbbbxxxxxxxxbbbxx";
    #define pattern_count 3
    const char *patterns[pattern_count] = {"aaa","bbb","1 & 2"};
    unsigned int ids[pattern_count] = {1,2,3};
    unsigned int flags[pattern_count] = {HS_FLAG_QUIET,HS_FLAG_QUIET,HS_FLAG_COMBINATION};
    hs_expr_ext_t e;
    e.flags = HS_EXT_FLAG_COMBINATION_PRIORITY;
    e.combinationPriorityCount = 1;
    e.combinationPriority =
        malloc(sizeof(hs_combination_subid_priority_t) * 1);
    hs_combination_subid_priority_t p;
    p.frontID = 1;
    p.backID = 2;
    p.distance = 10;
    e.combinationPriority[0] = &p;

    const hs_expr_ext_t **exts =
        malloc(pattern_count* sizeof(hs_expr_ext_t *));
 
    exts[0]=NULL;
    exts[1] =NULL;
    exts[2] =&e;


    hs_database_t *database;
    hs_compile_error_t *compile_err;
    if (hs_compile_ext_multi(patterns, flags, ids, exts, pattern_count, HS_MODE_BLOCK, NULL,
                             &database, &compile_err) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to compile pattern \": %s\n",
                compile_err->message);
        hs_free_compile_error(compile_err);
        return ;
    }
    hs_scratch_t *scratch = NULL;
    if (hs_alloc_scratch(database, &scratch) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to allocate scratch space. Exiting.\n");
        hs_free_database(database);
        return ;
    }
    if (hs_scan(database, corpus, strlen(corpus), 0, scratch, eventHandler, NULL) !=
        HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to scan input buffer. Exiting.\n");
        hs_free_scratch(scratch);
    }
}
int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <pattern> <input file>\n", argv[0]);
        return -1;
    }
    test_depth();
    test_offset();
    test_logical_combiantaion();
    test_logical_combination_relative_position();
}
