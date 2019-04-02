#include "stdio.h"
#include "hs.h"

// note this function is never called, but without it this code works fine
static int __cdecl eventHandler(unsigned int id, unsigned long long from,
                        unsigned long long to, unsigned int flags, void *ctx) {
    printf("Match for pattern");
    return 0;
}

char* crs_patterns[] = {
  ".",
  "dummy2",
};

static int __cdecl blowupEventHandler(unsigned int id, unsigned long long from,
    unsigned long long to, unsigned int flags, void *ctx)
{
    return 1;
}


int __cdecl main(int argc, char* argv[])
{
    hs_database_t* pRegexDatabase = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t hsError = hs_compile(
        "Sample?Value",
        0,
        HS_MODE_BLOCK,
        nullptr,
        &pRegexDatabase,
        &compile_err);
    if (hsError != HS_SUCCESS)
    {
        printf("Dead 1\n");
        exit(-2);
    }

    hs_scratch_t* pScratch = nullptr;
    hsError = hs_alloc_scratch(pRegexDatabase, &pScratch);
    if (hsError != HS_SUCCESS)
    {
        printf("Dead 2\n");
        exit(-2);
    }

    bool matched = false;
    printf("Scan 1\n");
    hsError = hs_scan(
        pRegexDatabase,
        "Sample?Value",
        12,
        0,
        pScratch,
        blowupEventHandler,
        &matched);
    if (hsError != HS_SUCCESS)
    {
        printf("Dead 3\n");
        exit(-2);
    }

    printf("Scan 2\n");
    hsError = hs_scan(
        pRegexDatabase,
        "SampleeValue",
        12,
        0,
        pScratch,
        blowupEventHandler,
        &matched);
    if (hsError != HS_SUCCESS)
    {
        printf("Dead 4\n");
        exit(-2);
    }

    printf("good exit");
    return 0;
}
