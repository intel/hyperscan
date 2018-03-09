/*
 * Copyright (c) 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <tuple>

#include "gtest/gtest.h"
#include "chimera/ch.h"

using namespace std;
using namespace testing;

namespace {

class HybridScanParams {
public:
    HybridScanParams() {}
    HybridScanParams(const char *s, unsigned int f)
        : patterns(1, s), flags(1, f) {}

    void add(const char *pattern, unsigned int myflags) {
        patterns.push_back(pattern);
        flags.push_back(myflags);
    }

    size_t size() const {
        return patterns.size();
    }

    const char * const * getPatterns() const {
        return &patterns[0];
    }

    const unsigned int * getFlags() const {
        return &flags[0];
    }

private:
    vector<const char *> patterns;
    vector<unsigned int> flags;
};

static
vector<HybridScanParams> paramFactory() {
    vector<HybridScanParams> hsp;

    // Some simple single-pattern cases.
    hsp.push_back(HybridScanParams(".", CH_FLAG_DOTALL));
    hsp.push_back(HybridScanParams("foobar", 0));
    hsp.push_back(HybridScanParams("foo.*bar", 0));
    hsp.push_back(HybridScanParams("fred.*bill", CH_FLAG_DOTALL));
    hsp.push_back(HybridScanParams(".*", 0)); // vacuosity!
    hsp.push_back(HybridScanParams("\\A(.?.{7,27}jf[tmqq]l(f|t|hgmr.+.fg|abks)){3,7}", 0));
    hsp.push_back(HybridScanParams("^begin", CH_FLAG_MULTILINE));
    hsp.push_back(HybridScanParams("match", CH_FLAG_SINGLEMATCH));

    // Single-pattern cases where the pattern isn't supported by hyperscan but
    // can be prefiltered.
    hsp.push_back(HybridScanParams("foo(?!bar)", 0));
    hsp.push_back(HybridScanParams("(sens|respons)e and \\1ibility", 0));

    // A case that can't be prefiltered (as of this writing) because it's too
    // gosh-darned big. This tests that the hybrid matcher can run without the
    // multi-matcher (or with a "fake" one).
    hsp.push_back(HybridScanParams("((c(p|p)h{2,}bh.|p|((((cq|j|c|(\\b)|.[^nbgn]|(\\B)[qfh]a)){10,12}|ih|a|mnde[pa].|.g)){5,8})){3}", 0));

    // Simple multi-pattern literal case.
    hsp.push_back(HybridScanParams());
    hsp.back().add("hatstand", 0);
    hsp.back().add("teakettle", 0);
    hsp.back().add("badgerbrush", 0);
    hsp.back().add("mnemosyne", 0);

    // More complex multi-pattern case.
    hsp.push_back(HybridScanParams());
    hsp.back().add("foo.{3,7}bar", 0);
    hsp.back().add("foo.{30,70}bar", 0);
    hsp.back().add("foobar.*foobar", 0);
    hsp.back().add("^blingwrapper.*foo", 0);
    hsp.back().add("[0-9a-f]{70,}\\n", 0);

    // A couple of trivial Unicode patterns, mostly to make sure we accept
    // the flags.
    hsp.push_back(HybridScanParams());
    hsp.back().add("foo.*bar", CH_FLAG_UTF8);
    hsp.back().add("today", CH_FLAG_UTF8|CH_FLAG_UCP);

    // PCRE exotica.
    hsp.push_back(HybridScanParams());
    hsp.back().add("benign literal", 0);
    hsp.back().add("(?|(abc)|(def))\\1", 0);
    hsp.back().add("(?|(abc)|(def))(?1)", 0);
    hsp.back().add("(sens|respons)e and \\1ibility", 0);
    hsp.back().add("\\w+(?=;)", 0);
    hsp.back().add("foo(?!bar)", 0);
    hsp.back().add("(?<=bullock|donkey)", 0);

    return hsp;
}

// Dummy callback.
static
ch_callback_t dummyHandler(unsigned, unsigned long long, unsigned long long,
                           unsigned, unsigned,const ch_capture_t *, void *) {
    // empty
    return CH_CALLBACK_CONTINUE;
}

static
void checkGroups(unsigned int num, const ch_capture_t *captured) {
    // We should have _some_ group info.
    ASSERT_LT(0U, num);
    ASSERT_TRUE(captured != nullptr);

    // Group 0 is always active.
    ASSERT_TRUE(captured[0].flags & CH_CAPTURE_FLAG_ACTIVE);

    // Sanity-checking.
    for (unsigned int i = 0; i < num; i++) {
        if (!(captured[i].flags & CH_CAPTURE_FLAG_ACTIVE)) {
            continue;
        }
        ASSERT_LE(captured[i].from, captured[i].to) << "Group " << i
                  << "not sane.";
    }
}

// Dummy callback that checks that we had some groups set.
static
ch_callback_t dummyGroupHandler(unsigned, unsigned long long,
                                unsigned long long, unsigned, unsigned num,
                                const ch_capture_t *captured, void *) {
    checkGroups(num, captured);
    return CH_CALLBACK_CONTINUE;
}

class HybridScan : public TestWithParam<tuple<HybridScanParams, bool>> {
protected:
    virtual void SetUp() {
        ch_error_t err;
        ch_compile_error_t *compile_err = nullptr;
        const HybridScanParams &hsp = get<0>(GetParam());
        groups = get<1>(GetParam());

        err = ch_compile_ext_multi(hsp.getPatterns(), hsp.getFlags(), nullptr,
                                   hsp.size(), groups ? CH_MODE_GROUPS :
                                   CH_MODE_NOGROUPS, 10000000, 8000,
                                   nullptr, &db, &compile_err);
        ASSERT_EQ(err, CH_SUCCESS);
        ASSERT_TRUE(db != nullptr);

        err = ch_alloc_scratch(db, &scratch);
        ASSERT_EQ(err, CH_SUCCESS);
        ASSERT_TRUE(scratch != nullptr);
    }

    virtual void TearDown() {
        ch_free_database(db);
        ch_free_scratch(scratch);
    }

    ch_database_t *db = nullptr;
    ch_scratch_t *scratch = nullptr;
    bool groups;
};

static const string SCAN_DATA(
    "Beware the Jabberwock, my son!\n"
    "The jaws that bite, the claws that catch!\n"
    "Beware the Jubjub bird, and shun\n"
    "The frumious Bandersnatch!\n");

TEST_P(HybridScan, BuildAndScan) {
    ASSERT_TRUE(db != nullptr);

    size_t sz;
    ch_error_t err = ch_database_size(db, &sz);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_LT(16U, sz);

    ch_match_event_handler cb = groups ? dummyGroupHandler : dummyHandler;

    err = ch_scan(db, SCAN_DATA.c_str(), SCAN_DATA.length(), 0,
                  scratch, cb, nullptr, nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

TEST_P(HybridScan, ScanNearly4KData) {
    ASSERT_TRUE(db != nullptr);

    string data(4000, '*'); // it's full of stars!

    // Insert some strings that will match a few patterns.
    data.insert(278, "foo");
    data.insert(285, "bar");
    data.insert(1178, "foobar");
    data.insert(1894, "bar");
    data.insert(3000, "foobar");

    ch_match_event_handler cb = groups ? dummyGroupHandler : dummyHandler;

    ch_error_t err = ch_scan(db, data.c_str(), data.length(), 0,
                             scratch, cb, nullptr, nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

TEST_P(HybridScan, ScanBigData) {
    ASSERT_TRUE(db != nullptr);

    // More than 4MB, as that pushes us into using PCRE for non-Pawn cases.
    string data(5*1024*1024, '*'); // it's full of stars!

    // Insert some strings that will match a few patterns.
    data.insert(278, "foo");
    data.insert(285, "bar");
    data.insert(1178, "foobar");
    data.insert(1894, "bar");
    data.insert(3000, "foobar");

    ch_match_event_handler cb = groups ? dummyGroupHandler : dummyHandler;

    ch_error_t err = ch_scan(db, data.c_str(), data.length(), 0,
                             scratch, cb, nullptr, nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

TEST_P(HybridScan, ScanClonedScratch) {
    ASSERT_TRUE(db != nullptr);

    ch_error_t err;
    ch_scratch_t *clonedScratch = nullptr;
    err = ch_clone_scratch(scratch, &clonedScratch);
    ASSERT_EQ(CH_SUCCESS, err);

    ch_match_event_handler cb = groups ? dummyGroupHandler : dummyHandler;

    err = ch_scan(db, SCAN_DATA.c_str(), SCAN_DATA.length(), 0,
                  clonedScratch, cb, nullptr, nullptr);
    ASSERT_EQ(CH_SUCCESS, err);

    ch_free_scratch(clonedScratch);
}

TEST_P(HybridScan, DatabaseInfo) {
    ASSERT_TRUE(db != nullptr);

    char *info = nullptr;
    ch_error_t err = ch_database_info(db, &info);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(info != nullptr);

    const string strinfo(info);
    const string prefix("Chimera ");
    ASSERT_GE(strinfo.size(), prefix.size());
    ASSERT_EQ(prefix, strinfo.substr(0, prefix.size()));

    free(info);
}

TEST_P(HybridScan, NonZeroScratchSize) {
    ASSERT_TRUE(db != nullptr);
    size_t curr_size;
    ch_error_t err = ch_scratch_size(scratch, &curr_size);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_LT(0, curr_size);
}

INSTANTIATE_TEST_CASE_P(Scan, HybridScan,
                        Combine(ValuesIn(paramFactory()), Bool()));

// Counting callback that returns CH_CALLBACK_CONTINUE.
static
ch_callback_t countHandler(unsigned, unsigned long long, unsigned long long,
                           unsigned, unsigned, const ch_capture_t *,
                           void *ctx) {
    unsigned int *count = (unsigned int *)ctx;
    ++(*count);
    return CH_CALLBACK_CONTINUE;
}

// Counting callback that returns CH_CALLBACK_SKIP_PATTERN.
static
ch_callback_t skipHandler(unsigned, unsigned long long, unsigned long long,
                          unsigned, unsigned, const ch_capture_t *,
                          void *ctx) {
    unsigned int *count = (unsigned int *)ctx;
    ++(*count);
    return CH_CALLBACK_SKIP_PATTERN;
}

// Counting callback that returns CH_CALLBACK_TERMINATE.
static
ch_callback_t terminateHandler(unsigned, unsigned long long, unsigned long long,
                               unsigned, unsigned, const ch_capture_t *,
                               void *ctx) {
    unsigned int *count = (unsigned int *)ctx;
    ++(*count);
    return CH_CALLBACK_TERMINATE;
}

static
void makeDatabase(ch_database_t **db, const char * const expr[], size_t num) {
    *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err = ch_compile_ext_multi(expr, nullptr, nullptr, num, 0,
                                          10000000, 8000, nullptr, db,
                                          &compile_err);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(*db != nullptr);
}

struct RescanContext {
    RescanContext(const ch_database_t *db_in, ch_scratch_t *scratch_in)
        : db(db_in), scratch(scratch_in) {}
    const ch_database_t *db;
    ch_scratch_t *scratch;
    size_t matches = 0;
};

static
int rescan_block_cb(unsigned, unsigned long long, unsigned long long, unsigned,
                    unsigned, const ch_capture_t *, void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    const string data = "___foo___bar_";

    hs_error_t err = ch_scan(rctx->db, data.c_str(), data.length(), 0,
                             rctx->scratch, nullptr, nullptr, nullptr);
    EXPECT_EQ(CH_SCRATCH_IN_USE, err);
    return 0;
}

TEST(Scan, ScratchInUse) {
    static const char * const expr[] = { "foo.*bar" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 1);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    RescanContext rc(db, scratch);

    const string data("___foo___bar_");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, rescan_block_cb, 0, &rc);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(1U, rc.matches);

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackSkip1) {
    static const char * const expr[] = { "." };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 1);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("qwertyuiop");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, skipHandler, 0, &count);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(1U, count);

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackSkip2) {
    static const char * const expr[] = { "[a-z]+", "[0-9]" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("foo 0123 0 bar 39483 n34jfhlqekrcoi3q4");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, skipHandler, 0, &count);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(2U, count); // both patterns should match once

    ch_free_scratch(scratch);
    ch_free_database(db);
}

// This case includes a pattern that we use libpcre for.
TEST(Scan, CallbackSkip3) {
    static const char * const expr[] = { "[a-z]+", "foo(?!bar)" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("foobaz foobing foobar");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, skipHandler, 0, &count);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(2U, count); // both patterns should match once

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackNoSkip1) {
    static const char * const expr[] = { "foo|bar", "[0-9]{3}" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("foo 012 bar 345 foobar 678");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, countHandler, 0, &count);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(7U, count); // seven matches in total

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackNoSkip2) {
    static const char * const expr[] = { "foo(?!bar)", "[0-9]{3}" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("foo 012 bar 345 foobar 678");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, countHandler, 0, &count);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_EQ(4U, count); // four matches in total

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackTerm1) {
    static const char * const expr[] = { "." };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 1);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("qwertyuiop");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, terminateHandler, 0, &count);
    ASSERT_EQ(CH_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, count);

    ch_free_scratch(scratch);
    ch_free_database(db);
}

TEST(Scan, CallbackTerm2) {
    static const char * const expr[] = { "[a-z]+", "[0-9]" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != 0);

    unsigned int count = 0;
    const string data("foo 0123 0 bar 39483 n34jfhlqekrcoi3q4");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, terminateHandler, 0, &count);
    ASSERT_EQ(CH_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, count);

    ch_free_scratch(scratch);
    ch_free_database(db);
}

// This case includes a pattern that we use libpcre for.
TEST(Scan, CallbackTerm3) {
    static const char * const expr[] = { "[a-z]+", "foo(?!bar)" };
    ch_database_t *db = nullptr;
    makeDatabase(&db, expr, 2);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int count = 0;
    const string data("foobaz foobing foobar");
    err = ch_scan(db, data.c_str(), data.length(), 0,
                  scratch, terminateHandler, 0, &count);
    ASSERT_EQ(CH_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, count);

    ch_free_scratch(scratch);
    ch_free_database(db);
}

} // namespace
