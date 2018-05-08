/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "config.h"

#include "ResultSet.h"
#include "UltimateTruth.h"
#include "util/database_util.h"
#include "util/ExpressionParser.h"
#include "util/string_util.h"

#include "ue2common.h"
#include "common.h"
#include "crc32.h"
#include "hs.h"
#include "hs_internal.h"
#include "util/make_unique.h"

#include "scratch.h"
#include "nfa/nfa_api_queue.h"
#include "rose/rose_internal.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

using namespace std;
using namespace ue2;
using boost::ptr_vector;

#ifndef RELEASE_BUILD

#include "database.h"
#include "state.h"

static
hs_error_t open_magic_stream(const hs_database_t *db, unsigned flags,
                             hs_stream_t **stream, hs_scratch_t *scratch,
                             unsigned long long start_offset) {
    hs_error_t ret = hs_open_stream(db, flags, stream);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    const char dummy_data[100] = { 0 };
    UNUSED const struct RoseEngine *rose
        = (const struct RoseEngine *)hs_get_bytecode(db);
    assert(sizeof(dummy_data) >= rose->historyRequired);
    hs_scan_stream(*stream, dummy_data, MIN(start_offset, sizeof(dummy_data)), 0,
                   scratch, nullptr, nullptr);
    (*stream)->offset = start_offset;
    return ret;
}

#endif // RELEASE_BUILD

class BaseDB : boost::noncopyable {
public:
    // Constructor takes iterators over a container of pattern IDs.
    template <class Iter>
    BaseDB(Iter ids_begin, Iter ids_end)
        : ids(ids_begin, ids_end) {}

    virtual ~BaseDB();

    // The set of expression IDs that must return their matches in order.
    unordered_set<unsigned> ordered;

    // The complete set of expression IDs associated with this database.
    unordered_set<unsigned> ids;
};

BaseDB::~BaseDB() { }

class HyperscanDB : public BaseDB {
public:
    // Constructor takes iterators over a container of pattern IDs.
    template <class Iter>
    HyperscanDB(hs_database_t *db_in, Iter ids_begin, Iter ids_end)
        : BaseDB(ids_begin, ids_end), db(db_in) {}

    ~HyperscanDB();

    // Underlying Hyperscan database pointer.
    hs_database_t *db;
};

HyperscanDB::~HyperscanDB() {
    hs_free_database(db);
}

#ifdef HS_HYBRID

class HybridDB : public BaseDB {
public:
    // Constructor takes iterators over a container of pattern IDs.
    template <class Iter>
    HybridDB(ch_database_t *db_in, Iter ids_begin, Iter ids_end)
        : BaseDB(ids_begin, ids_end), db(db_in) {}

    ~HybridDB();

    // Underlying Hyperscan database pointer.
    ch_database_t *db;
};

HybridDB::~HybridDB() {
    ch_free_database(db);
}

#endif // HS_HYBRID

// Used to track the ID and result set.
namespace {
struct MultiContext {
    MultiContext(unsigned int id_in, const BaseDB &db_in, ResultSet *rs_in,
                 bool single_in, ostream &os)
        : id(id_in), db(db_in), rs(rs_in), single(single_in), out(os) {}
    unsigned int id;
    int block = 0;
    const BaseDB &db;
    ResultSet *rs;
    u64a lastRawMatch = 0; /* store last known unadjusted match location */
    u64a lastOrderMatch = 0;
    bool single;
    bool use_max_offset = false;
    unsigned long long max_offset = 0; /* don't record matches beyond this */
    bool terminated = false; //!< user has instructed us to stop
    bool in_scan_call = false;
    ostream &out;
};
}

// Callback used for all (both single and multi-mode) scans.
static
int HS_CDECL callbackMulti(unsigned int id, unsigned long long from,
                           unsigned long long to,
                           UNUSED unsigned int flags, void *ctx) {
    MultiContext *mctx = static_cast<MultiContext *>(ctx);
    assert(mctx);
    assert(mctx->rs);
    assert(mctx->in_scan_call);

    ostream &out = mctx->out;

    // Sanity check: in single mode, we'd better not be getting matches for the
    // wrong ID!
    if (mctx->single && id != mctx->id) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id
            << " which is not the id we're looking for" << endl;
        mctx->rs->invalid_id = true;
        return 1;
    }

    // In any mode, we should NEVER get a match from an ID outside our known set.
    if (mctx->db.ids.find(id) == mctx->db.ids.end()) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id
            << " which is not in the pattern set" << endl;
        mctx->rs->invalid_id = true;
        return 1;
    }

    if (mctx->terminated) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id
            << " after termination" << endl;
        mctx->rs->match_after_halt = true;
    }

#ifndef RELEASE_BUILD
    unsigned int adjustment = flags & HS_MATCH_FLAG_ADJUSTED ? 1 : 0;
    if (mctx->lastRawMatch > to + adjustment) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id
            << " unordered" << endl;
        mctx->rs->uoom = true;
    }
    mctx->lastRawMatch = to + adjustment;
#endif

    if (mctx->db.ordered.find(id) != mctx->db.ordered.end()) {
        if (mctx->lastOrderMatch > to) {
            out << "UE2 Match @ (" << from << "," << to << ") for " << id
                << " unordered" << endl;
            mctx->rs->uoom = true;
        }
        mctx->lastOrderMatch = to;
    }

    if (mctx->use_max_offset && to > mctx->max_offset) {
        if (echo_matches) {
            out << "UE2 Match @ (" << from << "," << to << ") for " << id
                << " ignored" << endl;
        }
        return 0;
    }

    if (to - g_streamOffset < g_corpora_prefix.size()) {
        if (echo_matches) {
            out << "UE2 Match @ (" << from << "," << to << ") for " << id
                << " too early" << endl;
        }
        return 0;
    }

    u64a offsetDelta = g_corpora_prefix.size() + g_streamOffset;

    if (from) {
        // from only set in SOM mode, otherwise zero. If we wanted to be REALLY
        // principled about this, we'd probably want to stash the flags
        // somewhere at compile time.
        from -= (from > offsetDelta ? offsetDelta : from);
    }

    to -= offsetDelta;

    if (echo_matches) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id << endl;
    }

    if (mctx->single || id == mctx->id) {
        mctx->rs->addMatch(from, to, mctx->block);
        if (limit_matches && mctx->rs->matches.size() == limit_matches) {
            if (echo_matches) {
                out << "Terminating matching (hit match limit)" << endl;
            }
            mctx->terminated = true;
            return 1; // terminate matching.
        }
    }

    return 0;
}

#ifdef HS_HYBRID

// Hybrid matcher callback.
static
ch_callback_t HS_CDECL callbackHybrid(unsigned id, unsigned long long from,
                             unsigned long long to, unsigned, unsigned size,
                             const ch_capture_t *captured, void *ctx) {
    MultiContext *mctx = static_cast<MultiContext *>(ctx);
    assert(mctx);
    assert(mctx->rs);
    assert(mctx->in_scan_call);

    ostream &out = mctx->out;

    to -= g_corpora_prefix.size();

    if (mctx->terminated) {
        out << "UE2 Match @ (" << from << "," << to << ") for " << id
            << " after termination" << endl;
        mctx->rs->match_after_halt = true;
    }

    if (mctx->single || id == mctx->id) {
        CaptureVec cap;
        for (unsigned int i = 0; i < size; i++) {
            if (!(captured[i].flags & CH_CAPTURE_FLAG_ACTIVE)) {
                cap.push_back(make_pair(-1, -1));
            } else {
                cap.push_back(make_pair(captured[i].from, captured[i].to));
            }
        }
        mctx->rs->addMatch(from, to, cap);
    }

    if (echo_matches) {
        out << "Match @ [" << from << "," << to << "] for " << id << endl;
        out << "  Captured " << size << " groups: ";
        for (unsigned int i = 0; i < size; i++) {
            if (!(captured[i].flags & CH_CAPTURE_FLAG_ACTIVE)) {
                out << "{} ";
            } else {
                out << "{" << captured[i].from << "," << captured[i].to << "} ";
            }
        }
        out << endl;
    }

    if (limit_matches && mctx->rs->matches.size() == limit_matches) {
        mctx->terminated = true;
        return CH_CALLBACK_TERMINATE;
    }

    return CH_CALLBACK_CONTINUE;
}

// Hybrid matcher error callback.
static
ch_callback_t HS_CDECL errorCallback(UNUSED ch_error_event_t errorType,
                                     UNUSED unsigned int id, void *,
                                     void *ctx) {
    UNUSED MultiContext *mctx = static_cast<MultiContext *>(ctx);
    assert(mctx);
    assert(mctx->rs);
    assert(mctx->in_scan_call);

    return CH_CALLBACK_SKIP_PATTERN;
}

#endif // HS_HYBRID

static
void filterLeftmostSom(ResultSet &rs) {
    if (rs.matches.size() <= 1) {
        return;
    }

    set<u64a> seen; // End offsets.
    auto it = rs.matches.begin();
    while (it != rs.matches.end()) {
        if (seen.insert(it->to).second) {
            ++it; // First time we've seen this end-offset.
        } else {
            rs.matches.erase(it++);
        }
    }
}

UltimateTruth::UltimateTruth(ostream &os, const ExpressionMap &expr,
                             const hs_platform_info_t *plat,
                             const Grey &grey_in, unsigned int streamBlocks)
    : grey(grey_in), out(os), m_expr(expr), m_xcompile(false),
      m_streamBlocks(streamBlocks), scratch(nullptr),
#ifdef HS_HYBRID
      chimeraScratch(nullptr),
#endif
      platform(plat) {
    // Build our mode flags.

    switch (colliderMode) {
    case MODE_STREAMING:
        m_mode = HS_MODE_STREAM;
        break;
    case MODE_BLOCK:
        m_mode = HS_MODE_BLOCK;
        break;
    case MODE_VECTORED:
        m_mode = HS_MODE_VECTORED;
        break;
    case MODE_HYBRID:
        m_mode = 0;
        break;
    }

    // Set desired SOM precision, if we're in streaming mode.
    if (colliderMode == MODE_STREAMING) {
        m_mode |= somPrecisionMode;
    }

#ifdef HS_HYBRID
    if (colliderMode == MODE_HYBRID && !no_groups) {
        m_mode |= CH_MODE_GROUPS;
    }
#endif
}

UltimateTruth::~UltimateTruth() {
#ifdef HS_HYBRID
    ch_free_scratch(chimeraScratch);
#endif
    hs_free_scratch(scratch);
}

static
void mangle_scratch(hs_scratch_t *scratch) {
    /* Use our knowledge of the internals of scratch to make a mess */

    memset(&scratch->tctxt, 0xc0, sizeof(scratch->tctxt));
    memset(scratch->bstate, 0xd0, scratch->bStateSize);
    memset(scratch->tstate, 0xe0, scratch->tStateSize);
    memset(scratch->fullState, 0xf0, scratch->fullStateSize);

    for (u32 i = 0; i < scratch->queueCount; i++) {
        struct mq *q = &scratch->queues[i];
        memset(q, 0x01, sizeof(*q));
        q->scratch = scratch;
    }

    memset(scratch->aqa, 0xb0, scratch->activeQueueArraySize);
    for (u32 i = 0; i < DELAY_SLOT_COUNT; i++) {
        memset(scratch->delay_slots[i], 0x05, scratch->delay_fatbit_size);
    }

    memset(scratch->catchup_pq.qm, 0x06,
           scratch->queueCount * sizeof(struct queue_match));
    scratch->catchup_pq.qm_size = 45;
    memset(&scratch->core_info, 0x07, sizeof(scratch->core_info));
    memset(scratch->deduper.som_start_log[0], 0x90,
           sizeof(u64a) * scratch->deduper.dkey_count);
    memset(scratch->deduper.som_start_log[1], 0x09,
           sizeof(u64a) * scratch->deduper.dkey_count);
    memset(scratch->deduper.log[0], 0xa0, scratch->deduper.log_size);
    memset(scratch->deduper.log[1], 0x0a, scratch->deduper.log_size);
    memset(scratch->deduper.som_log[0], 0xd0, scratch->deduper.log_size);
    memset(scratch->deduper.som_log[1], 0x0d, scratch->deduper.log_size);

    for (u32 i = 0; i < scratch->anchored_literal_region_len; i++) {
        memset(scratch->al_log[i], 0xa0, scratch->anchored_literal_fatbit_size);
    }
    scratch->al_log_sum=0xf0f;

    memset(scratch->handled_roles, 0x05, scratch->handledKeyFatbitSize);
    memset(scratch->som_store, 0x06,
           scratch->som_store_count * sizeof(u64a));
    memset(scratch->som_attempted_store, 0x06,
           scratch->som_store_count * sizeof(u64a));
    memset(scratch->som_set_now, 0x03, scratch->som_fatbit_size);
    memset(scratch->som_attempted_set, 0x04, scratch->som_fatbit_size);
    scratch->som_set_now_offset = 45;
    memset(&scratch->fdr_conf, 0x0d, sizeof(scratch->fdr_conf));
    scratch->fdr_conf_offset = 0xe4;
}

bool UltimateTruth::blockScan(const BaseDB &bdb, const string &buffer,
                              size_t align, match_event_handler callback,
                              void *ctx_in, ResultSet *) {
    assert(colliderMode == MODE_BLOCK);
    assert(!m_xcompile);

    const hs_database_t *db = reinterpret_cast<const HyperscanDB &>(bdb).db;
    assert(db);
    MultiContext *ctx = (MultiContext *)ctx_in;

    char *realigned = setupScanBuffer(buffer.c_str(), buffer.size(), align);
    if (!realigned) {
        return false;
    }

    if (use_copy_scratch && !cloneScratch()) {
        return false;
    }

    ctx->in_scan_call = true;
    hs_error_t ret =
        hs_scan(db, realigned, buffer.size(), 0, scratch, callback, ctx);
    ctx->in_scan_call = false;

    if (g_verbose) {
        out << "Scan call returned " << ret << endl;
    }

    if (ctx->terminated) {
        if (g_verbose && ret != HS_SCAN_TERMINATED) {
            out << "Scan should have returned HS_SCAN_TERMINATED, returned "
                 << ret << " instead." << endl;
        }
        return ret == HS_SCAN_TERMINATED;
    }

    if (g_verbose && ret != HS_SUCCESS) {
        out << "Scan should have returned HS_SUCCESS, returned " << ret
             << " instead." << endl;
    }

    if (use_mangle_scratch) {
        mangle_scratch(scratch);
    }

    return ret == HS_SUCCESS;
}

static
vector<char> compressAndCloseStream(hs_stream_t *stream) {
    size_t needed;
    hs_error_t err = hs_compress_stream(stream, nullptr, 0, &needed);
    if (err != HS_INSUFFICIENT_SPACE) {
        return {};
    }

    vector<char> buf(needed);
    err = hs_compress_stream(stream, buf.data(), needed, &needed);
    if (err != HS_SUCCESS) {
        return {};
    }
    assert(needed == buf.size());

    err = hs_close_stream(stream, nullptr, nullptr, nullptr);
    if (err != HS_SUCCESS) {
        return {};
    }

    return buf;
}


static
hs_stream_t *compressAndExpandStream(const hs_database_t *db,
                                     hs_stream_t *stream) {
    vector<char> buf = compressAndCloseStream(stream);
    hs_stream_t *out;
    hs_error_t err = hs_expand_stream(db, &out, buf.data(), buf.size());

    if (err != HS_SUCCESS) {
        return nullptr;
    }

    return out;
}

static
hs_stream_t *compressAndResetExpandStream(const hs_database_t *db,
                                          hs_stream_t *stream) {
    vector<char> buf = compressAndCloseStream(stream);
    if (buf.empty()) {
        return nullptr;
    }

    hs_stream_t *out;

    hs_error_t err = hs_open_stream(db, 0, &out);

    if (err != HS_SUCCESS) {
        return nullptr;
    }

    err = hs_reset_and_expand_stream(out, buf.data(), buf.size(), nullptr,
                                     nullptr, nullptr);
    if (err != HS_SUCCESS) {
        return nullptr;
    }

    return out;
}

bool UltimateTruth::streamingScan(const BaseDB &bdb, const string &buffer,
                                  size_t align, match_event_handler callback,
                                  void *ctx_in, ResultSet *rs) {
    assert(colliderMode == MODE_STREAMING);
    assert(!m_xcompile);

    const hs_database_t *db = reinterpret_cast<const HyperscanDB &>(bdb).db;
    assert(db);
    MultiContext *ctx = (MultiContext *)ctx_in;

    // open a stream
    hs_stream_t *stream;
    size_t stream_size;
    int ret;

    ret = hs_stream_size(db, &stream_size);
    if (ret != HS_SUCCESS) {
        out << "Unable to size stream." << endl;
        return false;
    }

    if (!g_streamOffset) {
        ret = hs_open_stream(db, 0, &stream);
    } else {
#ifndef RELEASE_BUILD
        ret = open_magic_stream(db, 0, &stream, scratch, g_streamOffset);
#else
        ret = HS_INVALID;
#endif
    }

    if (ret != HS_SUCCESS) {
        out << "Unable to open stream." << endl;
        return false;
    }

    // scan our data, split into blocks and copied into a temporary buffer
    // aligned as requested (out of paranoia)
    unsigned blockSize = buffer.size() / m_streamBlocks;
    if (blockSize == 0) {
        blockSize = 1;
    }
    const char *ptr = buffer.c_str();
    const char *end = ptr + buffer.size();
    ctx->block = 0;

    // We use a do-while loop here so that zero-byte cases still generate at
    // least one hs_scan_stream call, since it's something users might try.
    do {
        if (ptr + blockSize > end) {
            // last write is a runt
            blockSize = end - ptr;
        }
        char *realigned = setupScanBuffer(ptr, blockSize, align);
        if (!realigned) {
            return false;
        }
        ctx->in_scan_call = true;
        DEBUG_PRINTF("scan stream write %u\n", ctx->block);
        ret = hs_scan_stream(stream, realigned, blockSize, 0, scratch,
                             callback, ctx);
        DEBUG_PRINTF("scan %u done\n", ctx->block);
        ctx->in_scan_call = false;

        if (limit_matches && rs->matches.size() == limit_matches) {
            if (ret != HS_SCAN_TERMINATED) {
                DEBUG_PRINTF("failure to scan %d\n", ret);
                return false;
            }
        } else if (ret != HS_SUCCESS) {
            DEBUG_PRINTF("failure to scan %d\n", ret);
            return false;
        }

        if (use_copy_scratch && !cloneScratch()) {
            return false;
        }

        if (use_copy_stream) {
            hs_stream_t *s2;
            ret = hs_copy_stream(&s2, stream);
            if (ret != HS_SUCCESS) {
                DEBUG_PRINTF("failure to copy %d\n", ret);
                return false;
            }
            /* do a short write to the old stream so that it is in the wrong
             * state. */
            char temp[2] = {0, 0};
            ret = hs_scan_stream(stream, temp, sizeof(temp), 0, scratch,
                                 nullptr, nullptr);

            hs_error_t expected = HS_SUCCESS;
            if (limit_matches && rs->matches.size() == limit_matches) {
                expected = HS_SCAN_TERMINATED;
            }
            if (ret != expected) {
                DEBUG_PRINTF("failure to scan %d\n", ret);
                return false;
            }
            ret = hs_close_stream(stream, nullptr, nullptr, nullptr);
            if (ret != HS_SUCCESS) {
                DEBUG_PRINTF("failure to close %d\n", ret);
                return false;
            }
            stream = s2;
        }
        if (use_mangle_scratch) {
            mangle_scratch(scratch);
        }

        if (use_compress_expand) {
            auto rv = compressAndExpandStream(db, stream);
            if (!rv) {
                if (g_verbose) {
                    out << "Compress/Expand failed." << endl;
                }
                return false;
            } else {
                stream = rv;
            }
        }

        if (use_compress_reset_expand) {
            auto rv = compressAndResetExpandStream(db, stream);
            if (!rv) {
                if (g_verbose) {
                    out << "Compress/Expand failed." << endl;
                }
                return false;
            } else {
                stream = rv;
            }
        }

        ptr += blockSize;
        ctx->block++;
    } while (ptr < end);

    // close the stream
    ctx->in_scan_call = true;
    DEBUG_PRINTF("close stream %u\n", ctx->block);
    ret = hs_close_stream(stream, scratch, callback, ctx);
    DEBUG_PRINTF("close stream done\n");
    ctx->in_scan_call = false;

    if (ret != HS_SUCCESS) {
        return false;
    }

    // UE2 cannot dedupe SOM matches across stream boundaries, so we must
    // filter them out.
    filterLeftmostSom(*rs);

    return ret == HS_SUCCESS;
}

bool UltimateTruth::vectoredScan(const BaseDB &bdb, const string &buffer,
                                 size_t align, match_event_handler callback,
                                 void *ctx_in, ResultSet *rs) {
    assert(colliderMode == MODE_VECTORED);
    assert(!m_xcompile);

    const hs_database_t *db = reinterpret_cast<const HyperscanDB &>(bdb).db;
    assert(db);
    MultiContext *ctx = (MultiContext *)ctx_in;

    int ret;

    assert(!g_streamOffset);

    // scan our data, split into blocks and copied into a temporary buffer
    // aligned as requested (out of paranoia)
    unsigned blockSize = buffer.size() / m_streamBlocks;
    if (blockSize == 0) {
        blockSize = 1;
    }
    const char *ptr = buffer.c_str();
    const char *end = ptr + buffer.size();
    ctx->block = 0;

    // We use a do-while loop here so that zero-byte cases still generate at
    // least one hs_scan_stream call, since it's something users might try.

    vector<const char *> data;
    vector<unsigned int> length;

    u32 block_count = (buffer.size() + blockSize - 1) / blockSize;
    block_count = MAX(block_count, 1);

    if (block_count > raw_blocks.size()) {
        raw_blocks.resize(block_count);
    }

    do {
        if (ptr + blockSize > end) {
            // last write is a runt
            blockSize = end - ptr;
        }
        char *realigned = setupVecScanBuffer(ptr, blockSize, align, ctx->block);
        if (!realigned) {
            return false;
        }

        data.push_back(realigned);
        length.push_back(blockSize);

        ptr += blockSize;
        ctx->block++;

    } while (ptr < end);

    if (use_copy_scratch && !cloneScratch()) {
        return false;
    }

    DEBUG_PRINTF("scan vectored write %u\n", ctx->block);
    ctx->in_scan_call = true;
    ret = hs_scan_vector(db, &data[0], &length[0], ctx->block, 0, scratch,
                         callback, ctx);
    ctx->in_scan_call = false;
    DEBUG_PRINTF("scan %u done\n", ctx->block);
    if (use_mangle_scratch) {
        mangle_scratch(scratch);
    }

    rs->dupe_matches.clear(); /* TODO: dedupe across vectored blocks */

    if (limit_matches && rs->matches.size() == limit_matches) {
        if (ret != HS_SCAN_TERMINATED) {
            DEBUG_PRINTF("failure to scan %d\n", ret);
            return false;
        }
    } else if (ret != HS_SUCCESS) {
        DEBUG_PRINTF("failure to scan %d\n", ret);
        return false;
    }

    // UE2 cannot dedupe SOM matches across vector block boundaries, so we must
    // filter them out.
    filterLeftmostSom(*rs);

    return true;
}

#ifdef HS_HYBRID
bool UltimateTruth::hybridScan(const BaseDB &bdb, const string &buffer,
                               size_t align, ch_match_event_handler callback,
                               ch_error_event_handler error_callback,
                               void *ctx_in, ResultSet *) {
    assert(colliderMode == MODE_HYBRID);
    assert(!m_xcompile);

    const ch_database_t *db = reinterpret_cast<const HybridDB &>(bdb).db;
    assert(db);
    MultiContext *ctx = (MultiContext *)ctx_in;

    char *realigned = setupScanBuffer(buffer.c_str(), buffer.size(), align);
    if (!realigned) {
        return false;
    }

    if (use_copy_scratch && !cloneScratch()) {
        return false;
    }

    ctx->in_scan_call = true;
    ch_error_t ret =
        ch_scan(db, realigned, buffer.size(), 0, chimeraScratch, callback,
                error_callback, ctx);
    ctx->in_scan_call = false;

    if (g_verbose) {
        out << "Scan call returned " << ret << endl;
    }

    if (ctx->terminated) {
        if (g_verbose && ret != CH_SCAN_TERMINATED) {
            out << "Scan should have returned CH_SCAN_TERMINATED, returned "
                 << ret << " instead." << endl;
        }
        return ret == CH_SCAN_TERMINATED;
    }

    if (g_verbose && ret != CH_SUCCESS) {
        out << "Scan should have returned CH_SUCCESS, returned " << ret
             << " instead." << endl;
    }

    return ret == CH_SUCCESS;
}
#endif

bool UltimateTruth::run(unsigned int id, shared_ptr<const BaseDB> bdb,
                        const string &buffer, bool single_pattern,
                        unsigned int align, ResultSet &rs) {
    assert(!m_xcompile);
    assert(bdb);

    // Ensure that scratch is appropriate for this database.
    if (!allocScratch(bdb)) {
        out << "Scratch alloc failed." << endl;
        return false;
    }

    MultiContext ctx(id, *bdb, &rs, single_pattern, out);
    if (!g_corpora_suffix.empty()) {
        ctx.use_max_offset = true;
        ctx.max_offset = buffer.size() - g_corpora_suffix.size();
    }

    switch (colliderMode) {
    case MODE_BLOCK:
        return blockScan(*bdb, buffer, align, callbackMulti, &ctx, &rs);
    case MODE_STREAMING:
        return streamingScan(*bdb, buffer, align, callbackMulti, &ctx, &rs);
    case MODE_VECTORED:
        return vectoredScan(*bdb, buffer, align, callbackMulti, &ctx, &rs);
    case MODE_HYBRID:
#ifdef HS_HYBRID
        return hybridScan(*bdb, buffer, align, callbackHybrid, errorCallback,
                          &ctx, &rs);
#else
        cerr << "Hybrid mode not available in this build." << endl;
        abort();
#endif
        break;
    }

    assert(0);
    return false;
}

static
bool isOrdered(const string &expr, unsigned int flags) {
    // SOM doesn't produce ordered matches?
    if (flags & HS_FLAG_SOM_LEFTMOST) {
        return false;
    }

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *error = nullptr;
    hs_error_t err = hs_expression_info(expr.c_str(), flags, &info, &error);
    if (err != HS_SUCCESS) {
        // Expression will fail compilation and report error elsewhere.
        free(info);
        hs_free_compile_error(error);
        return false;
    }

    assert(info);

    // Any pattern that does not require offset adjustment should produce
    // matches in order.
    bool ordered = !info->unordered_matches;
    free(info);
    return ordered;
}

static unique_ptr<BaseDB>
compileHyperscan(vector<const char *> &patterns, vector<unsigned> &flags,
                 vector<unsigned> &idsvec, ptr_vector<hs_expr_ext> &ext,
                 unsigned mode, const hs_platform_info *platform, string &error,
                 const Grey &grey) {
    const unsigned count = patterns.size();
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err;

    hs_error_t err = hs_compile_multi_int(&patterns[0], &flags[0],
                                          &idsvec[0], ext.c_array(), count,
                                          mode, platform, &db,
                                          &compile_err, grey);

    if (err != HS_SUCCESS) {
        error = compile_err->message;
        hs_free_compile_error(compile_err);
        return nullptr;
    }

    return ue2::make_unique<HyperscanDB>(db, idsvec.begin(), idsvec.end());
}

#ifdef HS_HYBRID
static unique_ptr<BaseDB>
compileHybrid(vector<const char *> &patterns,
              vector<unsigned> &flags, vector<unsigned> &idsvec,
              unsigned mode, const hs_platform_info *platform, string &error) {
    const unsigned count = patterns.size();
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err;

    ch_error_t err = ch_compile_multi(&patterns[0], &flags[0],
                                      &idsvec[0], count, mode, platform, &db,
                                      &compile_err);

    if (err != HS_SUCCESS) {
        error = compile_err->message;
        ch_free_compile_error(compile_err);
        return nullptr;
    }

    return ue2::make_unique<HybridDB>(db, idsvec.begin(), idsvec.end());
}
#endif

shared_ptr<BaseDB> UltimateTruth::compile(const set<unsigned> &ids,
                                               string &error) const {
    // Build our vectors for compilation
    const size_t count = ids.size();
    vector<string> expressions(count);
    vector<unsigned> idsvec(ids.begin(), ids.end());
    vector<unsigned> flags(count);
    vector<bool> check_ordered(count, false);
    ptr_vector<hs_expr_ext> ext;
    ext.reserve(count);

    size_t n = 0;
    for (const auto &id : ids) {
        auto j = m_expr.find(id);
        if (j == m_expr.end()) {
            error = "Unable to find ID.";
            return nullptr;
        }

        ext.push_back(new hs_expr_ext);
        bool must_be_ordered;
        if (!readExpression(j->second, expressions[n], &flags[n], &ext[n],
                            &must_be_ordered)) {
            ostringstream oss;
            oss << "Unable to decode flags: '" << j->first << ":"
                << j->second << "'.";
            error = oss.str();
            return nullptr;
        }

        check_ordered[n] = must_be_ordered;

        if (force_utf8) {
            flags[n] |= HS_FLAG_UTF8;
        }

        if (force_prefilter) {
            flags[n] |= HS_FLAG_PREFILTER;
        }

        if (somFlags) {
            flags[n] |= somFlags;
        }

        if (force_edit_distance) {
            ext[n].flags |= HS_EXT_FLAG_EDIT_DISTANCE;
            ext[n].edit_distance = edit_distance;
        }

        if (colliderMode == MODE_HYBRID) {
            if (ext[n].flags) {
                error = "Hybrid does not support extended parameters.";
                return nullptr;
            }
            // We can also strip some other flags in the hybrid matcher.
            flags[n] &= ~HS_FLAG_PREFILTER; // prefilter always used
            flags[n] &= ~HS_FLAG_ALLOWEMPTY; // empty always allowed
            flags[n] &= ~HS_FLAG_SOM_LEFTMOST; // SOM always on
        }

        n++;
    }

    // Our compiler takes an array of plain ol' C strings.
    vector<const char *> patterns(count);
    for (unsigned int i = 0; i < count; i++) {
        patterns[i] = expressions[i].c_str();
    }

    // Compile
    if (!count) { /* slight hack to allow us to compile empty sets cleanly */
        patterns.push_back(nullptr);
        flags.push_back(0);
        idsvec.push_back(0);
    }

    unique_ptr<BaseDB> db;
    if (colliderMode == MODE_HYBRID) {
#ifdef HS_HYBRID
        db = compileHybrid(patterns, flags, idsvec, m_mode, platform, error);
#else
        error = "Hybrid mode not available in this build.";
#endif
    } else {
        db = compileHyperscan(patterns, flags, idsvec, ext, m_mode,
                              platform, error, grey);
    }

    if (!db) {
        return nullptr;
    }

    // Track IDs of patterns that require ordering for validation at match
    // time.
    for (unsigned int i = 0; i < count; i++) {
        bool is_ordered = isOrdered(expressions[i], flags[i]);
        if (check_ordered[i] && !is_ordered) {
            error = "Ordering required, but hs_expression_info suggests "
                    "that ordering is not guaranteed.";
            return nullptr;
        }
        if (is_ordered) {
            db->ordered.insert(idsvec[i]);
        }
    }

    return move(db);
}

bool UltimateTruth::allocScratch(shared_ptr<const BaseDB> db) {
    assert(db);

    // We explicitly avoid running scratch allocators for the same BaseDB
    // over and over again by retaining a shared_ptr to the last one we saw.
    if (db == last_db) {
        return true;
    }

    if (colliderMode == MODE_HYBRID) {
#ifdef HS_HYBRID
        ch_error_t err = ch_alloc_scratch(
            reinterpret_cast<const HybridDB *>(db.get())->db, &chimeraScratch);
        if (err != HS_SUCCESS) {
            return false;
        }
#endif // HS_HYBRID
    } else {
        hs_error_t err = hs_alloc_scratch(
            reinterpret_cast<const HyperscanDB *>(db.get())->db, &scratch);
        if (err != HS_SUCCESS) {
            return false;
        }
    }

    last_db = db;
    return true;
}

bool UltimateTruth::cloneScratch(void) {
    if (colliderMode == MODE_HYBRID) {
#ifdef HS_HYBRID
        ch_scratch_t *old_scratch = chimeraScratch;
        ch_scratch_t *new_scratch;
        ch_error_t ret = ch_clone_scratch(chimeraScratch, &new_scratch);
        if (ret != CH_SUCCESS) {
            DEBUG_PRINTF("failure to clone %d\n", ret);
            return false;
        }
        chimeraScratch = new_scratch;
        ret = ch_free_scratch(old_scratch);
        if (ret != CH_SUCCESS) {
            DEBUG_PRINTF("failure to free %d\n", ret);
            return false;
        }
        DEBUG_PRINTF("hybrid scratch cloned from %p to %p\n",
                     old_scratch, chimeraScratch);
#endif // HS_HYBRID
    } else {
        hs_scratch_t *old_scratch = scratch;
        hs_scratch_t *new_scratch;
        hs_error_t ret = hs_clone_scratch(scratch, &new_scratch);
        if (ret != HS_SUCCESS) {
            DEBUG_PRINTF("failure to clone %d\n", ret);
            return false;
        }
        scratch = new_scratch;
        ret = hs_free_scratch(old_scratch);
        if (ret != HS_SUCCESS) {
            DEBUG_PRINTF("failure to free %d\n", ret);
            return false;
        }
        DEBUG_PRINTF("scratch cloned from %p to %p\n", old_scratch, scratch);
    }
    return true;
}

// Return an appropriately aligned (modulo max align) copy of the given buffer
char * UltimateTruth::setupScanBuffer(const char *begin, size_t len,
                                      size_t align) {
    if (align >= MAX_MAX_UE2_ALIGN) {
        return nullptr;
    }

    // Realloc if necessary
    size_t maxBufSize = len + MAX_MAX_UE2_ALIGN;
    if (maxBufSize > m_scanBuf.size()) {
        m_scanBuf.resize(maxBufSize);
    }

    uintptr_t currentAlign = (uintptr_t)(m_scanBuf.data()) % MAX_MAX_UE2_ALIGN;
    char *ptr;

    ptrdiff_t diff = align - currentAlign;
    if (diff >= 0) {
        ptr = (m_scanBuf.data() + diff);
    } else {
        ptr = (m_scanBuf.data() + (MAX_MAX_UE2_ALIGN + diff));
    }
    assert((uintptr_t)(ptr) % MAX_MAX_UE2_ALIGN == align);

    // copy the buffer
    memcpy(ptr, begin, len);
    return ptr;
}

char *UltimateTruth::setupVecScanBuffer(const char *begin, size_t len,
                                        size_t align, u32 block_id) {
    if (align >= MAX_MAX_UE2_ALIGN) {
        return nullptr;
    }

    assert(block_id < raw_blocks.size());
    vector<char> &raw = raw_blocks[block_id];

    // Realloc if necessary
    size_t maxBufSize = len + MAX_MAX_UE2_ALIGN;
    if (maxBufSize > raw.size()) {
        raw.resize(maxBufSize);
    }
    assert(maxBufSize <= raw.size());

    uintptr_t currentAlign = (uintptr_t)(&raw[0]) % MAX_MAX_UE2_ALIGN;
    char *ptr;

    ptrdiff_t diff = align - currentAlign;
    if (diff >= 0) {
        ptr = (&raw[0] + diff);
    } else {
        ptr = (&raw[0] + (MAX_MAX_UE2_ALIGN + diff));
    }
    assert((uintptr_t)(ptr) % MAX_MAX_UE2_ALIGN == align);

    // copy the buffer
    memcpy(ptr, begin, len);
    return ptr;
}

bool UltimateTruth::saveDatabase(const BaseDB &bdb,
                                 const string &filename) const {
    if (colliderMode == MODE_HYBRID) {
        cerr << "Hybrid mode doesn't support serialization." << endl;
        abort();
    } else {
        return ::saveDatabase(reinterpret_cast<const HyperscanDB *>(&bdb)->db,
                              filename.c_str(), g_verbose);
    }
    return false;
}

shared_ptr<BaseDB>
UltimateTruth::loadDatabase(const string &filename,
                            const std::set<unsigned> &ids) const {
    shared_ptr<BaseDB> db;

    if (colliderMode == MODE_HYBRID) {
        cerr << "Hybrid mode doesn't support deserialization." << endl;
        abort();
    } else {
        hs_database_t *hs_db = ::loadDatabase(filename.c_str(), g_verbose);
        if (!hs_db) {
            return nullptr;
        }

        db = make_shared<HyperscanDB>(hs_db, ids.begin(), ids.end());
    }

    assert(db);

    // Fill db::ordered with the expressions that require the ordered flag.
    for (const auto &id : ids) {
        auto j = m_expr.find(id);
        if (j == m_expr.end()) {
            cerr << "Can't find expression with ID " << id << endl;
            assert(0);
            db.reset();
            return db;
        }
        string expr;
        hs_expr_ext ext;
        unsigned int flags;
        if (!readExpression(j->second, expr, &flags, &ext)) {
            cerr << "Can't parse expression with ID " << id << ": "
                 << j->second << endl;
            assert(0);
            db.reset();
            return db;
        }
        if (isOrdered(expr, flags)) {
            db->ordered.insert(id);
        }
    }

    return db;
}

unsigned int UltimateTruth::describe() const {
    return m_mode;
}

// Hash the settings used to compile a database, returning a string that can be
// used as a filename.
string UltimateTruth::dbSettingsHash(const set<unsigned int> &ids) const {
    // create a single string to contain a description of the db
    ostringstream info_oss;

    // settings from UltimateTruth::describe()
    info_oss << ' ' << describe() << ' ';

    // our set
    for (unsigned int id : ids) {
        info_oss << id << ' ';
    }

    string info = info_oss.str();

    u32 crc = Crc32c_ComputeBuf(0, info.data(), info.size());

    // return STL string with printable version of digest
    ostringstream oss;
    oss << hex << setw(8) << setfill('0') << crc << dec;

    return oss.str();
}

string UltimateTruth::dbFilename(const set<unsigned int> &ids) const {
    ostringstream oss;
    oss << serializePath << '/' << dbSettingsHash(ids) << ".db";
    return oss.str();
}
