/*
 * Copyright (c) 2015-2026, Intel Corporation
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

/** \file
 * \brief Runtime code for hs_database manipulation.
  */

#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "allocator.h"
#include "hs_common.h"
#include "hs_compile.h"
#include "hs_internal.h"
#include "hs_version.h"
#include "ue2common.h"
#include "database.h"
#include <openssl/hmac.h>
#include <openssl/crypto.h>
#include "hs_db_hmac_key.h"
#include "nfa/nfa_internal.h"
#include "nfa/limex_internal.h"
#include "nfa/lbr_internal.h"
#include "nfa/repeat_internal.h"
#include "nfa/mpv_internal.h"
#include "nfa/tamarama_internal.h"
#include "nfa/mcsheng_internal.h"
#include "fdr/fdr.h"
#include "fdr/fdr_internal.h"
#include "hwlm/hwlm_internal.h"
#include "hwlm/noodle_internal.h"
#include "rose/rose_internal.h"
#include "util/compile_error.h"
#include "util/multibit_internal.h"
#include "util/scatter.h"
#include "util/unaligned.h"

static hs_error_t db_check_integrity(const hs_database_t *db);

// Verify HMAC over header fields (magic, version, length, platform)
// excluding padding bytes
static
hs_error_t db_check_header_integrity(const hs_database_t *db) {
    u8 buf[4 + 4 + 4 + 8]; // magic + version + length + platform = 20 bytes
    memcpy(buf, &db->magic, 4);
    memcpy(buf + 4, &db->version, 4);
    memcpy(buf + 8, &db->length, 4);
    memcpy(buf + 12, &db->platform, 8);

    u8 computed[32];
    unsigned int hmac_len = 32;
    if (!HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
              buf, sizeof(buf), computed, &hmac_len) || hmac_len != 32) {
        return HS_INVALID;
    }
    if (CRYPTO_memcmp(computed, db->hmac_hdr, 32) != 0) {
        return HS_INVALID;
    }
    return HS_SUCCESS;
}

static really_inline
int db_correctly_aligned(const void *db) {
    return ISALIGNED_N(db, alignof(unsigned long long));
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_free_database(hs_database_t *db) {
    if (db && unlikely(db->magic != HS_DB_MAGIC)) {
        return HS_INVALID;
    }
    if (db) {
        // Verify header HMAC to authenticate db->length before trusting it.
        if (unlikely(db_check_header_integrity(db) != HS_SUCCESS)) {
            return HS_INVALID;
        }
        size_t db_len = sizeof(struct hs_database) + db->length;
        hs_db_unprotect(db, db_len);
        hs_db_free(db, db_len);
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialize_database(const hs_database_t *db, char **bytes,
                                          size_t *serialized_length) {
    if (unlikely(!db || !bytes || !serialized_length)) {
        return HS_INVALID;
    }

    if (unlikely(!db_correctly_aligned(db))) {
        return HS_BAD_ALIGN;
    }

    hs_error_t ret = validDatabase(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Verify header HMAC to authenticate db->length before trusting it.
    ret = db_check_header_integrity(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Verify bytecode HMAC after confirming db->length is authentic.
    ret = db_check_integrity(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    size_t length = sizeof(struct hs_database) + db->length;

    char *out = hs_misc_alloc(length);
    ret = hs_check_alloc(out);
    if (unlikely(ret != HS_SUCCESS)) {
        hs_misc_free(out);
        return ret;
    }

    memset(out, 0, length);

    u32 *buf = (u32 *)out;
    *buf = db->magic;
    buf++;
    *buf = db->version;
    buf++;
    *buf = db->length;
    buf++;
    memcpy(buf, &db->platform, sizeof(u64a));
    buf += 2;
    memcpy(buf, db->hmac, 32);
    buf += (32 / sizeof(u32));
    memcpy(buf, db->hmac_hdr, 32);
    buf += (32 / sizeof(u32));

    const char *bytecode = hs_get_bytecode(db);
    memcpy(buf, bytecode, db->length);

    *bytes = out;
    *serialized_length = length;
    return HS_SUCCESS;
}

// check that the database header's platform is compatible with the current
// runtime platform.
static
hs_error_t db_check_platform(const u64a p) {
    if (p == HS_PLATFORM_ALL) {
        return HS_SUCCESS;
    }

    if (unlikely(p != hs_current_platform
        && p != (hs_current_platform | hs_current_platform_no_avx2)
        && p != (hs_current_platform | hs_current_platform_no_avx512)
        && p != (hs_current_platform | hs_current_platform_no_avx512vbmi))) {
        return HS_DB_PLATFORM_ERROR;
    }
    // passed all checks
    return HS_SUCCESS;
}

// Decode and check the database header, returning appropriate errors or
// HS_SUCCESS if it's OK. The header should be allocated on the stack
// and later copied into the deserialized database.
static
hs_error_t db_decode_header(const char **bytes, const size_t length,
                            struct hs_database *header) {
    if (unlikely(!*bytes)) {
        return HS_INVALID;
    }

    // Check minimum 12 bytes (magic + version + length) so that legacy
    // databases reach the version check and get HS_DB_VERSION_ERROR.
    if (unlikely(length < 3 * sizeof(u32))) {
        return HS_INVALID;
    }

    // There's no requirement, really, that the serialized stream of bytes
    // we've been given is 4-byte aligned, so we use unaligned loads here.

    const u32 *buf = (const u32 *)*bytes;

    // Zero header so that none of it (e.g. its padding) is uninitialized.
    memset(header, 0, sizeof(struct hs_database));

    header->magic = unaligned_load_u32(buf++);
    if (unlikely(header->magic != HS_DB_MAGIC)) {
        return HS_INVALID;
    }

    header->version = unaligned_load_u32(buf++);
    if (unlikely(header->version != HS_DB_VERSION)) {
        return HS_DB_VERSION_ERROR;
    }

    // Now that magic and version are confirmed, require full header size.
    if (unlikely(length < sizeof(struct hs_database))) {
        return HS_INVALID;
    }

    header->length = unaligned_load_u32(buf++);
    if (unlikely(length != sizeof(struct hs_database) + header->length)) {
        DEBUG_PRINTF("bad length %zu, expecting %zu\n", length,
                     sizeof(struct hs_database) + header->length);
        return HS_INVALID;
    }

    header->platform = unaligned_load_u64a(buf);
    buf += 2;
    memcpy(header->hmac, buf, 32);
    buf += (32 / sizeof(u32));
    memcpy(header->hmac_hdr, buf, 32);
    buf += (32 / sizeof(u32));

    *bytes = (const char *)buf;

    return HS_SUCCESS; // Header checks out
}

// Check the integrity of a database using HMAC-SHA256
static
hs_error_t db_check_integrity(const hs_database_t *db) {
    const char *bytecode = hs_get_bytecode(db);
    u8 computed[32];
    unsigned int hmac_len = 32;
    if (!HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
             (const unsigned char *)bytecode, db->length, computed,
             &hmac_len) || hmac_len != 32) {
        DEBUG_PRINTF("hmac computation failed!\n");
        return HS_INVALID;
    }
    if (unlikely(CRYPTO_memcmp(computed, db->hmac, 32) != 0)) {
        DEBUG_PRINTF("hmac mismatch!\n");
        return HS_INVALID;
    }
    return HS_SUCCESS;
}

/**
 * \brief Validate mcsheng (8-bit and 16-bit) NFA successor table entries.
 *
 * Each entry in the successor table is a next-state value used as an index
 * into the aux table via get_aux(m, s). A forged value >= state_count causes
 * an out-of-bounds read (CWE-125). We also check that start states are
 * in-range and that the aux table fits within the NFA image.
 *
 * Crucially, we scan the FULL actual transition table (derived from aux_offset
 * minus the end of the mcsheng struct), not just entries for declared states,
 * because the runtime can reach entries for start states that are themselves
 * outside the declared state_count range.
 */

static
hs_error_t db_validate_mcsheng_succ_table(const struct RoseEngine *rose,
                                          u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > rose_base + rose_size)) {
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        if (nfa->type != MCSHENG_NFA_8 && nfa->type != MCSHENG_NFA_16) {
            continue;
        }

        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            DEBUG_PRINTF("mcsheng[%u] NFA body out of bounds\n", qi);
            return HS_INVALID;
        }

        if (unlikely(nfa->length <= sizeof(struct NFA) + sizeof(struct mcsheng))) {
            DEBUG_PRINTF("mcsheng[%u] NFA too small for mcsheng header\n", qi);
            return HS_INVALID;
        }

        const struct mcsheng *m =
            (const struct mcsheng *)getImplNfa(nfa);
        u32 nfa_len = nfa->length;
        u32 state_count = m->state_count;

        if (unlikely(state_count == 0)) {
            DEBUG_PRINTF("mcsheng[%u] state_count is zero\n", qi);
            return HS_INVALID;
        }

        /* Reject if start states are out of bounds. */
        if (unlikely(m->start_anchored >= state_count ||
                     m->start_floating >= state_count)) {
            DEBUG_PRINTF("mcsheng[%u] start state out of bounds\n", qi);
            return HS_INVALID;
        }

        /* Validate aux table fits within the NFA image.
         * aux_offset is relative to the start of the NFA header. */
        u32 aux_offset = m->aux_offset;
        u64a aux_end = (u64a)aux_offset +
                       (u64a)state_count * sizeof(struct mstate_aux);
        if (unlikely(aux_offset < sizeof(struct NFA) + sizeof(struct mcsheng) ||
                     aux_end > nfa_len)) {
            DEBUG_PRINTF("mcsheng[%u] aux table out of bounds\n", qi);
            return HS_INVALID;
        }

        /* Validate sherman offset bounds if present. */
        if (m->sherman_offset) {
            if (unlikely(m->sherman_offset < sizeof(struct NFA) +
                             sizeof(struct mcsheng) ||
                         m->sherman_offset > nfa_len)) {
                DEBUG_PRINTF("mcsheng[%u] sherman_offset out of bounds\n", qi);
                return HS_INVALID;
            }
        }
        if (m->sherman_end) {
            if (unlikely(m->sherman_end > nfa_len)) {
                DEBUG_PRINTF("mcsheng[%u] sherman_end out of bounds\n", qi);
                return HS_INVALID;
            }
        }

        /* Validate successor transition table entries.
         * The runtime indexes succ_table[(s << as) + cprime] and uses the
         * result (masked with STATE_MASK) as a state index into aux/state
         * structures. Forged entries can drive out-of-bounds reads. */
        u32 as = m->alphaShift;

        /* alphaShift must be <= 8 (alphabet size at most 256). */
        if (unlikely(as > 8)) {
            DEBUG_PRINTF("mcsheng[%u] alphaShift %u > 8\n", qi, as);
            return HS_INVALID;
        }

        u32 sheng_end = m->sheng_end;
        u32 alphabet_size = 1U << as;

        if (unlikely(sheng_end > state_count)) {
            DEBUG_PRINTF("mcsheng[%u] sheng_end > state_count\n", qi);
            return HS_INVALID;
        }

        /* sherman_limit must not exceed state_count. */
        if (unlikely(m->sherman_limit > state_count)) {
            DEBUG_PRINTF("mcsheng[%u] sherman_limit %u > state_count %u\n",
                         qi, m->sherman_limit, state_count);
            return HS_INVALID;
        }

        /* succ_table starts right after the mcsheng header. It covers states
         * [sheng_end, sherman_limit) with alphabet_size entries each. */
        u32 succ_table_offset = (u32)sizeof(struct NFA) +
                                (u32)sizeof(struct mcsheng);
        u32 normal_state_count = m->sherman_limit > sheng_end
                                     ? m->sherman_limit - sheng_end
                                     : 0;

        if (nfa->type == MCSHENG_NFA_16) {
            u64a succ_table_size =
                (u64a)normal_state_count * alphabet_size * sizeof(u16);
            if (unlikely((u64a)succ_table_offset + succ_table_size > nfa_len)) {
                DEBUG_PRINTF("mcsheng[%u] succ_table(16) out of bounds\n", qi);
                return HS_INVALID;
            }
            const u16 *succ_table =
                (const u16 *)((const char *)nfa + succ_table_offset);
            u64a entry_count = (u64a)normal_state_count * alphabet_size;
            for (u64a i = 0; i < entry_count; i++) {
                u16 s = succ_table[i] & STATE_MASK;
                if (unlikely(s >= state_count)) {
                    DEBUG_PRINTF("mcsheng[%u] succ_table16 entry %llu "
                                 "state %u >= state_count %u\n",
                                 qi, (unsigned long long)i, s, state_count);
                    return HS_INVALID;
                }
            }
        } else {
            /* MCSHENG_NFA_8 */
            u64a succ_table_size =
                (u64a)normal_state_count * alphabet_size * sizeof(u8);
            if (unlikely((u64a)succ_table_offset + succ_table_size > nfa_len)) {
                DEBUG_PRINTF("mcsheng[%u] succ_table(8) out of bounds\n", qi);
                return HS_INVALID;
            }
            const u8 *succ_table =
                (const u8 *)((const char *)nfa + succ_table_offset);
            u64a entry_count = (u64a)normal_state_count * alphabet_size;
            for (u64a i = 0; i < entry_count; i++) {
                u8 s = succ_table[i];
                if (unlikely(s >= state_count)) {
                    DEBUG_PRINTF("mcsheng[%u] succ_table8 entry %llu "
                                 "state %u >= state_count %u\n",
                                 qi, (unsigned long long)i, (u32)s,
                                 state_count);
                    return HS_INVALID;
                }
            }
        }
    }

    return HS_SUCCESS;
}
	

/**
 * \brief Validate LBR NFA repeatInfoOffset fields.
 *
 * All LBR NFA types (Dot, Verm, NVerm, Shuf, VShuf, Truf) share a
 * lbr_common header whose repeatInfoOffset field is used by getRepeatInfo()
 * to derive a RepeatInfo pointer relative to the start of the lbr_common
 * structure. A forged offset can move this pointer outside the NFA image,
 * causing an out-of-bounds read in clearRepeat() / repeatPack() /
 * repeatUnpack() during hs_scan().
 *
 * This function iterates every NFA engine in the RoseEngine and, for each
 * LBR variant, validates that repeatInfoOffset + sizeof(RepeatInfo) stays
 * within the NFA body.
 */
static
hs_error_t db_validate_lbr_repeat_info(const struct RoseEngine *rose,
                                       u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines */
    }

    const char *base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > base + rose_size)) {
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }
        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            return HS_INVALID;
        }

        const struct NFA *nfa = (const struct NFA *)(base + ni->nfaOffset);

        if (!isLbrType(nfa->type)) {
            continue;
        }

        /* Whole NFA blob must fit inside rose. */
        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            DEBUG_PRINTF("LBR NFA[%u] body out of bounds\n", qi);
            return HS_INVALID;
        }

        if (unlikely(nfa->length < sizeof(struct NFA) + sizeof(struct lbr_common))) {
            DEBUG_PRINTF("LBR NFA[%u] too small for lbr_common\n", qi);
            return HS_INVALID;
        }

        u32 nfa_body_size = nfa->length - (u32)sizeof(struct NFA);

        const struct lbr_common *lbr =
            (const struct lbr_common *)(base + ni->nfaOffset +
                                        sizeof(struct NFA));

        if (unlikely(lbr->repeatInfoOffset < sizeof(struct lbr_common))) {
            DEBUG_PRINTF("LBR NFA[%u] repeatInfoOffset too small: offset=%u\n",
                         qi, lbr->repeatInfoOffset);
            return HS_INVALID;
        }

        /* repeatInfoOffset is relative to lbr_common start. Validate that
         * the derived RepeatInfo fits within the NFA body. */
        u64a repeat_end = (u64a)lbr->repeatInfoOffset +
                          (u64a)sizeof(struct RepeatInfo);
        if (unlikely(repeat_end > nfa_body_size)) {
            DEBUG_PRINTF("LBR NFA[%u] repeatInfoOffset OOB: offset=%u "
                         "repeat_end=%llu nfa_body=%u\n",
                         qi, lbr->repeatInfoOffset,
                         (unsigned long long)repeat_end, nfa_body_size);
            return HS_INVALID;
        }
    }

    return HS_SUCCESS;
}

/**
 * \brief Validate Noodle matcher table fields.
 *
 * The Noodle literal matcher computes buffer load addresses using:
 *   buf + pos + key_offset - msk_len
 * If key_offset or msk_len are forged, this can underflow the buffer pointer,
 * causing an out-of-bounds read. Additionally, msk_len must be in [1,8] since
 * msk/cmp are u64a (8 bytes).
 *
 * This function iterates every HWLM matcher offset in the RoseEngine and,
 * for each Noodle engine, validates the noodTable fields.
 */
static
hs_error_t db_validate_noodle_table(const struct RoseEngine *rose,
                                    u32 rose_size) {
    const char *base = (const char *)rose;

    const u32 offsets[] = {
        rose->fmatcherOffset,
        rose->ematcherOffset,
        rose->amatcherOffset,
        rose->sbmatcherOffset,
        rose->drmatcherOffset,
    };

    for (u32 k = 0; k < ARRAY_LENGTH(offsets); k++) {
        u32 off = offsets[k];
        if (!off) {
            continue;
        }
        if (unlikely(off >= rose_size ||
                     off + sizeof(struct HWLM) > rose_size)) {
            return HS_INVALID;
        }

        const struct HWLM *hwlm = (const struct HWLM *)(base + off);

        if (hwlm->type != HWLM_ENGINE_NOOD) {
            continue;
        }

        /* noodTable follows the HWLM header, cache-line aligned. */
        u32 nood_rel = (u32)ROUNDUP_CL(sizeof(struct HWLM));
        if (unlikely((u64a)off + nood_rel + sizeof(struct noodTable) >
                     rose_size)) {
            DEBUG_PRINTF("noodTable out of bounds at offset %u\n", off);
            return HS_INVALID;
        }

        const struct noodTable *nood =
            (const struct noodTable *)(base + off + nood_rel);

        /* msk_len must be in [1, 8] */
        if (unlikely(nood->msk_len == 0 || nood->msk_len > 8)) {
            DEBUG_PRINTF("noodTable msk_len invalid: %u\n", nood->msk_len);
            return HS_INVALID;
        }

        if (nood->single) {
            /* Single-byte key fragment implies key_offset must be 1. */
            if (unlikely(nood->key_offset != 1)) {
                DEBUG_PRINTF("noodTable key_offset invalid for single: %u\n",
                             nood->key_offset);
                return HS_INVALID;
            }
        } else {
            /* Two-byte key fragment: key_offset must be >=2 and <= msk_len. */
            if (unlikely(nood->msk_len < 2 || nood->key_offset < 2 ||
                         nood->key_offset > nood->msk_len)) {
                DEBUG_PRINTF("noodTable key_offset invalid: %u (msk_len=%u)\n",
                             nood->key_offset, nood->msk_len);
                return HS_INVALID;
            }
        }
    }

    return HS_SUCCESS;
}

/**
 * \brief Validate FDR engineID fields in HWLM matchers (CWE-125).
 *
 * fdrExec() uses fdr->engineID as an index into a static funcs[] dispatch
 * table of FDR_ENGINE_COUNT entries. A forged value >= FDR_ENGINE_COUNT
 * causes a global-buffer-overflow. Reject any database containing such a
 * value before hs_scan() can reach fdrExec().
 *
 * We check every HWLM matcher reachable from the RoseEngine (fmatcherOffset,
 * ematcherOffset, amatcherOffset, sbmatcherOffset, drmatcherOffset).
 */
static
hs_error_t db_validate_fdr_engine_id(const struct RoseEngine *rose,
                                     u32 rose_size) {
    const char *base = (const char *)rose;

    /* All matcher offset fields we need to check */
    const u32 offsets[] = {
        rose->fmatcherOffset,
        rose->ematcherOffset,
        rose->amatcherOffset,
        rose->sbmatcherOffset,
        rose->drmatcherOffset,
    };

    for (u32 k = 0; k < ARRAY_LENGTH(offsets); k++) {
        u32 off = offsets[k];
        if (!off) {
            continue;
        }

        if (unlikely(off >= rose_size ||
                     off + sizeof(struct HWLM) > rose_size)) {
            DEBUG_PRINTF("HWLM matcher offset %u out of bounds\n", off);
            return HS_INVALID;
        }

        const struct HWLM *hwlm = (const struct HWLM *)(base + off);

        if (hwlm->type != HWLM_ENGINE_FDR) {
            continue; /* only FDR engines have an engineID */
        }

        /* FDR struct immediately follows the HWLM header (cache-line aligned).
         * Use the same layout formula as HWLM_C_DATA(). */
        u32 fdr_rel = (u32)ROUNDUP_CL(sizeof(struct HWLM));
        if (unlikely((u64a)off + fdr_rel + sizeof(struct FDR) > rose_size)) {
            DEBUG_PRINTF("FDR struct out of bounds at offset %u\n", off);
            return HS_INVALID;
        }

        const struct FDR *fdr = (const struct FDR *)HWLM_C_DATA(hwlm);

        if (unlikely(fdr->engineID >= FDR_ENGINE_COUNT)) {
            DEBUG_PRINTF("FDR engineID %u >= FDR_ENGINE_COUNT %u\n",
                         fdr->engineID, FDR_ENGINE_COUNT);
            return HS_INVALID;
        }
    }

    return HS_SUCCESS;
}


/**
 * \brief Validate Tamarama subengine offsets to prevent out-of-bounds access
 * (CWE-125). A forged child-offset can cause getSubEngine() to return a wild
 * pointer that is then dereferenced by nfaQueueInitState().
 */
static
hs_error_t db_validate_tamarama_offsets(const struct RoseEngine *rose,
                                        u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines - nothing to check */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > rose_base + rose_size)) {
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        if (nfa->type != TAMARAMA_NFA) {
            continue;
        }

        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            return HS_INVALID;
        }
        
        if (unlikely(nfa->length <= sizeof(struct NFA))) {
            return HS_INVALID;
        }

        const char *tama_base = (const char *)getImplNfa(nfa);
        u32 tama_len = nfa->length - (u32)sizeof(struct NFA);

        if (unlikely(tama_len < sizeof(struct Tamarama))) {
            DEBUG_PRINTF("Tamarama[%u] too small for header\n", qi);
            return HS_INVALID;
        }

        const struct Tamarama *t = (const struct Tamarama *)tama_base;
        u32 numSub = t->numSubEngines;

        if (unlikely(numSub == 0)) {
            DEBUG_PRINTF("Tamarama[%u] has zero subengines\n", qi);
            return HS_INVALID;
        }

        /* Explicitly bound numSub to what can fit in tama_len before using it
         * as a loop bound (addresses TAINTED_SCALAR: CID 2540142). */
        if (unlikely(numSub > tama_len / sizeof(u32))) {
            DEBUG_PRINTF("Tamarama[%u] numSubEngines %u exceeds tama_len\n",
                         qi, numSub);
            return HS_INVALID;
        }

        /* The layout after the Tamarama header is:
         *   u32 baseTops[numSub]   — top values
         *   u32 subOffsets[numSub] — offsets to child NFAs
         * Both arrays must fit within tama_len. */
        u64a table_end = (u64a)sizeof(struct Tamarama) +
                         (u64a)numSub * 2 * sizeof(u32);
        if (unlikely(table_end > tama_len)) {
            DEBUG_PRINTF("Tamarama[%u] offset table overflows\n", qi);
            return HS_INVALID;
        }

        /* subOffsets array starts after baseTops */
        const u32 *subOffsets =
            (const u32 *)(tama_base + sizeof(struct Tamarama) +
                          numSub * sizeof(u32));

        for (u32 i = 0; i < numSub; i++) {
            u32 child_off = subOffsets[i];

            if (unlikely(child_off >= tama_len ||
                         child_off + sizeof(struct NFA) > tama_len)) {
                DEBUG_PRINTF("Tamarama[%u] sub[%u] offset %u out of bounds "
                             "(tama_len=%u)\n", qi, i, child_off, tama_len);
                return HS_INVALID;
            }

	 /* Verify child NFA body fits within the Tamarama image */
            const struct NFA *child =
                (const struct NFA *)(tama_base + child_off);
            if (unlikely(child_off + child->length > tama_len)) {
                DEBUG_PRINTF("Tamarama[%u] sub[%u] child body overflows\n",
                             qi, i);
                return HS_INVALID;
            }
        }
    }

    return HS_SUCCESS;
}



/**
 * \brief Validate that all LimEx reachMap[] entries are within the bounds of
 * the reach table (CWE-125). A forged reachMap entry could index past the end
 * of the reach table during scan-time execution.
 */
static
hs_error_t db_validate_limex_reach_map(const struct RoseEngine *rose,
                                       u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines - nothing to check */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > rose_base + rose_size)) {
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        if (!isNfaType(nfa->type)) {
            continue;
        }

        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            return HS_INVALID;
        }

        if (unlikely(nfa->length <= sizeof(struct NFA))) {
            return HS_INVALID;
        }

	 const char *limex_base = (const char *)getImplNfa(nfa);
        u32 limex_len = nfa->length - (u32)sizeof(struct NFA);

        /* Ensure we can read reachMap[256] and reachSize. */
        const u32 reach_prefix_end =
            (u32)(N_CHARS + sizeof(u32)); /* reachMap + reachSize */
        if (unlikely(limex_len < reach_prefix_end)) {
            DEBUG_PRINTF("LimEx[%u] too small for reachMap\n", qi);
            return HS_INVALID;
        }

        const struct LimExNFA32 *limex =
            (const struct LimExNFA32 *)limex_base;
        const u32 reachSize = limex->reachSize;

        if (unlikely(reachSize == 0)) {
            DEBUG_PRINTF("LimEx[%u] reachSize is zero\n", qi);
            return HS_INVALID;
        }

        /* Validate every reachMap entry is within bounds. */
        for (u32 i = 0; i < N_CHARS; i++) {
            if (unlikely(limex->reachMap[i] >= reachSize)) {
                DEBUG_PRINTF("LimEx[%u] reachMap[%u]=%u >= reachSize=%u\n",
                             qi, i, limex->reachMap[i], reachSize);
                return HS_INVALID;
            }
        }
    }

    return HS_SUCCESS;
}

/**
 * Validate LimEx NFA repeat metadata to prevent out-of-bounds access.
 *
 * A forged serialized database can craft repeatOffset tables and
 * NFARepeatInfo records so that packedCtrlOffset / stateOffset point
 * outside the per-engine stream-state allocation.  During streaming
 * scans the LimEx expansion and compression use these offsets directly,
 * turning normal scan traffic into OOB reads and writes (CWE-787).
 *
 * This function iterates every NFA engine embedded in the RoseEngine
 * and, for each LimEx variant, validates:
 *   1. The repeat-offset lookup table fits inside the engine blob.
 *   2. Every NFARepeatInfo record fits inside the engine blob.
 *   3. packedCtrlOffset + stateSize stays within nfa->streamStateSize.
 *   4. stateOffset stays within nfa->streamStateSize.
 *   5. ctrlIndex < repeatCount (scratch-space RepeatControl bound).
 *   6. tugMaskOffset stays within the engine blob.
 */

static
hs_error_t db_validate_limex_repeats(const struct RoseEngine *rose,
                                     u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines - nothing to check */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > rose_base + rose_size)) {
            DEBUG_PRINTF("NfaInfo[%u] out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            DEBUG_PRINTF("NFA[%u] header out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        /* Validate NfaInfo.stateOffset and fullStateOffset BEFORE
         * the isNfaType gate so it applies to every queue engine type.
         * initOutfixQueue() uses these as:
         *   q->streamState = state + info->stateOffset
         *   q->state = scratch->fullState + info->fullStateOffset
         * If forged, they cause OOB writes during NFA state init/store. */
        {
            u64a stream_end = (u64a)ni->stateOffset +
                              (u64a)nfa->streamStateSize;
            if (unlikely(stream_end > rose->stateOffsets.end)) {
                DEBUG_PRINTF("NfaInfo[%u] stateOffset+streamStateSize exceeds "
                             "stateOffsets.end: %llu > %u\n", qi,
                             stream_end, rose->stateOffsets.end);
                return HS_INVALID;
            }
        }
        {
            u64a scratch_end = (u64a)ni->fullStateOffset +
                               (u64a)nfa->scratchStateSize;
            if (unlikely(scratch_end > rose->scratchStateSize)) {
                DEBUG_PRINTF("NfaInfo[%u] fullStateOffset+scratchStateSize "
                             "exceeds scratchStateSize: %llu > %u\n", qi,
                             scratch_end, rose->scratchStateSize);
                return HS_INVALID;
            }
        }

        /* Only LimEx NFAs carry repeat metadata. */
        if (!isNfaType(nfa->type)) {
            continue;
        }

        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            DEBUG_PRINTF("NFA[%u] body out of bounds\n", qi);
            return HS_INVALID;
        }

        if (unlikely(nfa->length <= sizeof(struct NFA))) {
            DEBUG_PRINTF("NFA[%u] impossibly small\n", qi);
            return HS_INVALID;
        }

        const char *limex_base = (const char *)getImplNfa(nfa);
        u32 limex_len = nfa->length - (u32)sizeof(struct NFA);

        /*
         * All LimExNFA variants (32/64/128/256/384/512) share an identical
         * prefix of u32 fields up to and including 'flags'.  The fields we
         * need - repeatCount, repeatOffset, stateSize - are in that common
         * prefix, so casting to LimExNFA32 is safe.
         */
        const u32 prefix_end =
            (u32)(offsetof(struct LimExNFA32, flags) + sizeof(u32));
        if (unlikely(limex_len < prefix_end)) {
            DEBUG_PRINTF("LimEx[%u] smaller than common prefix\n", qi);
            return HS_INVALID;
        }

        const struct LimExNFA32 *limex =
            (const struct LimExNFA32 *)limex_base;
        const u32 repeatCount = limex->repeatCount;
        const u32 rep_table_off = limex->repeatOffset;
        const u32 nfa_state_size = limex->stateSize;

        if (repeatCount == 0) {
            continue;
        }

        /* Upper-bound repeatCount to sanitise tainted loop boundary. */
        if (unlikely(repeatCount > limex_len / sizeof(u32))) {
            DEBUG_PRINTF("LimEx[%u] repeatCount too large: %u\n",
                         qi, repeatCount);
            return HS_INVALID;
        }

        /* 1. Repeat-offset lookup table must fit inside the engine blob. */
        if (unlikely(rep_table_off >= limex_len)) {
            DEBUG_PRINTF("LimEx[%u] repeatOffset out of bounds\n", qi);
            return HS_INVALID;
        }

        u64a table_end =
            (u64a)rep_table_off + (u64a)repeatCount * sizeof(u32);
        if (unlikely(table_end > limex_len)) {
            DEBUG_PRINTF("LimEx[%u] repeat offset table overflows\n", qi);
            return HS_INVALID;
        }

        const u32 *rep_offsets =
            (const u32 *)(limex_base + rep_table_off);

        for (u32 i = 0; i < repeatCount; i++) {
            u32 info_off = rep_offsets[i];

            /* 2. NFARepeatInfo must fit inside the engine blob. */
            if (unlikely(info_off >= limex_len ||
                         info_off + sizeof(struct NFARepeatInfo) > limex_len)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] info out of bounds\n",
                             qi, i);
                return HS_INVALID;
            }

            const struct NFARepeatInfo *info =
                (const struct NFARepeatInfo *)(limex_base + info_off);

            /* 3. packedCtrlOffset + stateSize within stream state.
             *    Stream-state layout:
             *      [0 .. nfa_state_size)           NFA state bitvector
             *      [nfa_state_size .. streamState)  repeat packed state
             *    repeatPack/Unpack use:  (src + nfa_state_size) + packedCtrlOffset
             */
            u64a ctrl_end = (u64a)nfa_state_size +
                            (u64a)info->packedCtrlOffset +
                            (u64a)info->stateSize;
            if (unlikely(ctrl_end > nfa->streamStateSize)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] packedCtrlOffset+stateSize "
                             "exceeds streamStateSize\n", qi, i);
                return HS_INVALID;
            }

            /* 4. stateOffset within stream state. */
            u64a state_end = (u64a)nfa_state_size +
                             (u64a)info->stateOffset;
            if (unlikely(state_end > nfa->streamStateSize)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] stateOffset exceeds "
                             "streamStateSize\n", qi, i);
                return HS_INVALID;
            }

            /* 5. ctrlIndex must be within repeatCount (scratch bound). */
            if (unlikely(info->ctrlIndex >= repeatCount)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] ctrlIndex out of range\n",
                             qi, i);
                return HS_INVALID;
            }

            /* 6. tugMaskOffset must stay inside the engine blob. */
            u64a tug_end = (u64a)info_off + (u64a)info->tugMaskOffset;
            if (unlikely(tug_end >= limex_len)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] tugMaskOffset "
                             "out of bounds\n", qi, i);
                return HS_INVALID;
            }
        }
    }

    return HS_SUCCESS;
}

/**
 * Validate RoseStateOffsets layout (CWE-122, CWE-190).
 *
 * This function mirrors fillStateOffsets() in rose_build_bytecode.cpp.
 * It re-derives and validates every stateOffsets field against the counts
 * stored in the RoseEngine, using 64-bit arithmetic to detect integer
 * overflows, and ensures every region fits within stateOffsets.end.
 *
 * Called from db_validate_rose_offsets() so it covers block, streaming,
 * and vectored modes uniformly.
 */
hs_error_t validateStateLayout(const struct RoseEngine *rose) {
    const struct RoseStateOffsets *so = &rose->stateOffsets;
    const u32 end = so->end;

    /* end must be at least 1 byte (status byte at offset 0). */
    if (unlikely(end < sizeof(u8))) {
        DEBUG_PRINTF("stateOffsets.end too small: %u\n", end);
        return HS_INVALID;
    }

    /* Upper-bound counts to prevent absurd mmbit_size computations.
     * Use u64a to prevent 32-bit overflow of end * 8u for large forged end. */
    const u64a end_bits = (u64a)end * 8u;
    if (unlikely((u64a)rose->activeArrayCount > end_bits)) {
        DEBUG_PRINTF("activeArrayCount unreasonable: %u\n",
                     rose->activeArrayCount);
        return HS_INVALID;
    }
    if (unlikely((u64a)rose->activeLeftCount > end_bits)) {
        DEBUG_PRINTF("activeLeftCount unreasonable: %u\n",
                     rose->activeLeftCount);
        return HS_INVALID;
    }
    if (unlikely((u64a)rose->ekeyCount > end_bits)) {
        DEBUG_PRINTF("ekeyCount unreasonable: %u\n", rose->ekeyCount);
        return HS_INVALID;
    }

    /* Validate activeLeafArray: offset + size <= end, size == mmbit_size(count) */
    {
        u32 expected_size = rt_mmbit_size(rose->activeArrayCount);
        if (unlikely(so->activeLeafArray_size != expected_size)) {
            DEBUG_PRINTF("activeLeafArray_size mismatch: %u != %u (count=%u)\n",
                         so->activeLeafArray_size, expected_size,
                         rose->activeArrayCount);
            return HS_INVALID;
        }
        if (unlikely(so->activeLeafArray > end ||
                     so->activeLeafArray_size > end - so->activeLeafArray)) {
            DEBUG_PRINTF("activeLeafArray region OOB: off=%u size=%u end=%u\n",
                         so->activeLeafArray, so->activeLeafArray_size, end);
            return HS_INVALID;
        }
    }

    /* Validate activeLeftArray: offset + size <= end, size == mmbit_size(count) */
    {
        u32 expected_size = rt_mmbit_size(rose->activeLeftCount);
        if (unlikely(so->activeLeftArray_size != expected_size)) {
            DEBUG_PRINTF("activeLeftArray_size mismatch: %u != %u (count=%u)\n",
                         so->activeLeftArray_size, expected_size,
                         rose->activeLeftCount);
            return HS_INVALID;
        }
        if (unlikely(so->activeLeftArray > end ||
                     so->activeLeftArray_size > end - so->activeLeftArray)) {
            DEBUG_PRINTF("activeLeftArray region OOB: off=%u size=%u end=%u\n",
                         so->activeLeftArray, so->activeLeftArray_size, end);
            return HS_INVALID;
        }
    }

    /* Validate longLitState: offset + size <= end.
     * NOTE: rose->longLitStreamState is not populated by the compiler
     * (fillStateOffsets uses a local longLitStreamStateRequired parameter
     * to set so->longLitState_size directly), so we cannot cross-check
     * against it. Only verify bounds. */
    {
        if (unlikely(so->longLitState > end ||
                     so->longLitState_size > end - so->longLitState)) {
            DEBUG_PRINTF("longLitState region OOB: off=%u size=%u end=%u\n",
                         so->longLitState, so->longLitState_size, end);
            return HS_INVALID;
        }
    }

    /* Validate leftfixLagTable: offset <= end (no stored size, bounded by
     * activeLeftCount which is an upper bound on lagged roses). */
    if (unlikely(so->leftfixLagTable > end)) {
        DEBUG_PRINTF("leftfixLagTable OOB: %u > %u\n", so->leftfixLagTable, end);
        return HS_INVALID;
    }

    /* Validate anchorState: offset + anchorStateSize <= end */
    if (unlikely(so->anchorState > end ||
                 rose->anchorStateSize > end - so->anchorState)) {
        DEBUG_PRINTF("anchorState region OOB: off=%u size=%u end=%u\n",
                     so->anchorState, rose->anchorStateSize, end);
        return HS_INVALID;
    }

    /* Validate groups: offset + groups_size <= end, groups_size <= 8 */
    if (unlikely(so->groups_size > sizeof(u64a))) {
        DEBUG_PRINTF("groups_size too large: %u > %zu\n",
                     so->groups_size, sizeof(u64a));
        return HS_INVALID;
    }
    if (unlikely(so->groups > end ||
                 so->groups_size > end - so->groups)) {
        DEBUG_PRINTF("groups region OOB: off=%u size=%u end=%u\n",
                     so->groups, so->groups_size, end);
        return HS_INVALID;
    }

    /* Validate history: offset + historyRequired <= end */
    if (unlikely(so->history > end ||
                 rose->historyRequired > end - so->history)) {
        DEBUG_PRINTF("history region OOB: off=%u size=%u end=%u\n",
                     so->history, rose->historyRequired, end);
        return HS_INVALID;
    }

    /* Validate exhausted: offset + size <= end, size == mmbit_size(ekeyCount) */
    {
        u32 expected_size = rt_mmbit_size(rose->ekeyCount);
        if (unlikely(so->exhausted_size != expected_size)) {
            DEBUG_PRINTF("exhausted_size mismatch: %u != %u (count=%u)\n",
                         so->exhausted_size, expected_size, rose->ekeyCount);
            return HS_INVALID;
        }
        if (unlikely(so->exhausted > end ||
                     so->exhausted_size > end - so->exhausted)) {
            DEBUG_PRINTF("exhausted region OOB: off=%u size=%u end=%u\n",
                         so->exhausted, so->exhausted_size, end);
            return HS_INVALID;
        }
    }

    /* Validate logicalVec: offset + size <= end, size == mmbit_size(lkeyCount+lopCount) */
    {
        /* Overflow-safe addition check for lkeyCount + lopCount. */
        if (unlikely(rose->lkeyCount > MMB_MAX_BITS ||
                     rose->lopCount > MMB_MAX_BITS - rose->lkeyCount)) {
            DEBUG_PRINTF("logical key count overflow: lkey=%u lop=%u\n",
                         rose->lkeyCount, rose->lopCount);
            return HS_INVALID;
        }
        u32 logicalCount = rose->lkeyCount + rose->lopCount;
        u32 expected_size = rt_mmbit_size(logicalCount);
        if (unlikely(so->logicalVec_size != expected_size)) {
            DEBUG_PRINTF("logicalVec_size mismatch: %u != %u (count=%u)\n",
                         so->logicalVec_size, expected_size, logicalCount);
            return HS_INVALID;
        }
        if (unlikely(so->logicalVec > end ||
                     so->logicalVec_size > end - so->logicalVec)) {
            DEBUG_PRINTF("logicalVec region OOB: off=%u size=%u end=%u\n",
                         so->logicalVec, so->logicalVec_size, end);
            return HS_INVALID;
        }
    }

    /* Validate combVec: offset + size <= end, size == mmbit_size(ckeyCount) */
    {
        if (unlikely(rose->ckeyCount > MMB_MAX_BITS)) {
            DEBUG_PRINTF("ckeyCount too large: %u\n", rose->ckeyCount);
            return HS_INVALID;
        }
        u32 expected_size = rt_mmbit_size(rose->ckeyCount);
        if (unlikely(so->combVec_size != expected_size)) {
            DEBUG_PRINTF("combVec_size mismatch: %u != %u (count=%u)\n",
                         so->combVec_size, expected_size, rose->ckeyCount);
            return HS_INVALID;
        }
        if (unlikely(so->combVec > end ||
                     so->combVec_size > end - so->combVec)) {
            DEBUG_PRINTF("combVec region OOB: off=%u size=%u end=%u\n",
                         so->combVec, so->combVec_size, end);
            return HS_INVALID;
        }
    }

    /* Validate SOM regions. */
    if (rose->somLocationCount) {
        /* Upper-bound somLocationCount before it flows into rt_mmbit_size()
         * (whose internal `while` loop would otherwise appear as a tainted
         * loop bound to Coverity). */
        if (unlikely(rose->somLocationCount > MMB_MAX_BITS)) {
            DEBUG_PRINTF("somLocationCount unreasonable: %u\n",
                         rose->somLocationCount);
            return HS_INVALID;
        }
        /* somMultibit_size must match mmbit_size(somLocationCount). */
        u32 expected_mb_size = rt_mmbit_size(rose->somLocationCount);
        if (unlikely(so->somMultibit_size != expected_mb_size)) {
            DEBUG_PRINTF("somMultibit_size mismatch: %u != %u (count=%u)\n",
                         so->somMultibit_size, expected_mb_size,
                         rose->somLocationCount);
            return HS_INVALID;
        }

        /* somValid: offset + somMultibit_size <= end */
        if (unlikely(so->somValid > end ||
                     so->somMultibit_size > end - so->somValid)) {
            DEBUG_PRINTF("somValid region OOB: off=%u size=%u end=%u\n",
                         so->somValid, so->somMultibit_size, end);
            return HS_INVALID;
        }

        /* somWritable: offset + somMultibit_size <= end */
        if (unlikely(so->somWritable > end ||
                     so->somMultibit_size > end - so->somWritable)) {
            DEBUG_PRINTF("somWritable region OOB: off=%u size=%u end=%u\n",
                         so->somWritable, so->somMultibit_size, end);
            return HS_INVALID;
        }

        /* somLocation: the SOM value store.
         * Size = somLocationCount * somHorizon. Use 64-bit to detect overflow.
         * somHorizon is 0 in block mode (SOM values live in scratch only). */
        if (rose->somHorizon) {
            u64a som_store_total =
                (u64a)rose->somLocationCount * (u64a)rose->somHorizon;
            if (unlikely(som_store_total > (u64a)end)) {
                DEBUG_PRINTF("somLocation store overflows: %llu > %u\n",
                             som_store_total, end);
                return HS_INVALID;
            }
            if (unlikely(so->somLocation > end ||
                         (u32)som_store_total > end - so->somLocation)) {
                DEBUG_PRINTF("somLocation region OOB: off=%u size=%llu end=%u\n",
                             so->somLocation, som_store_total, end);
                return HS_INVALID;
            }
        }
    }

    /* Validate nfaStateBegin: must be <= end (NFA state extends to end). */
    if (unlikely(so->nfaStateBegin > end)) {
        DEBUG_PRINTF("nfaStateBegin > end: %u > %u\n", so->nfaStateBegin, end);
        return HS_INVALID;
    }

    /* Validate rolesWithStateCount: the role state multibit occupies
     * mmbit_size(rolesWithStateCount) bytes in stream state, placed after the
     * 1-byte status. It must fit before end. */
    if (rose->rolesWithStateCount) {
        /* Coverity CID 2547130 (TAINTED_SCALAR): rolesWithStateCount comes
         * from the untrusted serialized bytecode and flows into the loop
         * bound inside rt_mmbit_size(). Reject values above the maximum
         * supported multibit size before use. */
        if (unlikely(rose->rolesWithStateCount > MMB_MAX_BITS)) {
            DEBUG_PRINTF("rolesWithStateCount unreasonable: %u\n",
                         rose->rolesWithStateCount);
            return HS_INVALID;
        }
        u32 role_state_size = rt_mmbit_size(rose->rolesWithStateCount);
        /* The role state starts at offset 1 (after status byte). */
        if (unlikely(role_state_size > end - 1)) {
            DEBUG_PRINTF("rolesWithStateCount too large: mmbit_size(%u)=%u > end-1=%u\n",
                         rose->rolesWithStateCount, role_state_size, end - 1);
            return HS_INVALID;
        }
    }

    /* Validate dkeyCount / dkeyLogSize consistency (CWE-122).
     * dkeyLogSize is used as the fatbit size in scratch. If dkeyCount is
     * forged larger than what dkeyLogSize can hold, fatbit operations go OOB.
     * Also guard against integer overflow in scratch sizing. */
    if (rose->dkeyCount) {
        /* Coverity CID 2547130 (TAINTED_SCALAR): dkeyCount comes from the
         * untrusted bytecode and is used as a loop bound inside
         * rt_fatbit_size() -> rt_mmbit_size(). Bound it against
         * MMB_MAX_BITS up-front. */
        if (unlikely(rose->dkeyCount > MMB_MAX_BITS)) {
            DEBUG_PRINTF("dkeyCount unreasonable: %u\n", rose->dkeyCount);
            return HS_INVALID;
        }
        u32 expected_log_size = rt_fatbit_size(rose->dkeyCount);
        if (unlikely(rose->dkeyLogSize < expected_log_size)) {
            DEBUG_PRINTF("dkeyLogSize too small for dkeyCount: %u < %u (count=%u)\n",
                         rose->dkeyLogSize, expected_log_size, rose->dkeyCount);
            return HS_INVALID;
        }
        /* Prevent integer overflow: 2 * sizeof(u64a) * dkeyCount in scratch. */
        if (unlikely(rose->dkeyCount > UINT32_MAX / (2 * sizeof(u64a)))) {
            DEBUG_PRINTF("dkeyCount would overflow scratch sizing: %u\n",
                         rose->dkeyCount);
            return HS_INVALID;
        }
    }

    /* Validate somRevCount: bounded by rose_size (offset array lives in
     * bytecode). Each entry is a u32 offset, accessed via somRevOffsetOffset. */
    if (rose->somRevCount) {
        if (unlikely(rose->somRevCount > rose->size / sizeof(u32))) {
            DEBUG_PRINTF("somRevCount too large: %u (rose_size=%u)\n",
                         rose->somRevCount, rose->size);
            return HS_INVALID;
        }
    }

    return HS_SUCCESS;
}

/**
 * Validate MPV NFA offsets to prevent out-of-bounds access (CWE-787).
 *
 * Forged active_offset/reporter_offset can make queue init and runtime
 * operations touch memory outside stream/scratch state.
 */
static
hs_error_t db_validate_mpv_offsets(const struct RoseEngine *rose,
                                   u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines - nothing to check */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        u64a info_end = (u64a)rose->nfaInfoOffset +
                         ((u64a)qi + 1) * sizeof(struct NfaInfo);
        if (unlikely(info_end > rose_size)) {
            DEBUG_PRINTF("NfaInfo[%u] out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            DEBUG_PRINTF("NFA[%u] header out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        if (nfa->type != MPV_NFA) {
            continue;
        }

        if (unlikely((u64a)ni->nfaOffset + (u64a)nfa->length > (u64a)rose_size)) {
            DEBUG_PRINTF("MPV[%u] body out of bounds\n", qi);
            return HS_INVALID;
        }

        if (unlikely(nfa->length < sizeof(struct NFA) + sizeof(struct mpv))) {
            DEBUG_PRINTF("MPV[%u] too small for mpv header\n", qi);
            return HS_INVALID;
        }

        const struct mpv *m = (const struct mpv *)getImplNfa(nfa);

        if (unlikely(m->kilo_count > MMB_MAX_BITS)) {
            DEBUG_PRINTF("MPV[%u] kilo_count too large: %u\n", qi,
                         m->kilo_count);
            return HS_INVALID;
        }

        u32 mb_size = rt_mmbit_size(m->kilo_count);

        if (unlikely(m->active_offset > nfa->streamStateSize ||
                     mb_size > nfa->streamStateSize - m->active_offset)) {
            DEBUG_PRINTF("MPV[%u] active mmbit OOB: off=%u size=%u stream=%u\n",
                         qi, m->active_offset, mb_size, nfa->streamStateSize);
            return HS_INVALID;
        }

        if (unlikely(m->reporter_offset > nfa->scratchStateSize ||
                     mb_size > nfa->scratchStateSize - m->reporter_offset)) {
            DEBUG_PRINTF("MPV[%u] reporter mmbit OOB: off=%u size=%u scratch=%u\n",
                         qi, m->reporter_offset, mb_size,
                         nfa->scratchStateSize);
            return HS_INVALID;
        }
    }

    return HS_SUCCESS;
}

/**
 * Validate RoseEngine offsets to prevent out-of-bounds access (CWE-125).
 * An attacker could craft a malicious serialized database with corrupted
 * offset values and recomputed HMAC. This function validates that all
 * offsets are within the bounds of the RoseEngine structure to prevent
 * heap out-of-bounds reads during scanning operations.
 */
static
hs_error_t db_validate_rose_offsets(const hs_database_t *db) {
    const struct RoseEngine *rose = hs_get_bytecode(db);

    if (unlikely(!rose)) {
        DEBUG_PRINTF("null rose engine\n");
        return HS_INVALID;
    }

    // Get the total RoseEngine size
    u32 rose_size = rose->size;

    // Validate rose_size itself is reasonable
    if (unlikely(rose_size < sizeof(struct RoseEngine) || rose_size > db->length)) {
        DEBUG_PRINTF("invalid rose size: %u (db length: %u)\n",
                     rose_size, db->length);
        return HS_INVALID;
    }

    // Validate critical offsets that are dereferenced during scanning.
    // Each offset, if non-zero, must be within rose_size bounds.
    // This is a macro to reduce repetition and ensure consistency.
#define VALIDATE_OFFSET(field)                                              \
    if (unlikely(rose->field && rose->field >= rose_size)) {                \
        DEBUG_PRINTF(#field " out of bounds: %u >= %u\n",                  \
                     rose->field, rose_size);                               \
        return HS_INVALID;                                                  \
    }

    // Literal matcher offsets
    VALIDATE_OFFSET(fmatcherOffset);
    VALIDATE_OFFSET(ematcherOffset);
    VALIDATE_OFFSET(amatcherOffset);
    VALIDATE_OFFSET(sbmatcherOffset);
    VALIDATE_OFFSET(drmatcherOffset);
    VALIDATE_OFFSET(longLitTableOffset);
    VALIDATE_OFFSET(smallWriteOffset);

    // NFA / left-engine offsets
    VALIDATE_OFFSET(nfaInfoOffset);
    VALIDATE_OFFSET(leftOffset);
    VALIDATE_OFFSET(activeLeftIterOffset);

    // Program offsets
    VALIDATE_OFFSET(reportProgramOffset);
    VALIDATE_OFFSET(delayProgramOffset);
    VALIDATE_OFFSET(anchoredProgramOffset);
    VALIDATE_OFFSET(eodProgramOffset);
    VALIDATE_OFFSET(flushCombProgramOffset);
    VALIDATE_OFFSET(lastFlushCombProgramOffset);

    // Misc offsets
    VALIDATE_OFFSET(lastByteHistoryIterOffset);
    VALIDATE_OFFSET(eagerIterOffset);
    VALIDATE_OFFSET(somRevOffsetOffset);
    VALIDATE_OFFSET(combInfoMapOffset);
    VALIDATE_OFFSET(logicalTreeOffset);
    VALIDATE_OFFSET(invDkeyOffset);

    // Boundary report program offsets (used by roseRunBoundaryProgram in all
    // modes: block, stream, and vectored).
    VALIDATE_OFFSET(boundary.reportEodOffset);
    VALIDATE_OFFSET(boundary.reportZeroOffset);
    VALIDATE_OFFSET(boundary.reportZeroEodOffset);

#undef VALIDATE_OFFSET

    /* Validate fatbit size / count consistency (CVE-mitigating).
     * A crafted database could set a small fatbit_size with a large count,
     * causing fatbit_set() to write past the allocated scratch row.
     * Upper-bound the count against rose_size to satisfy taint analysis. */
#define VALIDATE_FATBIT(size_field, count_field)                             \
    if (rose->count_field > 0) {                                            \
        if (unlikely(rose->count_field > rose_size)) {                      \
            DEBUG_PRINTF(#count_field " exceeds rose_size: %u > %u\n",     \
                         rose->count_field, rose_size);                     \
            return HS_INVALID;                                              \
        }                                                                   \
        if (rose->size_field < rt_fatbit_size(rose->count_field)) {         \
            DEBUG_PRINTF(#size_field " too small for " #count_field         \
                         ": %u < %u (count=%u)\n",                         \
                         rose->size_field,                                  \
                         rt_fatbit_size(rose->count_field),                 \
                         rose->count_field);                                \
            return HS_INVALID;                                              \
        }                                                                   \
    }

    VALIDATE_FATBIT(anchored_fatbit_size, anchored_count);
    VALIDATE_FATBIT(delay_fatbit_size, delay_count);
    VALIDATE_FATBIT(somLocationFatbitSize, somLocationCount);
    VALIDATE_FATBIT(handledKeyFatbitSize, handledKeyCount);
    VALIDATE_FATBIT(activeQueueArraySize, queueCount);

#undef VALIDATE_FATBIT

    /* Validate queue index fields (CWE-125).
     * These are used as indices into the NfaInfo array (of size queueCount).
     * A crafted database can set them beyond queueCount, causing OOB reads
     * in init_outfixes(), stream.c, and other runtime paths.
     * Applies to all modes: block, stream, and vectored. */
    if (unlikely(rose->outfixEndQueue > rose->queueCount)) {
        DEBUG_PRINTF("outfixEndQueue out of bounds: %u > queueCount %u\n",
                     rose->outfixEndQueue, rose->queueCount);
        return HS_INVALID;
    }
    if (unlikely(rose->outfixBeginQueue > rose->outfixEndQueue)) {
        DEBUG_PRINTF("outfixBeginQueue > outfixEndQueue: %u > %u\n",
                     rose->outfixBeginQueue, rose->outfixEndQueue);
        return HS_INVALID;
    }
    if (unlikely(rose->leftfixBeginQueue > rose->queueCount)) {
        DEBUG_PRINTF("leftfixBeginQueue out of bounds: %u > queueCount %u\n",
                     rose->leftfixBeginQueue, rose->queueCount);
        return HS_INVALID;
    }
    /* initMpvNfa is MO_INVALID_IDX (0xFFFFFFFF) when unused; otherwise it
     * must be a valid queue index. */
    if (rose->initMpvNfa != MO_INVALID_IDX &&
        unlikely(rose->initMpvNfa >= rose->queueCount)) {
        DEBUG_PRINTF("initMpvNfa out of bounds: %u >= queueCount %u\n",
                     rose->initMpvNfa, rose->queueCount);
        return HS_INVALID;
    }

    /* Validate state_init scatter plan offsets (CWE-125 / CWE-787).
     * The scatter function reads arrays from rose + offset, then writes values
     * to stream state. Forged offsets cause OOB reads of scatter entries and
     * potentially OOB writes to state. Each non-zero offset must point within
     * rose_size, and the referenced array must fit entirely.
     * Additionally: non-zero offset requires non-zero count (scatter() asserts
     * this), and the plan array must be properly aligned for its element type
     * to avoid UB on strict-alignment platforms. */
#define VALIDATE_SCATTER(type_suffix, unit_size)                              \
    if (rose->state_init.s_##type_suffix##_offset) {                         \
        if (unlikely(!rose->state_init.s_##type_suffix##_count)) {           \
            DEBUG_PRINTF("state_init.s_" #type_suffix " has offset but zero count\n"); \
            return HS_INVALID;                                               \
        }                                                                    \
        if (unlikely(rose->state_init.s_##type_suffix##_offset %             \
                     alignof(struct scatter_unit_##type_suffix))) {           \
            DEBUG_PRINTF("state_init.s_" #type_suffix " misaligned offset=%u\n", \
                         rose->state_init.s_##type_suffix##_offset);         \
            return HS_INVALID;                                               \
        }                                                                    \
        u64a scatter_end = (u64a)rose->state_init.s_##type_suffix##_offset   \
            + (u64a)rose->state_init.s_##type_suffix##_count * (unit_size);   \
        if (unlikely(scatter_end > rose_size)) {                              \
            DEBUG_PRINTF("state_init.s_" #type_suffix " out of bounds: "     \
                         "offset=%u count=%u end=%llu > rose_size=%u\n",     \
                         rose->state_init.s_##type_suffix##_offset,          \
                         rose->state_init.s_##type_suffix##_count,           \
                         scatter_end, rose_size);                            \
            return HS_INVALID;                                               \
        }                                                                    \
    }

    VALIDATE_SCATTER(u64a, sizeof(struct scatter_unit_u64a));
    VALIDATE_SCATTER(u32, sizeof(struct scatter_unit_u32));
    VALIDATE_SCATTER(u16, sizeof(struct scatter_unit_u16));
    VALIDATE_SCATTER(u8, sizeof(struct scatter_unit_u8));

#undef VALIDATE_SCATTER

    /* Validate count fields that control mmbit/array operations (CWE-125).
     * These are used as sizes/indices for queue-backed arrays and bitsets.
     * Applies to all modes. */
    if (unlikely(rose->activeArrayCount > rose->queueCount)) {
        DEBUG_PRINTF("activeArrayCount too large: %u > queueCount %u\n",
                     rose->activeArrayCount, rose->queueCount);
        return HS_INVALID;
    }
    if (unlikely(rose->activeLeftCount >
                 rose->queueCount - rose->leftfixBeginQueue)) {
        DEBUG_PRINTF("activeLeftCount too large: %u > "
                     "(queueCount-leftfixBeginQueue) %u\n",
                     rose->activeLeftCount,
                     rose->queueCount - rose->leftfixBeginQueue);
        return HS_INVALID;
    }
    if (unlikely(rose->rolesWithStateCount > MMB_MAX_BITS)) {
        DEBUG_PRINTF("rolesWithStateCount too large: %u > %u\n",
                     rose->rolesWithStateCount, MMB_MAX_BITS);
        return HS_INVALID;
    }
    if (unlikely(rose->ekeyCount > MMB_MAX_BITS)) {
        DEBUG_PRINTF("ekeyCount too large: %u > %u\n",
                     rose->ekeyCount, MMB_MAX_BITS);
        return HS_INVALID;
    }
    if (unlikely(rose->lkeyCount > MMB_MAX_BITS ||
                 rose->lopCount > MMB_MAX_BITS - rose->lkeyCount)) {
        DEBUG_PRINTF("logical key/op counts out of bounds: "
                     "lkey=%u lop=%u max=%u\n",
                     rose->lkeyCount, rose->lopCount, MMB_MAX_BITS);
        return HS_INVALID;
    }
    if (unlikely(rose->ckeyCount > MMB_MAX_BITS)) {
        DEBUG_PRINTF("ckeyCount too large: %u > %u\n",
                     rose->ckeyCount, MMB_MAX_BITS);
        return HS_INVALID;
    }
    if (unlikely(rose->dkeyCount > MMB_MAX_BITS)) {
        DEBUG_PRINTF("dkeyCount too large: %u > %u\n",
                     rose->dkeyCount, MMB_MAX_BITS);
        return HS_INVALID;
    }
    /* somRevCount controls indexing into the somRevOffsetOffset u32 table.
     * Validate that the entire table fits within the engine blob. */
    if (rose->somRevCount) {
        if (unlikely(!rose->somRevOffsetOffset)) {
            DEBUG_PRINTF("somRevCount=%u but somRevOffsetOffset is 0\n",
                         rose->somRevCount);
            return HS_INVALID;
        }
        u64a table_end = (u64a)rose->somRevOffsetOffset
            + (u64a)rose->somRevCount * sizeof(u32);
        if (unlikely(table_end > rose_size)) {
            DEBUG_PRINTF("somRev table out of bounds: offset=%u count=%u "
                         "end=%llu > rose_size=%u\n",
                         rose->somRevOffsetOffset, rose->somRevCount,
                         table_end, rose_size);
            return HS_INVALID;
        }
    }

    /* Validate LimEx NFA repeat metadata (CWE-787). */
    if (unlikely(db_validate_limex_repeats(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("LimEx repeat validation failed\n");
        return HS_INVALID;
    }

    /* Validate LimEx NFA reachMap entries (CWE-125). */
    if (unlikely(db_validate_limex_reach_map(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("LimEx reachMap validation failed\n");
        return HS_INVALID;
    }

     /* Validate Tamarama subengine offsets (CWE-125). */
    if (unlikely(db_validate_tamarama_offsets(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("Tamarama subengine offset validation failed\n");
        return HS_INVALID;
    }

    /* Validate FDR engineID (CWE-125). */
    if (unlikely(db_validate_fdr_engine_id(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("FDR engineID validation failed\n");
        return HS_INVALID;
    }

    /* Validate LBR NFA repeatInfoOffset. */
    if (unlikely(db_validate_lbr_repeat_info(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("LBR repeat info validation failed\n");
        return HS_INVALID;
    }

    /* Validate Noodle matcher table fields. */
    if (unlikely(db_validate_noodle_table(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("Noodle table validation failed\n");
        return HS_INVALID;
    }

     /* Validate mcsheng successor table entries (CWE-125). */
    if (unlikely(db_validate_mcsheng_succ_table(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("mcsheng successor table validation failed\n");
        return HS_INVALID;
    }
	
    /* Validate MPV active/reporter offsets (CWE-787). */
    if (unlikely(db_validate_mpv_offsets(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("MPV offset validation failed\n");
        return HS_INVALID;
    }

    // Validate stream-state stateOffsets layout (CWE-122).
    // These invariants mirror fillStateOffsets() in rose_build_bytecode.cpp.
    // Every offset region dereferenced by init_stream() and its helpers
    // must be bounded by stateOffsets.end.
    if (rose->mode == HS_MODE_STREAM) {
        // The status byte at offset 0 is always written, so end must be >= 1.
        if (unlikely(rose->stateOffsets.end < sizeof(u8))) {
            DEBUG_PRINTF("stateOffsets.end too small: %u\n",
                         rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.history > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.history > end: %u > %u\n",
                         rose->stateOffsets.history, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->historyRequired >
            rose->stateOffsets.end - rose->stateOffsets.history)) {
            DEBUG_PRINTF("historyRequired overflows end: %u > %u\n",
                         rose->historyRequired,
                         rose->stateOffsets.end - rose->stateOffsets.history);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.exhausted > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.exhausted > end: %u > %u\n",
                         rose->stateOffsets.exhausted, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.logicalVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.logicalVec > end: %u > %u\n",
                         rose->stateOffsets.logicalVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.combVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.combVec > end: %u > %u\n",
                         rose->stateOffsets.combVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somValid > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somValid > end: %u > %u\n",
                         rose->stateOffsets.somValid, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somWritable > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somWritable > end: %u > %u\n",
                         rose->stateOffsets.somWritable, rose->stateOffsets.end);
            return HS_INVALID;
        }
    }

    // Unified state layout validation (CWE-122, CWE-190).
    // Mirrors fillStateOffsets() in rose_build_bytecode.cpp.
    // Validates ALL modes (block, stream, vectored) so no path is uncovered.
    if (unlikely(validateStateLayout(rose) != HS_SUCCESS)) {
        DEBUG_PRINTF("state layout validation failed\n");
        return HS_INVALID;
    }

    DEBUG_PRINTF("rose offset validation passed\n");
    return HS_SUCCESS;
}

static
void db_copy_bytecode(const char *serialized, hs_database_t *db) {
    // we need to align things manually
    uintptr_t shift = (uintptr_t)db->bytes & 0x3f;
    db->bytecode = offsetof(struct hs_database, bytes) - shift;
    char *bytecode = (char *)db + db->bytecode;

    // Copy the bytecode into place
    memcpy(bytecode, serialized, db->length);
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_deserialize_database_at(const char *bytes,
                                               const size_t length,
                                               hs_database_t *db) {
    if (unlikely(!bytes || !db)) {
        return HS_INVALID;
    }

    // We require the user to deserialize into an 8-byte aligned region.
    if (unlikely(!ISALIGNED_N(db, 8))) {
        return HS_BAD_ALIGN;
    }

    // Decode the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Calculate total required size
    size_t dblength = sizeof(struct hs_database) + header.length;

    // Additional overflow check for size calculation
    if (unlikely(dblength < header.length)) {
        DEBUG_PRINTF("database size calculation overflow\n");
        return HS_INVALID;
    }

    // Validate total deserialized size against centralized defensive limit.
    if (unlikely(dblength > MAX_DATABASE_SIZE)) {
        DEBUG_PRINTF("database total size exceeds maximum: %zu > %zu\n",
                     dblength, (size_t)MAX_DATABASE_SIZE);
        return HS_INVALID;
    }

    // SECURITY NOTE: This function cannot verify that the user-provided 'db'
    // buffer is actually large enough to hold 'dblength' bytes. Users MUST
    // call hs_serialized_database_size() first to determine required size and
    // allocate sufficient memory. The checks above only prevent obviously
    // malicious oversized values to mitigate heap buffer overflow (CWE-122).
    memset(db, 0, dblength);

    // Copy the decoded header into place
    memcpy(db, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, db);

    if (unlikely(db_check_header_integrity(db) != HS_SUCCESS)) {
        return HS_INVALID;
    }

    if (unlikely(db_check_integrity(db) != HS_SUCCESS)) {
        return HS_INVALID;
    }

    // Validate RoseEngine offsets to prevent out-of-bounds access (CWE-125)
    if (unlikely(db_validate_rose_offsets(db) != HS_SUCCESS)) {
        DEBUG_PRINTF("rose offset validation failed\n");
        return HS_INVALID;
    }

    hs_db_protect(db, sizeof(struct hs_database) + header.length);
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_deserialize_database(const char *bytes,
                                            const size_t length,
                                            hs_database_t **db) {
    if (unlikely(!bytes || !db)) {
        return HS_INVALID;
    }

    *db = NULL;

    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Allocate space for new database
    size_t dblength = sizeof(struct hs_database) + header.length;

    // Check for overflow in size calculation
    if (unlikely(dblength < header.length)) {
        DEBUG_PRINTF("database size calculation overflow\n");
        return HS_INVALID;
    }

    // Validate total allocation size against centralized defensive limit.
    if (unlikely(dblength > MAX_DATABASE_SIZE)) {
        DEBUG_PRINTF("database total size exceeds maximum: %zu > %zu\n",
                     dblength, (size_t)MAX_DATABASE_SIZE);
        return HS_INVALID;
    }

    struct hs_database *tempdb = (struct hs_database *)hs_db_alloc(dblength);
    ret = hs_check_alloc(tempdb);
    if (unlikely(ret != HS_SUCCESS)) {
        hs_db_free(tempdb, dblength);
        return ret;
    }

    // Zero new space for safety
    memset(tempdb, 0, dblength);

    // Copy the decoded header into place
    memcpy(tempdb, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, tempdb);

    if (unlikely(db_check_header_integrity(tempdb) != HS_SUCCESS)) {
        hs_db_free(tempdb, dblength);
        return HS_INVALID;
    }

    if (unlikely(db_check_integrity(tempdb) != HS_SUCCESS)) {
        hs_db_free(tempdb, dblength);
        return HS_INVALID;
    }

    // Validate RoseEngine offsets to prevent out-of-bounds access (CWE-125)
    if (unlikely(db_validate_rose_offsets(tempdb) != HS_SUCCESS)) {
        DEBUG_PRINTF("rose offset validation failed\n");
        hs_db_free(tempdb, dblength);
        return HS_INVALID;
    }

    hs_db_protect(tempdb, dblength);
    *db = tempdb;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_database_size(const hs_database_t *db, size_t *size) {
    if (unlikely(!size)) {
        return HS_INVALID;
    }

    hs_error_t ret = validDatabase(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Verify header HMAC to authenticate db->length before trusting it.
    ret = db_check_header_integrity(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    *size = sizeof(struct hs_database) + db->length;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialized_database_size(const char *bytes,
                                                const size_t length,
                                                size_t *size) {
    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    if (unlikely(!size)) {
        return HS_INVALID;
    }

    *size = sizeof(struct hs_database) + header.length;
    return HS_SUCCESS;
}

hs_error_t dbIsValid(const hs_database_t *db) {
    if (unlikely(db->magic != HS_DB_MAGIC)) {
        DEBUG_PRINTF("bad magic\n");
        return HS_INVALID;
    }

    if (unlikely(db->version != HS_DB_VERSION)) {
        DEBUG_PRINTF("bad version\n");
        return HS_DB_VERSION_ERROR;
    }

    if (unlikely(db_check_platform(db->platform) != HS_SUCCESS)) {
        DEBUG_PRINTF("bad platform\n");
        return HS_DB_PLATFORM_ERROR;
    }

    if (unlikely(!ISALIGNED_16(hs_get_bytecode(db)))) {
        DEBUG_PRINTF("bad alignment\n");
        return HS_INVALID;
    }

    // Verify header HMAC to authenticate db->length before trusting it.
    // Then verify full integrity that reads db->length bytes of bytecode.
    hs_error_t rv = db_check_header_integrity(db);
    if (unlikely(rv != HS_SUCCESS)) {
        DEBUG_PRINTF("bad header integrity check\n");
        return rv;
    }

    rv = db_check_integrity(db);
    if (unlikely(rv != HS_SUCCESS)) {
        DEBUG_PRINTF("bad integrity check\n");
        return rv;
    }

    // Validate bytecode offset is within the expected header region.
    if (db->bytecode < offsetof(struct hs_database, padding) ||
        db->bytecode > offsetof(struct hs_database, bytes)) {
        DEBUG_PRINTF("bad bytecode offset\n");
        return HS_INVALID;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);

    // Validate stream-state stateOffsets layout (CWE-122).
    // These invariants mirror fillStateOffsets() in rose_build_bytecode.cpp.
    // Every offset region dereferenced by init_stream() and its helpers
    // must be bounded by stateOffsets.end.
    if (rose->mode == HS_MODE_STREAM) {
        if (unlikely(rose->stateOffsets.end < sizeof(u8))) {
            DEBUG_PRINTF("stateOffsets.end too small: %u\n",
                         rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.history > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.history > end: %u > %u\n",
                         rose->stateOffsets.history, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->historyRequired >
            rose->stateOffsets.end - rose->stateOffsets.history)) {
            DEBUG_PRINTF("historyRequired overflows end: %u > %u\n",
                         rose->historyRequired,
                         rose->stateOffsets.end - rose->stateOffsets.history);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.exhausted > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.exhausted > end: %u > %u\n",
                         rose->stateOffsets.exhausted, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.logicalVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.logicalVec > end: %u > %u\n",
                         rose->stateOffsets.logicalVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.combVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.combVec > end: %u > %u\n",
                         rose->stateOffsets.combVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somValid > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somValid > end: %u > %u\n",
                         rose->stateOffsets.somValid, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somWritable > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somWritable > end: %u > %u\n",
                         rose->stateOffsets.somWritable, rose->stateOffsets.end);
            return HS_INVALID;
        }
    }

    // Validate NfaInfo entries: ensure every fullStateOffset and stateOffset
    // is within its respective buffer.  This prevents out-of-bounds writes
    // via forged NfaInfo fields (CWE-787).
    if (rose->nfaInfoOffset) {
        if (unlikely(rose->nfaInfoOffset >= rose->size)) {
            DEBUG_PRINTF("nfaInfoOffset %u out of range (size=%u)\n",
                         rose->nfaInfoOffset, rose->size);
            return HS_INVALID;
        }

        if (unlikely(rose->size < sizeof(struct NFA))) {
            DEBUG_PRINTF("rose->size %u too small for NFA header\n",
                         rose->size);
            return HS_INVALID;
        }

        u64a nfa_info_table_end = (u64a)rose->nfaInfoOffset +
            (u64a)rose->queueCount * sizeof(struct NfaInfo);
        if (unlikely(nfa_info_table_end > rose->size)) {
            DEBUG_PRINTF("NfaInfo table overflows blob\n");
            return HS_INVALID;
        }

        const struct NfaInfo *infos =
            (const struct NfaInfo *)((const char *)rose + rose->nfaInfoOffset);

        /* Explicit bound check on queueCount to satisfy taint analysis. */
        const u32 queue_count = rose->queueCount;
        if (unlikely(queue_count > rose->size / sizeof(struct NfaInfo))) {
            DEBUG_PRINTF("queueCount %u exceeds maximum\n", queue_count);
            return HS_INVALID;
        }

        for (u32 qi = 0; qi < queue_count; qi++) {
            const struct NfaInfo *info = &infos[qi];

            if (unlikely(info->nfaOffset == 0 ||
                         info->nfaOffset > rose->size - sizeof(struct NFA))) {
                DEBUG_PRINTF("qi=%u: nfaOffset %u out of range (size=%u)\n",
                             qi, info->nfaOffset, rose->size);
                return HS_INVALID;
            }

            const struct NFA *nfa =
                (const struct NFA *)((const char *)rose + info->nfaOffset);

            if (unlikely(info->fullStateOffset > rose->scratchStateSize ||
                         nfa->scratchStateSize >
                             rose->scratchStateSize - info->fullStateOffset)) {
                DEBUG_PRINTF("qi=%u: fullStateOffset OOB\n", qi);
                return HS_INVALID;
            }

            if (unlikely(info->stateOffset > rose->stateOffsets.end ||
                         nfa->streamStateSize >
                             rose->stateOffsets.end - info->stateOffset)) {
                DEBUG_PRINTF("qi=%u: stateOffset OOB\n", qi);
                return HS_INVALID;
            }
        }
    }

    /* Validate LBR NFA repeatInfoOffset. */
    if (unlikely(db_validate_lbr_repeat_info(rose, rose->size) != HS_SUCCESS)) {
        DEBUG_PRINTF("LBR repeat info validation failed\n");
        return HS_INVALID;
    }

    /* Validate Noodle matcher table fields. */
    if (unlikely(db_validate_noodle_table(rose, rose->size) != HS_SUCCESS)) {
        DEBUG_PRINTF("Noodle table validation failed\n");
        return HS_INVALID;
    }

    return HS_SUCCESS;
}

#if defined(_WIN32)
#define SNPRINTF_COMPAT _snprintf
#else
#define SNPRINTF_COMPAT snprintf
#endif

/** Allocate a buffer and prints the database info into it. Returns an
 * appropriate error code on failure, or HS_SUCCESS on success. */
static
hs_error_t print_database_string(char **s, u32 version, const platform_t plat,
                                 u32 raw_mode) {
    assert(s);
    *s = NULL;

    u8 release = (version >> 8) & 0xff;
    u8 minor = (version >> 16) & 0xff;
    u8 major = (version >> 24) & 0xff;

    const char *features = (plat & HS_PLATFORM_NOAVX512VBMI)
                               ? (plat & HS_PLATFORM_NOAVX512)
                                   ? (plat & HS_PLATFORM_NOAVX2) ? "" : "AVX2"
                                   : "AVX512"
                               : "AVX512VBMI";

    const char *mode = NULL;

    if (raw_mode == HS_MODE_STREAM) {
        mode = "STREAM";
    } else if (raw_mode == HS_MODE_VECTORED) {
        mode = "VECTORED";
    } else {
        assert(raw_mode == HS_MODE_BLOCK);
        mode = "BLOCK";
    }

    // Initial allocation size, which should be large enough to print our info.
    // If it isn't, snprintf will tell us and we can resize appropriately.
    size_t len = 256;

    while (1) {
        char *buf = hs_misc_alloc(len);
        hs_error_t ret = hs_check_alloc(buf);
        if (ret != HS_SUCCESS) {
            hs_misc_free(buf);
            return ret;
        }

        // Note: SNPRINTF_COMPAT is a macro defined above, to cope with systems
        // that don't have snprintf but have a workalike.
        int p_len = SNPRINTF_COMPAT(
            buf, len, "Version: %u.%u.%u Features: %s Mode: %s",
            major, minor, release, features, mode);
        if (p_len < 0) {
            DEBUG_PRINTF("snprintf output error, returned %d\n", p_len);
            hs_misc_free(buf);
            break;
        } else if ((size_t)p_len < len) { // output fit within buffer.
            assert(buf[p_len] == '\0');
            *s = buf;
            return HS_SUCCESS;
        } else { // output didn't fit: resize and reallocate.
            len = (size_t)p_len + 1; // must add one for null terminator.
            hs_misc_free(buf);
        }
    }

    return HS_NOMEM;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialized_database_info(const char *bytes,
                                                size_t length, char **info) {
    if (!info) {
        return HS_INVALID;
    }
    *info = NULL;

    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    u32 mode = unaligned_load_u32(bytes + offsetof(struct RoseEngine, mode));

    return print_database_string(info, header.version, header.platform, mode);
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_database_info(const hs_database_t *db, char **info) {
    if (!info) {
        return HS_INVALID;
    }
    *info = NULL;

    if (!db || !db_correctly_aligned(db) || db->magic != HS_DB_MAGIC) {
        return HS_INVALID;
    }

    // Verify header HMAC to authenticate db->length before trusting it.
    hs_error_t ret = db_check_header_integrity(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    platform_t plat;
    plat = db->platform;

    const struct RoseEngine *rose = hs_get_bytecode(db);

    return print_database_string(info, db->version, plat, rose->mode);
}
