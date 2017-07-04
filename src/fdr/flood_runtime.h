/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#ifndef FLOOD_RUNTIME
#define FLOOD_RUNTIME

#if defined(ARCH_64_BIT)
#define FLOOD_64
#else
#define FLOOD_32
#endif
#define FLOOD_MINIMUM_SIZE 256
#define FLOOD_BACKOFF_START 32

static really_inline
const u8 * nextFloodDetect(const u8 * buf, size_t len, u32 floodBackoff) {
    // if we don't have a flood at either the start or end,
    // or have a very small buffer, don't bother with flood detection
    if (len < FLOOD_MINIMUM_SIZE) {
        return buf + len;
    }

    /* entry points in runtime.c prefetch relevant data */
#ifndef FLOOD_32
    u64a x11 = *(const u64a *)ROUNDUP_PTR(buf, 8);
    u64a x12 = *(const u64a *)ROUNDUP_PTR(buf+8, 8);
    if (x11 == x12) {
        return buf + floodBackoff;
    }
    u64a x21 = *(const u64a *)ROUNDUP_PTR(buf + len/2, 8);
    u64a x22 = *(const u64a *)ROUNDUP_PTR(buf + len/2 + 8, 8);
    if (x21 == x22) {
        return buf + floodBackoff;
    }
    u64a x31 = *(const u64a *)ROUNDUP_PTR(buf + len - 24, 8);
    u64a x32 = *(const u64a *)ROUNDUP_PTR(buf + len - 16, 8);
    if (x31 == x32) {
        return buf + floodBackoff;
    }
#else
    u32 x11 = *(const u32 *)ROUNDUP_PTR(buf, 4);
    u32 x12 = *(const u32 *)ROUNDUP_PTR(buf+4, 4);
    if (x11 == x12) {
        return buf + floodBackoff;
    }
    u32 x21 = *(const u32 *)ROUNDUP_PTR(buf + len/2, 4);
    u32 x22 = *(const u32 *)ROUNDUP_PTR(buf + len/2 + 4, 4);
    if (x21 == x22) {
        return buf + floodBackoff;
    }
    u32 x31 = *(const u32 *)ROUNDUP_PTR(buf + len - 12, 4);
    u32 x32 = *(const u32 *)ROUNDUP_PTR(buf + len - 8, 4);
    if (x31 == x32) {
        return buf + floodBackoff;
    }
#endif
    return buf + len;
}

static really_inline
const u8 * floodDetect(const struct FDR * fdr,
                       const struct FDR_Runtime_Args * a,
                       const u8 ** ptrPtr,
                       const u8 * tryFloodDetect,
                       u32 * floodBackoffPtr,
                       hwlmcb_rv_t * control,
                       u32 iterBytes) {
    DEBUG_PRINTF("attempting flood detection at %p\n", tryFloodDetect);
    const u8 * buf = a->buf;
    const size_t len = a->len;
    HWLMCallback cb = a->cb;
    struct hs_scratch *scratch = a->scratch;

    const u8 * ptr = *ptrPtr;
    // tryFloodDetect is never put in places where unconditional
    // reads a short distance forward or backward here
    // TODO: rationale for this line needs to be rediscovered!!
    size_t mainLoopLen = len > 2 * iterBytes ? len - 2 * iterBytes : 0;
    const u32 i = ptr - buf;
    u32 j = i;

    // go from c to our FDRFlood structure
    u8 c = buf[i];
    const u8 * fBase = ((const u8 *)fdr) + fdr->floodOffset;
    u32 fIdx = ((const u32 *)fBase)[c];
    const struct FDRFlood * fsb = (const struct FDRFlood *)(fBase + sizeof(u32) * 256);
    const struct FDRFlood * fl = &fsb[fIdx];

#ifndef FLOOD_32
    u64a cmpVal = c;
    cmpVal |= cmpVal << 8;
    cmpVal |= cmpVal << 16;
    cmpVal |= cmpVal << 32;
    u64a probe = *(const u64a *)ROUNDUP_PTR(buf+i, 8);
#else
    u32 cmpVal = c;
    cmpVal |= cmpVal << 8;
    cmpVal |= cmpVal << 16;
    u32 probe = *(const u32 *)ROUNDUP_PTR(buf+i, 4);
#endif

    if ((probe != cmpVal) || (fl->idCount >= FDR_FLOOD_MAX_IDS)) {
        *floodBackoffPtr *= 2;
        goto floodout;
    }

    if (i < fl->suffix + 7) {
        *floodBackoffPtr *= 2;
        goto floodout;
    }

    j = i - fl->suffix;

#ifndef FLOOD_32
    j -= (u32)((uintptr_t)buf + j) & 0x7; // push j back to yield 8-aligned addrs
    for (; j + 32 < mainLoopLen; j += 32) {
        u64a v = *(const u64a *)(buf + j);
        u64a v2 = *(const u64a *)(buf + j + 8);
        u64a v3 = *(const u64a *)(buf + j + 16);
        u64a v4 = *(const u64a *)(buf + j + 24);
        if ((v4 != cmpVal) || (v3 != cmpVal) || (v2 != cmpVal) || (v != cmpVal)) {
            break;
        }
    }
    for (; j + 8 < mainLoopLen; j += 8) {
        u64a v = *(const u64a *)(buf + j);
        if (v != cmpVal) {
            break;
        }
    }
#else
    j -= (u32)((size_t)buf + j) & 0x3; // push j back to yield 4-aligned addrs
    for (; j + 16 < mainLoopLen; j += 16) {
        u32 v = *(const u32 *)(buf + j);
        u32 v2 = *(const u32 *)(buf + j + 4);
        u32 v3 = *(const u32 *)(buf + j + 8);
        u32 v4 = *(const u32 *)(buf + j + 12);
        if ((v4 != cmpVal) || (v3 != cmpVal) || (v2 != cmpVal) || (v != cmpVal)) {
            break;
        }
    }
    for (; j + 4 < mainLoopLen; j += 4) {
        u32 v = *(const u32 *)(buf + j);
        if (v != cmpVal) {
            break;
        }
    }
#endif
    for (; j < mainLoopLen; j++) {
        u8 v = *(const u8 *)(buf + j);
        if (v != c) {
            break;
        }
    }
    if (j > i ) {
        j--; // needed for some reaches
        u32 itersAhead = (j-i)/iterBytes;
        u32 floodSize = itersAhead*iterBytes;

        DEBUG_PRINTF("flooding %u size j %u i %u fl->idCount %hu "
                     "*control %016llx fl->allGroups %016llx\n",
                     floodSize, j, i, fl->idCount, *control, fl->allGroups);
        DEBUG_PRINTF("mainloopLen %zu mainStart ??? mainEnd ??? len %zu\n",
                     mainLoopLen, len);

        if (fl->idCount && (*control & fl->allGroups)) {
            switch (fl->idCount) {
#if !defined(FLOOD_DEBUG)
            // Carefully unrolled code
            case 1:
                for (u32 t = 0; t < floodSize && (*control & fl->allGroups);
                     t += 4) {
                    DEBUG_PRINTF("aaa %u %llx\n", t, fl->groups[0]);
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 0, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 1, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 2, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 3, fl->ids[0], scratch);
                    }
                }
                break;
            case 2:
                for (u32 t = 0; t < floodSize && (*control & fl->allGroups); t += 4) {
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control =
                            cb(i + t + 1, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t + 1, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 2, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t + 2, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 3, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t + 3, fl->ids[1], scratch);
                    }
                }
                break;
            case 3:
                for (u32 t = 0; t < floodSize && (*control & fl->allGroups); t += 2) {
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[2]) {
                        *control = cb(i + t, fl->ids[2], scratch);
                    }
                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 1, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t + 1, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[2]) {
                        *control = cb(i + t + 1, fl->ids[2], scratch);
                    }
                }
                break;
            default:
                // slow generalized loop
                for (u32 t = 0; t < floodSize && (*control & fl->allGroups); t += 2) {

                    if (*control & fl->groups[0]) {
                        *control = cb(i + t, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[2]) {
                        *control = cb(i + t, fl->ids[2], scratch);
                    }
                    if (*control & fl->groups[3]) {
                        *control = cb(i + t, fl->ids[3], scratch);
                    }

                    for (u32 t2 = 4; t2 < fl->idCount; t2++) {
                        if (*control & fl->groups[t2]) {
                            *control = cb(i + t, fl->ids[t2], scratch);
                        }
                    }

                    if (*control & fl->groups[0]) {
                        *control = cb(i + t + 1, fl->ids[0], scratch);
                    }
                    if (*control & fl->groups[1]) {
                        *control = cb(i + t + 1, fl->ids[1], scratch);
                    }
                    if (*control & fl->groups[2]) {
                        *control = cb(i + t + 1, fl->ids[2], scratch);
                    }
                    if (*control & fl->groups[3]) {
                        *control = cb(i + t + 1, fl->ids[3], scratch);
                    }

                    for (u32 t2 = 4; t2 < fl->idCount; t2++) {
                        if (*control & fl->groups[t2]) {
                            *control = cb(i + t + 1, fl->ids[t2], scratch);
                        }
                    }
                }
                break;
#else
            // Fallback for debugging
            default:
                for (u32 t = 0; t < floodSize && (*control & fl->allGroups); t++) {
                    for (u32 t2 = 0; t2 < fl->idCount; t2++) {
                        if (*control & fl->groups[t2]) {
                            *control = cb(i + t, fl->ids[t2], scratch);
                        }
                    }
                }
#endif
            }
        }
        ptr += floodSize;
    } else {
        *floodBackoffPtr *= 2;
    }

floodout:
    if (j + *floodBackoffPtr < mainLoopLen - 128) {
        tryFloodDetect = buf + MAX(i,j) + *floodBackoffPtr;
    } else {
        tryFloodDetect = buf + mainLoopLen; // set so we never do another flood detect
    }
    *ptrPtr = ptr;
    DEBUG_PRINTF("finished flood detection at %p (next check %p)\n",
                 ptr, tryFloodDetect);
    return tryFloodDetect;
}

#endif
