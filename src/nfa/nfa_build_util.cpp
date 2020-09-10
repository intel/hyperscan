/*
 * Copyright (c) 2015-2020, Intel Corporation
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

#include "nfa_build_util.h"

#include "limex_internal.h"
#include "mcclellancompile.h"
#include "mcsheng_compile.h"
#include "shengcompile.h"
#include "nfa_internal.h"
#include "repeat_internal.h"
#include "ue2common.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

using namespace std;

namespace ue2 {

namespace {

template<NFAEngineType t> struct NFATraits { };

template<template<NFAEngineType t> class sfunc, typename rv_t, typename arg_t,
         NFAEngineType lb>
struct DISPATCH_BY_NFA_TYPE_INT {
    static rv_t doOp(NFAEngineType i, const arg_t &arg) {
        if (i == lb) {
            return sfunc<lb>::call(arg);
        } else {
            return DISPATCH_BY_NFA_TYPE_INT<sfunc, rv_t, arg_t,
                                            (NFAEngineType)(lb + 1)>
                     ::doOp(i, arg);
        }
    }
};

template<template<NFAEngineType t> class sfunc, typename rv_t, typename arg_t>
struct DISPATCH_BY_NFA_TYPE_INT<sfunc, rv_t, arg_t, INVALID_NFA> {
    // dummy
    static rv_t doOp(NFAEngineType, const arg_t &) {
        assert(0);
        throw std::logic_error("Unreachable");
    }
};

#define DISPATCH_BY_NFA_TYPE(i, op, arg)                                       \
    DISPATCH_BY_NFA_TYPE_INT<op, decltype(op<(NFAEngineType)0>::call(arg)),    \
                             decltype(arg), (NFAEngineType)0>::doOp(i, arg)
}

typedef bool (*nfa_dispatch_fn)(const NFA *nfa);

template<typename T>
static
bool has_accel_limex(const NFA *nfa) {
    const T *limex = (const T *)getImplNfa(nfa);
    return limex->accelCount;
}

template<typename T>
static
bool has_repeats_limex(const NFA *nfa) {
    const T *limex = (const T *)getImplNfa(nfa);
    return limex->repeatCount;
}


template<typename T>
static
bool has_repeats_other_than_firsts_limex(const NFA *nfa) {
    const T *limex = (const T *)getImplNfa(nfa);
    const char *ptr = (const char *)limex;

    const u32 *repeatOffset = (const u32 *)(ptr + limex->repeatOffset);

    for (u32 i = 0; i < limex->repeatCount; i++) {
        u32 offset = repeatOffset[i];
        const NFARepeatInfo *info = (const NFARepeatInfo *)(ptr + offset);
        const RepeatInfo *repeat =
            (const RepeatInfo *)((const char *)info + sizeof(*info));
        if (repeat->type != REPEAT_FIRST) {
            return true;
        }
    }

    return false;
}

static
bool dispatch_false(const NFA *) {
    return false;
}

#ifdef DUMP_SUPPORT
namespace {
template<NFAEngineType t>
struct getName {
    static const char *call(void *) {
        return NFATraits<t>::name;
    }
};

// descr helper for LimEx NFAs
template<NFAEngineType t>
static
string getDescriptionLimEx(const NFA *nfa) {
    const typename NFATraits<t>::implNFA_t *limex =
        (const typename NFATraits<t>::implNFA_t *)getImplNfa(nfa);
    ostringstream oss;
    oss << NFATraits<t>::name << "/" << limex->exceptionCount;
    if (limex->repeatCount) {
        oss << " +" << limex->repeatCount << "r";
    }
    return oss.str();
}
}

// generic description: just return the name
namespace {
template<NFAEngineType t>
struct getDescription {
    static string call(const void *) {
        return string(NFATraits<t>::name);
    }
};
}
#endif


/* build-utility Traits */

namespace {
enum NFACategory {NFA_LIMEX, NFA_OTHER};

// Some of our traits we want around in DUMP_SUPPORT mode only.
#if defined(DUMP_SUPPORT)
#define DO_IF_DUMP_SUPPORT(a) a
#else
#define DO_IF_DUMP_SUPPORT(a)
#endif

#define MAKE_LIMEX_TRAITS(mlt_size, mlt_align)                          \
    template<> struct NFATraits<LIMEX_NFA_##mlt_size> {                 \
        static UNUSED const char *name;                                 \
        static const NFACategory category = NFA_LIMEX;                  \
        typedef LimExNFA##mlt_size implNFA_t;                           \
        static const nfa_dispatch_fn has_accel;                         \
        static const nfa_dispatch_fn has_repeats;                       \
        static const nfa_dispatch_fn has_repeats_other_than_firsts;     \
        static const u32 stateAlign =                                   \
                MAX(mlt_align, alignof(RepeatControl));                 \
    };                                                                  \
    const nfa_dispatch_fn NFATraits<LIMEX_NFA_##mlt_size>::has_accel    \
            = has_accel_limex<LimExNFA##mlt_size>;                      \
    const nfa_dispatch_fn NFATraits<LIMEX_NFA_##mlt_size>::has_repeats  \
            = has_repeats_limex<LimExNFA##mlt_size>;                    \
    const nfa_dispatch_fn                                               \
        NFATraits<LIMEX_NFA_##mlt_size>::has_repeats_other_than_firsts  \
            = has_repeats_other_than_firsts_limex<LimExNFA##mlt_size>;  \
    DO_IF_DUMP_SUPPORT(                                                 \
    const char *NFATraits<LIMEX_NFA_##mlt_size>::name                   \
        = "LimEx "#mlt_size;                                            \
    template<> struct getDescription<LIMEX_NFA_##mlt_size> {            \
        static string call(const void *p) {                             \
            return getDescriptionLimEx<LIMEX_NFA_##mlt_size>((const NFA *)p); \
        }                                                               \
    };)

MAKE_LIMEX_TRAITS(32,  alignof(u32))
MAKE_LIMEX_TRAITS(64,  alignof(m128)) /* special, 32bit arch uses m128 */
MAKE_LIMEX_TRAITS(128, alignof(m128))
MAKE_LIMEX_TRAITS(256, alignof(m256))
MAKE_LIMEX_TRAITS(384, alignof(m384))
MAKE_LIMEX_TRAITS(512, alignof(m512))

template<> struct NFATraits<MCCLELLAN_NFA_8> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_8>::has_accel = has_accel_mcclellan;
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_8>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_8>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCCLELLAN_NFA_8>::name = "McClellan 8";
#endif

template<> struct NFATraits<MCCLELLAN_NFA_16> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 2;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_16>::has_accel = has_accel_mcclellan;
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_16>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCCLELLAN_NFA_16>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCCLELLAN_NFA_16>::name = "McClellan 16";
#endif

template<> struct NFATraits<GOUGH_NFA_8> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<GOUGH_NFA_8>::has_accel = has_accel_mcclellan;
const nfa_dispatch_fn NFATraits<GOUGH_NFA_8>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<GOUGH_NFA_8>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<GOUGH_NFA_8>::name = "Goughfish 8";
#endif

template<> struct NFATraits<GOUGH_NFA_16> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<GOUGH_NFA_16>::has_accel = has_accel_mcclellan;
const nfa_dispatch_fn NFATraits<GOUGH_NFA_16>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<GOUGH_NFA_16>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<GOUGH_NFA_16>::name = "Goughfish 16";
#endif

template<> struct NFATraits<MPV_NFA> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MPV_NFA>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<MPV_NFA>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MPV_NFA>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MPV_NFA>::name = "Mega-Puff-Vac";
#endif

template<> struct NFATraits<CASTLE_NFA> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<CASTLE_NFA>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<CASTLE_NFA>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<CASTLE_NFA>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<CASTLE_NFA>::name = "Castle";
#endif

template<> struct NFATraits<LBR_NFA_DOT> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<LBR_NFA_DOT>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_DOT>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_DOT>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<LBR_NFA_DOT>::name = "Lim Bounded Repeat (D)";
#endif

template<> struct NFATraits<LBR_NFA_VERM> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<LBR_NFA_VERM>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_VERM>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_VERM>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<LBR_NFA_VERM>::name = "Lim Bounded Repeat (V)";
#endif

template<> struct NFATraits<LBR_NFA_NVERM> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<LBR_NFA_NVERM>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_NVERM>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_NVERM>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<LBR_NFA_NVERM>::name = "Lim Bounded Repeat (NV)";
#endif

template<> struct NFATraits<LBR_NFA_SHUF> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<LBR_NFA_SHUF>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_SHUF>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_SHUF>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<LBR_NFA_SHUF>::name = "Lim Bounded Repeat (S)";
#endif

template<> struct NFATraits<LBR_NFA_TRUF> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 8;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<LBR_NFA_TRUF>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_TRUF>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<LBR_NFA_TRUF>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<LBR_NFA_TRUF>::name = "Lim Bounded Repeat (M)";
#endif

template<> struct NFATraits<SHENG_NFA> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<SHENG_NFA>::has_accel = has_accel_sheng;
const nfa_dispatch_fn NFATraits<SHENG_NFA>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<SHENG_NFA>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<SHENG_NFA>::name = "Sheng";
#endif

template<> struct NFATraits<TAMARAMA_NFA> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 64;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<TAMARAMA_NFA>::has_accel = dispatch_false;
const nfa_dispatch_fn NFATraits<TAMARAMA_NFA>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<TAMARAMA_NFA>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<TAMARAMA_NFA>::name = "Tamarama";
#endif

template<> struct NFATraits<MCSHENG_NFA_8> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_8>::has_accel = has_accel_mcsheng;
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_8>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_8>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCSHENG_NFA_8>::name = "Shengy McShengFace 8";
#endif

template<> struct NFATraits<MCSHENG_NFA_16> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 2;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_16>::has_accel = has_accel_mcsheng;
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_16>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCSHENG_NFA_16>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCSHENG_NFA_16>::name = "Shengy McShengFace 16";
#endif

template<> struct NFATraits<SHENG_NFA_32> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<SHENG_NFA_32>::has_accel = has_accel_sheng;
const nfa_dispatch_fn NFATraits<SHENG_NFA_32>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<SHENG_NFA_32>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<SHENG_NFA_32>::name = "Sheng 32";
#endif

template<> struct NFATraits<SHENG_NFA_64> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<SHENG_NFA_64>::has_accel = has_accel_sheng;
const nfa_dispatch_fn NFATraits<SHENG_NFA_64>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<SHENG_NFA_64>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<SHENG_NFA_64>::name = "Sheng 64";
#endif

template<> struct NFATraits<MCSHENG_64_NFA_8> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 1;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_8>::has_accel = has_accel_mcsheng;
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_8>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_8>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCSHENG_64_NFA_8>::name = "Shengy64 McShengFace 8";
#endif

template<> struct NFATraits<MCSHENG_64_NFA_16> {
    UNUSED static const char *name;
    static const NFACategory category = NFA_OTHER;
    static const u32 stateAlign = 2;
    static const nfa_dispatch_fn has_accel;
    static const nfa_dispatch_fn has_repeats;
    static const nfa_dispatch_fn has_repeats_other_than_firsts;
};
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_16>::has_accel = has_accel_mcsheng;
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_16>::has_repeats = dispatch_false;
const nfa_dispatch_fn NFATraits<MCSHENG_64_NFA_16>::has_repeats_other_than_firsts = dispatch_false;
#if defined(DUMP_SUPPORT)
const char *NFATraits<MCSHENG_64_NFA_16>::name = "Shengy64 McShengFace 16";
#endif
} // namespace

#if defined(DUMP_SUPPORT)

const char *nfa_type_name(NFAEngineType type) {
    return DISPATCH_BY_NFA_TYPE(type, getName, nullptr);
}

string describe(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type, getDescription, &nfa);
}

#endif /* DUMP_SUPPORT */

namespace {
template<NFAEngineType t>
struct getStateAlign {
    static u32 call(void *) {
        return NFATraits<t>::stateAlign;
    }
};
}

u32 state_alignment(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type, getStateAlign, nullptr);
}

namespace {
template<NFAEngineType t>
struct is_limex {
    static bool call(const void *) {
        return NFATraits<t>::category == NFA_LIMEX;
    }
};
}

namespace {
template<NFAEngineType t>
struct has_repeats_other_than_firsts_dispatch {
    static nfa_dispatch_fn call(const void *) {
        return NFATraits<t>::has_repeats_other_than_firsts;
    }
};
}

bool has_bounded_repeats_other_than_firsts(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type,
                                has_repeats_other_than_firsts_dispatch,
                                &nfa)(&nfa);
}

namespace {
template<NFAEngineType t>
struct has_repeats_dispatch {
    static nfa_dispatch_fn call(const void *) {
        return NFATraits<t>::has_repeats;
    }
};
}

bool has_bounded_repeats(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type, has_repeats_dispatch,
                                &nfa)(&nfa);
}

namespace {
template<NFAEngineType t>
struct has_accel_dispatch {
    static nfa_dispatch_fn call(const void *) {
        return NFATraits<t>::has_accel;
    }
};
}

bool has_accel(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type, has_accel_dispatch,
                                &nfa)(&nfa);
}

bool requires_decompress_key(const NFA &nfa) {
    return DISPATCH_BY_NFA_TYPE((NFAEngineType)nfa.type, is_limex, &nfa);
}

} // namespace ue2
