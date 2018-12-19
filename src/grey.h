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

#ifndef GREY_H
#define GREY_H

#include <vector>
#include <string>

#include "ue2common.h"

namespace ue2 {

struct Grey {
    Grey(void);

    bool optimiseComponentTree;

    bool calcComponents;
    bool performGraphSimplification;
    bool prefilterReductions;
    bool removeEdgeRedundancy;

    bool allowGough;
    bool allowHaigLit;
    bool allowLitHaig;
    bool allowLbr;
    bool allowMcClellan;
    bool allowSheng;
    bool allowMcSheng;
    bool allowPuff;
    bool allowLiteral;
    bool allowViolet;
    bool allowExtendedNFA;
    bool allowLimExNFA;
    bool allowAnchoredAcyclic;
    bool allowSmallLiteralSet;
    bool allowCastle;
    bool allowDecoratedLiteral;
    bool allowApproximateMatching;

    bool allowNoodle;
    bool fdrAllowTeddy;
    bool fdrAllowFlood;

    u32  violetAvoidSuffixes; /* 0=never, 1=sometimes, 2=always */
    bool violetAvoidWeakInfixes;
    bool violetDoubleCut;
    bool violetExtractStrongLiterals;
    bool violetLiteralChains;
    u32  violetDoubleCutLiteralLen;
    u32  violetEarlyCleanLiteralLen;

    bool puffImproveHead;
    bool castleExclusive; // enable castle mutual exclusion analysis

    bool mergeSEP;
    bool mergeRose;
    bool mergeSuffixes;
    bool mergeOutfixes;
    bool onlyOneOutfix; // if > 1 outfix, fail compile

    bool allowShermanStates;
    bool allowMcClellan8;
    bool allowWideStates; // enable wide state for McClellan8
    bool highlanderPruneDFA;
    bool minimizeDFA;

    bool accelerateDFA;
    bool accelerateNFA;
    bool reverseAccelerate;

    bool squashNFA;
    bool compressNFAState;
    bool numberNFAStatesWrong;
    bool highlanderSquash;
    bool allowZombies;
    bool floodAsPuffette;

    u32 nfaForceSize;

    u32 maxHistoryAvailable;
    u32 minHistoryAvailable;
    u32 maxAnchoredRegion;
    u32 minRoseLiteralLength;
    u32 minRoseNetflowLiteralLength;
    u32 maxRoseNetflowEdges;
    u32 maxEditDistance;

    u32 minExtBoundedRepeatSize; /* to be considered for ng_repeat */

    bool goughCopyPropagate;
    bool goughRegisterAllocate;

    bool shortcutLiterals;

    bool roseGraphReduction;
    bool roseRoleAliasing;
    bool roseMasks;
    bool roseConvertFloodProneSuffixes;
    bool roseMergeRosesDuringAliasing;
    bool roseMultiTopRoses;
    bool roseHamsterMasks;
    bool roseLookaroundMasks;
    u32 roseMcClellanPrefix; /* 0 = off, 1 = only if large nfa, 2 = always */
    u32 roseMcClellanSuffix; /* 0 = off, 1 = only if very large nfa, 2 =
                              * always */
    u32 roseMcClellanOutfix; /* 0 = off, 1 = sometimes, 2 = almost always */
    bool roseTransformDelay;

    bool earlyMcClellanPrefix;
    bool earlyMcClellanInfix;
    bool earlyMcClellanSuffix;

    bool allowCountingMiracles;

    bool allowSomChain;
    u32 somMaxRevNfaLength;

    bool hamsterAccelForward;
    bool hamsterAccelReverse; // currently not implemented

    u32 miracleHistoryBonus; /* cheap hack to make miracles better, TODO
                              * something dignified */

    bool equivalenceEnable;

    // SmallWrite engine
    bool allowSmallWrite;
    bool allowSmallWriteSheng;
    u32 smallWriteLargestBuffer;  // largest buffer that can be small write
    u32 smallWriteLargestBufferBad;// largest buffer that can be small write
    u32 limitSmallWriteOutfixSize; //!< max total size of outfix DFAs
    u32 smallWriteMaxPatterns; // only try small writes if fewer patterns
    u32 smallWriteMaxLiterals; // only try small writes if fewer literals
    u32 smallWriteMergeBatchSize; // number of DFAs to merge in a batch

    // Tamarama engine
    bool allowTamarama;
    u32 tamaChunkSize; //!< max chunk size for exclusivity analysis in Tamarama

    enum DumpFlags {
        DUMP_NONE       = 0,
        DUMP_BASICS     = 1 << 0, // Dump basic textual data
        DUMP_PARSE      = 1 << 1, // Dump component tree to .txt
        DUMP_INT_GRAPH  = 1 << 2, // Dump non-implementation graphs
        DUMP_IMPL       = 1 << 3  // Dump implementation graphs
    };

    u32 dumpFlags;
    std::string dumpPath;

    /* Resource limits. These are somewhat arbitrary, but are intended to bound
     * the input to many of our internal structures. Exceeding one of these
     * limits will cause an error to be returned to the user.
     *
     * NOTE: Raising these limitations make cause smoke to come out of parts of
     * the runtime. */

    u32 limitPatternCount;  //!< max number of patterns
    u32 limitPatternLength; //!< max number of characters in a regex
    u32 limitGraphVertices; //!< max number of states in built NFA graph
    u32 limitGraphEdges;    //!< max number of edges in build NFA graph
    u32 limitReportCount;   //!< max number of ReportIDs allocated internally

    // HWLM literal matcher limits.
    u32 limitLiteralCount;        //!< max number of literals in an HWLM table
    u32 limitLiteralLength;       //!< max number of characters in a literal
    u32 limitLiteralMatcherChars; //!< max characters in an HWLM literal matcher
    u32 limitLiteralMatcherSize;  //!< max size of an HWLM matcher (in bytes)

    // Rose limits.
    u32 limitRoseRoleCount;    //!< max number of Rose roles
    u32 limitRoseEngineCount;  //!< max prefix/infix/suffix/outfix engines
    u32 limitRoseAnchoredSize; //!< max total size of anchored DFAs (bytes)

    // Engine (DFA/NFA/etc) limits.
    u32 limitEngineSize; //!< max size of an engine (in bytes)
    u32 limitDFASize;    //!< max size of a DFA (in bytes)
    u32 limitNFASize;    //!< max size of an NFA (in bytes)
    u32 limitLBRSize;    //!< max size of an LBR engine (in bytes)

    // Approximate matching limits.
    u32 limitApproxMatchingVertices; //!< max number of vertices per graph
};

#ifndef RELEASE_BUILD
#include <string>
void applyGreyOverrides(Grey *g, const std::string &overrides);
#endif

} // namespace ue2

#endif
