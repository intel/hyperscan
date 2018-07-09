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

/** \file
 * \brief Parse and build ParsedLogical::logicalTree and combInfoMap.
 */
#include "logical_combination.h"
#include "parser/parse_error.h"
#include "util/container.h"
#include "hs_compile.h"

#include <vector>

using namespace std;

namespace ue2 {

u32 ParsedLogical::getLogicalKey(u32 a) {
    auto it = toLogicalKeyMap.find(a);
    if (it == toLogicalKeyMap.end()) {
        // get size before assigning to avoid wacky LHS shenanigans
        u32 size = toLogicalKeyMap.size();
        bool inserted;
        tie(it, inserted) = toLogicalKeyMap.emplace(a, size);
        assert(inserted);
    }
    DEBUG_PRINTF("%u -> lkey %u\n", it->first, it->second);
    return it->second;
}

u32 ParsedLogical::getCombKey(u32 a) {
    auto it = toCombKeyMap.find(a);
    if (it == toCombKeyMap.end()) {
        u32 size = toCombKeyMap.size();
        bool inserted;
        tie(it, inserted) = toCombKeyMap.emplace(a, size);
        assert(inserted);
    }
    DEBUG_PRINTF("%u -> ckey %u\n", it->first, it->second);
    return it->second;
}

void ParsedLogical::addRelateCKey(u32 lkey, u32 ckey) {
    auto it = lkey2ckeys.find(lkey);
    if (it == lkey2ckeys.end()) {
        bool inserted;
        tie(it, inserted) = lkey2ckeys.emplace(lkey, set<u32>());
        assert(inserted);
    }
    it->second.insert(ckey);
    DEBUG_PRINTF("lkey %u belongs to combination key %u\n",
                 it->first, ckey);
}

#define TRY_RENUM_OP(ckey)                                                   \
do {                                                                         \
    if (ckey & LOGICAL_OP_BIT) {                                             \
        ckey = (ckey & ~LOGICAL_OP_BIT) + toLogicalKeyMap.size();            \
    }                                                                        \
} while(0)

u32 ParsedLogical::logicalTreeAdd(u32 op, u32 left, u32 right) {
    LogicalOp lop;
    assert((LOGICAL_OP_BIT & (u32)logicalTree.size()) == 0);
    lop.id = LOGICAL_OP_BIT | (u32)logicalTree.size();
    lop.op = op;
    lop.lo = left;
    lop.ro = right;
    logicalTree.push_back(lop);
    return lop.id;
}

void ParsedLogical::combinationInfoAdd(UNUSED u32 ckey, u32 id, u32 ekey,
                                       u32 lkey_start, u32 lkey_result,
                                       u64a min_offset, u64a max_offset) {
    assert(ckey == combInfoMap.size());
    CombInfo ci;
    ci.id = id;
    ci.ekey = ekey;
    ci.start = lkey_start;
    ci.result = lkey_result;
    ci.min_offset = min_offset;
    ci.max_offset = max_offset;
    combInfoMap.push_back(ci);

    DEBUG_PRINTF("ckey %u (id %u) -> lkey %u..%u, ekey=0x%x\n", ckey, ci.id,
                 ci.start, ci.result, ci.ekey);
}

void ParsedLogical::validateSubIDs(const unsigned *ids,
                                   const char *const *expressions,
                                   const unsigned *flags,
                                   unsigned elements) {
    for (const auto &it : toLogicalKeyMap) {
        bool unknown = true;
        u32 i = 0;
        for (i = 0; i < elements; i++) {
            if ((ids ? ids[i] : 0) == it.first) {
                unknown = false;
                break;
            }
        }
        if (unknown) {
            throw CompileError("Unknown sub-expression id.");
        }
        if (contains(toCombKeyMap, it.first)) {
            throw CompileError("Have combination of combination.");
        }
        if (flags && (flags[i] & HS_FLAG_SOM_LEFTMOST)) {
            throw CompileError("Have SOM flag in sub-expression.");
        }
        if (flags && (flags[i] & HS_FLAG_PREFILTER)) {
            throw CompileError("Have PREFILTER flag in sub-expression.");
        }
        hs_compile_error_t *compile_err = NULL;
        hs_expr_info_t *info = NULL;
        hs_error_t err = hs_expression_info(expressions[i], flags[i], &info,
                                            &compile_err);
        if (err != HS_SUCCESS) {
            hs_free_compile_error(compile_err);
            throw CompileError("Run hs_expression_info() failed.");
        }
        if (!info) {
            throw CompileError("Get hs_expr_info_t failed.");
        } else {
            if (info->unordered_matches) {
                throw CompileError("Have unordered match in sub-expressions.");
            }
            free(info);
        }
    }
}

void ParsedLogical::logicalKeyRenumber() {
    // renumber operation lkey in op vector
    for (auto &op : logicalTree) {
        TRY_RENUM_OP(op.id);
        TRY_RENUM_OP(op.lo);
        TRY_RENUM_OP(op.ro);
    }
    // renumber operation lkey in info map
    for (auto &ci : combInfoMap) {
        TRY_RENUM_OP(ci.start);
        TRY_RENUM_OP(ci.result);
    }
}

struct LogicalOperator {
    LogicalOperator(u32 op_in, u32 paren_in)
        : op(op_in), paren(paren_in) {}
    u32 op;
    u32 paren;
};

static
u32 toOperator(char c) {
    u32 op = UNKNOWN_OP;
    switch (c) {
    case '!' :
        op = LOGICAL_OP_NOT;
        break;
    case '&' :
        op = LOGICAL_OP_AND;
        break;
    case '|' :
        op = LOGICAL_OP_OR;
        break;
    default:
        break;
    };
    return op;
}

static
bool cmpOperator(const LogicalOperator &op1, const LogicalOperator &op2) {
    if (op1.paren < op2.paren) {
        return false;
    }
    if (op1.paren > op2.paren) {
        return true;
    }
    assert(op1.paren == op2.paren);
    if (op1.op > op2.op) {
        return false;
    }
    if (op1.op < op2.op) {
        return true;
    }
    return true;
}

static
u32 fetchSubID(const char *logical, u32 &digit, u32 end) {
    if (digit == (u32)-1) { // no digit parsing in progress
        return (u32)-1;
    }
    assert(end > digit);
    if (end - digit > 9) {
        throw LocatedParseError("Expression id too large");
    }
    u32 mult = 1;
    u32 sum = 0;
    for (u32 j = end - 1; (j >= digit) && (j != (u32)-1) ; j--) {
        assert(isdigit(logical[j]));
        sum += (logical[j] - '0') * mult;
        mult *= 10;
    }
    digit = (u32)-1;
    return sum;
}

static
void popOperator(vector<LogicalOperator> &op_stack, vector<u32> &subid_stack,
                 ParsedLogical &pl) {
    if (subid_stack.empty()) {
        throw LocatedParseError("Not enough operand");
    }
    u32 right = subid_stack.back();
    subid_stack.pop_back();
    u32 left = 0;
    if (op_stack.back().op != LOGICAL_OP_NOT) {
        if (subid_stack.empty()) {
            throw LocatedParseError("Not enough operand");
        }
        left = subid_stack.back();
        subid_stack.pop_back();
    }
    subid_stack.push_back(pl.logicalTreeAdd(op_stack.back().op, left, right));
    op_stack.pop_back();
}

static
char getValue(const vector<char> &lv, u32 ckey) {
    if (ckey & LOGICAL_OP_BIT) {
        return lv[ckey & ~LOGICAL_OP_BIT];
    } else {
        return 0;
    }
}

static
bool hasMatchFromPurelyNegative(const vector<LogicalOp> &tree,
                                u32 start, u32 result) {
    vector<char> lv(tree.size());
    assert(start <= result);
    for (u32 i = start; i <= result; i++) {
        assert(i & LOGICAL_OP_BIT);
        const LogicalOp &op = tree[i & ~LOGICAL_OP_BIT];
        assert(i == op.id);
        switch (op.op) {
        case LOGICAL_OP_NOT:
            lv[op.id & ~LOGICAL_OP_BIT] = !getValue(lv, op.ro);
            break;
        case LOGICAL_OP_AND:
            lv[op.id & ~LOGICAL_OP_BIT] = getValue(lv, op.lo) &
                                          getValue(lv, op.ro);
            break;
        case LOGICAL_OP_OR:
            lv[op.id & ~LOGICAL_OP_BIT] = getValue(lv, op.lo) |
                                          getValue(lv, op.ro);
            break;
        default:
            assert(0);
            break;
        }
    }
    return lv[result & ~LOGICAL_OP_BIT];
}

void ParsedLogical::parseLogicalCombination(unsigned id, const char *logical,
                                            u32 ekey, u64a min_offset,
                                            u64a max_offset) {
    u32 ckey = getCombKey(id);
    vector<LogicalOperator> op_stack;
    vector<u32> subid_stack;
    u32 lkey_start = INVALID_LKEY; // logical operation's lkey
    u32 paren = 0; // parentheses
    u32 digit = (u32)-1; // digit start offset, invalid offset is -1
    u32 subid = (u32)-1;
    u32 i;
    try {
        for (i = 0; logical[i]; i++) {
            if (isdigit(logical[i])) {
                if (digit == (u32)-1) { // new digit start
                    digit = i;
                }
            } else {
                if ((subid = fetchSubID(logical, digit, i)) != (u32)-1) {
                    subid_stack.push_back(getLogicalKey(subid));
                    addRelateCKey(subid_stack.back(), ckey);
                }
                if (logical[i] == ' ') { // skip whitespace
                    continue;
                }
                if (logical[i] == '(') {
                    paren += 1;
                } else if (logical[i] == ')') {
                    if (paren <= 0) {
                        throw LocatedParseError("Not enough left parentheses");
                    }
                    paren -= 1;
                } else {
                    u32 prio = toOperator(logical[i]);
                    if (prio != UNKNOWN_OP) {
                        LogicalOperator op(prio, paren);
                        while (!op_stack.empty()
                               && cmpOperator(op_stack.back(), op)) {
                            popOperator(op_stack, subid_stack, *this);
                            if (lkey_start == INVALID_LKEY) {
                                lkey_start = subid_stack.back();
                            }
                        }
                        op_stack.push_back(op);
                    } else {
                        throw LocatedParseError("Unknown character");
                    }
                }
            }
        }
        if (paren != 0) {
            throw LocatedParseError("Not enough right parentheses");
        }
        if ((subid = fetchSubID(logical, digit, i)) != (u32)-1) {
            subid_stack.push_back(getLogicalKey(subid));
            addRelateCKey(subid_stack.back(), ckey);
        }
        while (!op_stack.empty()) {
            popOperator(op_stack, subid_stack, *this);
            if (lkey_start == INVALID_LKEY) {
                lkey_start = subid_stack.back();
            }
        }
        if (subid_stack.size() != 1) {
            throw LocatedParseError("Not enough operator");
        }
    } catch (LocatedParseError &error) {
        error.locate(i);
        throw;
    }
    u32 lkey_result = subid_stack.back(); // logical operation's lkey
    if (lkey_start == INVALID_LKEY) {
        throw CompileError("No logical operation.");
    }
    if (hasMatchFromPurelyNegative(logicalTree, lkey_start, lkey_result)) {
        throw CompileError("Has match from purely negative sub-expressions.");
    }
    combinationInfoAdd(ckey, id, ekey, lkey_start, lkey_result,
                       min_offset, max_offset);
}

} // namespace ue2
