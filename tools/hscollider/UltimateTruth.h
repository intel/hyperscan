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

#ifndef ULTIMATETRUTH_H
#define ULTIMATETRUTH_H

#include "expressions.h"

#include "hs.h"

#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include <boost/core/noncopyable.hpp>

namespace ue2 {

struct Grey;

} // namespace ue2

class HyperscanDB;
class ResultSet;

// Wrapper around ue2 to generate results for an expression and corpus.
class UltimateTruth : boost::noncopyable {
public:
    UltimateTruth(std::ostream &os, const ExpressionMap &expr,
                  const hs_platform_info *plat, const ue2::Grey &grey,
                  unsigned streamBlocks = 0);

    ~UltimateTruth();

    std::shared_ptr<HyperscanDB> compile(const std::set<unsigned> &ids,
                                         std::string &error) const;

    bool saveDatabase(const HyperscanDB &db,
                      const std::string &filename) const;

    std::shared_ptr<HyperscanDB>
    loadDatabase(const std::string &filename,
                 const std::set<unsigned> &ids) const;

    // Are we runnable? (i.e. not xcompiling)
    bool runnable() const {
        return !m_xcompile;
    }

    bool run(unsigned id, std::shared_ptr<const HyperscanDB> db,
             const std::string &buffer, bool single_pattern, unsigned align,
             ResultSet &rs);

    // Returns a value completely representing this object's compile options.
    unsigned int describe() const;

    std::string dbFilename(const std::set<unsigned int> &ids) const;

private:
    bool blockScan(const HyperscanDB &db, const std::string &buffer,
                   size_t align, match_event_handler callback, void *ctx,
                   ResultSet *rs);
    bool streamingScan(const HyperscanDB &db, const std::string &buffer,
                       size_t align, match_event_handler callback, void *ctx,
                       ResultSet *rs);
    bool vectoredScan(const HyperscanDB &db, const std::string &buffer,
                      size_t align, match_event_handler callback, void *ctx,
                      ResultSet *rs);

    char *setupScanBuffer(const char *buf, size_t len, size_t align);

    char *setupVecScanBuffer(const char *buf, size_t len, size_t align,
                             unsigned int block_id);

    bool allocScratch(std::shared_ptr<const HyperscanDB> db);

    bool cloneScratch(void);

    std::string dbSettingsHash(const std::set<unsigned int> &ids) const;

    const ue2::Grey &grey;

    // Output stream.
    std::ostream &out;

    // Our expression map
    const ExpressionMap &m_expr;

    // Are we cross-compiling, and therefore unable to scan at all?
    bool m_xcompile;

    // Our mode flags to pass into the compiler: calculated from streaming,
    // etc.
    unsigned m_mode;

    // In streaming mode, what is the number of blocks to chop data into?
    unsigned m_streamBlocks;

    // Scratch space for Hyperscan.
    hs_scratch_t *scratch;

    // Temporary scan buffer used for realigned scanning
    std::vector<char> m_scanBuf;

    std::vector<std::vector<char> > raw_blocks; /* temp scan buffers used by
                                                 * vectored mode */

    // Last database we successfully allocated scratch for, so that we can
    // avoid unnecessarily reallocating for it.
    std::shared_ptr<const HyperscanDB> last_db;

    const hs_platform_info *platform;
};

#endif
