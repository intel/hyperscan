/*
 * Copyright (c) 2016, Intel Corporation
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

#ifndef DATACORPUS_H
#define DATACORPUS_H

#include <vector>
#include <string>

class DataBlock {
public:
    DataBlock(unsigned int in_id, unsigned int in_stream,
              unsigned int int_stream_index_in, std::string in_data)
        : id(in_id), stream_id(in_stream),
          internal_stream_index(int_stream_index_in),
          payload(std::move(in_data)) {}

    unsigned int id;        // unique block identifier
    unsigned int stream_id; // unique stream identifier (from corpus file)
    unsigned int internal_stream_index; /* dense index for this stream
                                         * (allocated by hsbench) */
    std::string payload;    // actual block payload
};

/** Exception thrown if an error occurs. */
class DataCorpusError {
public:
    explicit DataCorpusError(std::string msg_in) : msg(std::move(msg_in)) {}
    std::string msg;
};

/**
 * Interface to a corpus database. Any error will produce a DataCorpusError
 * and should be considered fatal.
 */
std::vector<DataBlock> readCorpus(const std::string &filename);

#endif // DATACORPUS_H
