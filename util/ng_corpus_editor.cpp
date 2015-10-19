/*
 * Copyright (c) 2015, Intel Corporation
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
 * \brief Corpus Editor: applies random transformation to a corpus.
 */

#include "config.h"

#include "ng_corpus_editor.h"
#include "ng_corpus_properties.h"
#include "ue2common.h"
#include "util/compare.h"
#include "util/unicode_def.h"
#include "parser/ucp_table.h"

#include <algorithm>
#include <cassert>
#include <string>

using namespace std;
using namespace ue2;

namespace {

enum Operation {
    EDIT_INSERT = 0,     //!< insert a character
    EDIT_REMOVE = 1,     //!< remove a character
    EDIT_SUBSTITUTE = 2, //!< substitute a character for another
    EDIT_TRANSPOSE = 3,  //!< swap two characters
    EDIT_FLIP_CASE = 4,  //!< invert the case of an alpha character
};

template<typename SeqT>
static
size_t choosePosition(const SeqT &corpus, CorpusProperties &props) {
    assert(!corpus.empty());
    unsigned pos = props.rand(0, corpus.size() - 1);
    return pos;
}

class CorpusEditor {
public:
    CorpusEditor(CorpusProperties &p) : props(p) {}

    // Apply edits to a corpus
    void applyEdits(string &corpus);

private:
    // operations
    void insert(string &corpus);
    void remove(string &corpus);
    void substitute(string &corpus);
    void transpose(string &corpus);
    void flip_case(string &corpus);

    Operation chooseOperation();
    u8 chooseByte();

    CorpusProperties &props;
};

Operation CorpusEditor::chooseOperation() {
    return (Operation)props.rand(EDIT_INSERT, EDIT_FLIP_CASE);
}

void CorpusEditor::applyEdits(string &corpus) {
    for (size_t i = 0; i != props.editDistance; i++) {
        Operation op = chooseOperation();
        switch (op) {
            case EDIT_INSERT:
                insert(corpus);
                break;
            case EDIT_REMOVE:
                remove(corpus);
                break;
            case EDIT_SUBSTITUTE:
                substitute(corpus);
                break;
            case EDIT_TRANSPOSE:
                transpose(corpus);
                break;
            case EDIT_FLIP_CASE:
                flip_case(corpus);
                break;
        }
    }
}

void CorpusEditor::insert(string &corpus) {
    unsigned pos = props.rand(0, corpus.size());
    u8 c = chooseByte();
    corpus.insert(pos, 1, (char)c);
}

void CorpusEditor::remove(string &corpus) {
    if (corpus.empty()) return;
    size_t pos = choosePosition(corpus, props);
    corpus.erase(pos, 1);
}

void CorpusEditor::substitute(string &corpus) {
    if (corpus.empty()) return;
    size_t pos = choosePosition(corpus, props);
    corpus[pos] = chooseByte();
}

void CorpusEditor::transpose(string &corpus) {
    if (corpus.empty()) return;
    size_t a = choosePosition(corpus, props);
    size_t b = choosePosition(corpus, props);
    u8 tmp = corpus[a];
    corpus[a] = corpus[b];
    corpus[b] = tmp;
}

void CorpusEditor::flip_case(string &corpus) {
    if (corpus.empty()) return;

    // Pick a random starting position and walk forward (wrapping at the end)
    // until we find an alpha character.
    const size_t len = corpus.size();
    const size_t pos = choosePosition(corpus, props);

    size_t i = pos;
    for (;;) {
        char c = corpus[i];
        if (ourisalpha(c)) {
            char upper = mytoupper(c), lower = mytolower(c);
            corpus[i] = c == upper ? lower : upper;
            DEBUG_PRINTF("flipped c=%c to %c\n", c, corpus[i]);
            return;
        }
        if (++i == len) {
            i = 0;
        }
        if (i == pos) { // wrapped, no alpha characters
            break;
        }
    }
}

u8 CorpusEditor::chooseByte() {
    return (u8)props.rand(0, 255);
}

class CorpusEditorUtf8 {
public:
    CorpusEditorUtf8(CorpusProperties &p) : props(p) {}

    // Apply edits to a corpus.
    void applyEdits(vector<unichar> &corpus);

private:
    // operations
    void insert(vector<unichar> &corpus);
    void remove(vector<unichar> &corpus);
    void substitute(vector<unichar> &corpus);
    void transpose(vector<unichar> &corpus);
    void flip_case(vector<unichar> &corpus);

    Operation chooseOperation();
    unichar chooseCodePoint();

    CorpusProperties &props;
};

Operation CorpusEditorUtf8::chooseOperation() {
    return (Operation)props.rand(EDIT_INSERT, EDIT_FLIP_CASE);
}

void CorpusEditorUtf8::applyEdits(vector<unichar> &corpus) {
    for (size_t i = 0; i != props.editDistance; i++) {
        Operation op = chooseOperation();
        switch (op) {
            case EDIT_INSERT:
                insert(corpus);
                break;
            case EDIT_REMOVE:
                remove(corpus);
                break;
            case EDIT_SUBSTITUTE:
                substitute(corpus);
                break;
            case EDIT_TRANSPOSE:
                transpose(corpus);
                break;
            case EDIT_FLIP_CASE:
                flip_case(corpus);
                break;
        }
    }
}

void CorpusEditorUtf8::insert(vector<unichar> &corpus) {
    unsigned pos = props.rand(0, corpus.size());
    corpus.insert(corpus.begin() + pos, chooseCodePoint());
}

void CorpusEditorUtf8::remove(vector<unichar> &corpus) {
    if (corpus.empty()) return;
    size_t pos = choosePosition(corpus, props);
    corpus.erase(corpus.begin() + pos);
}

void CorpusEditorUtf8::substitute(vector<unichar> &corpus) {
    if (corpus.empty()) return;
    size_t pos = choosePosition(corpus, props);
    corpus[pos] = chooseCodePoint();
}

void CorpusEditorUtf8::transpose(vector<unichar> &corpus) {
    if (corpus.empty()) return;
    size_t a = choosePosition(corpus, props);
    size_t b = choosePosition(corpus, props);
    unichar tmp = corpus[a];
    corpus[a] = corpus[b];
    corpus[b] = tmp;
}

void CorpusEditorUtf8::flip_case(vector<unichar> &corpus) {
    if (corpus.empty()) return;

    // Pick a random starting position and walk forward (wrapping at the end)
    // until we find an alpha character.
    const size_t len = corpus.size();
    const size_t pos = choosePosition(corpus, props);

    size_t i = pos;
    for (;;) {
        if (::flip_case(&corpus[i])) {
            return;
        }
        if (++i == len) {
            i = 0;
        }
        if (i == pos) { // wrapped, no alpha characters
            break;
        }
    }
}

unichar CorpusEditorUtf8::chooseCodePoint(void) {
    /* We need to ensure that we don't pick a surrogate cp */
    const u32 range =
        MAX_UNICODE + 1 - (UNICODE_SURROGATE_MAX + UNICODE_SURROGATE_MIN + 1);
    unichar raw = props.rand(0, range - 1);
    if (raw < UNICODE_SURROGATE_MIN) {
        return raw;
    } else {
        return raw + UNICODE_SURROGATE_MAX + 1;
    }
}

} // namespace

void editCorpus(string *corpus, CorpusProperties &props) {
    CorpusEditor ed(props);
    ed.applyEdits(*corpus);
}

void editCorpus(vector<unichar> *corpus, CorpusProperties &props) {
    CorpusEditorUtf8 ed(props);
    ed.applyEdits(*corpus);
}
