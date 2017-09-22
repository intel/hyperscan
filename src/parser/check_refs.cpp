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

/** \file
 * \brief Component tree analysis that checks that references (such as
 * back-refs, conditionals) have valid referents.
 */
#include "check_refs.h"
#include "ComponentBackReference.h"
#include "ComponentCondReference.h"
#include "ConstComponentVisitor.h"
#include "parse_error.h"
#include "util/container.h"
#include "util/flat_containers.h"

#include <sstream>

using namespace std;

namespace ue2 {

/**
 * \brief Visitor that checks the validity of references against a known list
 * of indices and labels.
 */
class ReferenceVisitor: public DefaultConstComponentVisitor {
private:
    const size_t num_ids;
    const flat_set<string> &names;

public:
    ReferenceVisitor(size_t num_groups, const flat_set<string> &targets)
        : num_ids(num_groups), names(targets) {}

    ~ReferenceVisitor() override;

    void invalid_index(const char *component, unsigned id) {
        assert(component);
        ostringstream str;
        str << "Invalid " << component << " to expression " << id << ".";
        throw ParseError(str.str());
    }

    void invalid_label(const char *component, const std::string &label) {
        assert(component);
        ostringstream str;
        str << "Invalid " << component << " to label '" << label << "'.";
        throw ParseError(str.str());
    }

    using DefaultConstComponentVisitor::pre;

    void pre(const ComponentBackReference &c) override {
        if (c.ref_id) {
            if (c.ref_id >= num_ids) {
                invalid_index("back reference", c.ref_id);
            }
        } else {
            if (!contains(names, c.name)) {
                invalid_label("back reference", c.name);
            }
        }
    }

    void pre(const ComponentCondReference &c) override {
        switch (c.kind) {
        case ComponentCondReference::CONDITION_NUMBER:
            if (c.ref_id >= num_ids) {
                invalid_index("conditional reference", c.ref_id);
            }
            break;
        case ComponentCondReference::CONDITION_NAME:
            if (c.ref_name == "DEFINE") {
                // The string "DEFINE" is a special "always false" condition
                // used to define subroutines.
                break;
            }
            if (!contains(names, c.ref_name)) {
                invalid_label("conditional reference", c.ref_name);
            }
            break;
        case ComponentCondReference::CONDITION_ASSERTION:
            break;
        }
    }
};

// Out-of-line destructor to silence weak vtable warnings.
ReferenceVisitor::~ReferenceVisitor() {}

void checkReferences(const Component &root, unsigned int groupIndices,
                     const flat_set<std::string> &groupNames) {
    ReferenceVisitor vis(groupIndices, groupNames);
    root.accept(vis);
}

} // namespace ue2
