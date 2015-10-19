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
 * \brief Visitor base class for working with the component tree.
 */

#ifndef CONSTCOMPONENTVISITOR_H
#define CONSTCOMPONENTVISITOR_H

namespace ue2 {

class AsciiComponentClass;
class Component;
class ComponentAlternation;
class ComponentAssertion;
class ComponentAtomicGroup;
class ComponentBackReference;
class ComponentBoundary;
class ComponentByte;
class ComponentCondReference;
class ComponentClass;
class ComponentEmpty;
class ComponentEUS;
class ComponentRepeat;
class ComponentSequence;
class ComponentWordBoundary;
class UTF8ComponentClass;

/**
 * \brief Visitor base class for traversing an immutable component tree.
 *
 * Our approach to implementing the visitor pattern for traversing the
 * Component tree for a pattern. This version operates on an immutable tree;
 * use \ref ComponentVisitor if you need to make changes to components during
 * traversal.
 */
class ConstComponentVisitor {
public:
    virtual ~ConstComponentVisitor();

    virtual void pre(const AsciiComponentClass &c) = 0;
    virtual void pre(const ComponentAlternation &c) = 0;
    virtual void pre(const ComponentAssertion &c) = 0;
    virtual void pre(const ComponentAtomicGroup &c) = 0;
    virtual void pre(const ComponentBackReference &c) = 0;
    virtual void pre(const ComponentBoundary &c) = 0;
    virtual void pre(const ComponentByte &c) = 0;
    virtual void pre(const ComponentCondReference &c) = 0;
    virtual void pre(const ComponentEmpty &c) = 0;
    virtual void pre(const ComponentEUS &c) = 0;
    virtual void pre(const ComponentRepeat &c) = 0;
    virtual void pre(const ComponentSequence &c) = 0;
    virtual void pre(const ComponentWordBoundary &c) = 0;
    virtual void pre(const UTF8ComponentClass &c) = 0;

    virtual void during(const AsciiComponentClass &c) = 0;
    virtual void during(const ComponentAlternation &c) = 0;
    virtual void during(const ComponentAssertion &c) = 0;
    virtual void during(const ComponentAtomicGroup &c) = 0;
    virtual void during(const ComponentBackReference &c) = 0;
    virtual void during(const ComponentBoundary &c) = 0;
    virtual void during(const ComponentByte &c) = 0;
    virtual void during(const ComponentCondReference &c) = 0;
    virtual void during(const ComponentEmpty &c) = 0;
    virtual void during(const ComponentEUS &c) = 0;
    virtual void during(const ComponentRepeat &c) = 0;
    virtual void during(const ComponentSequence &c) = 0;
    virtual void during(const ComponentWordBoundary &c) = 0;
    virtual void during(const UTF8ComponentClass &c) = 0;

    virtual void post(const AsciiComponentClass &c) = 0;
    virtual void post(const ComponentAlternation &c) = 0;
    virtual void post(const ComponentAssertion &c) = 0;
    virtual void post(const ComponentAtomicGroup &c) = 0;
    virtual void post(const ComponentBackReference &c) = 0;
    virtual void post(const ComponentBoundary &c) = 0;
    virtual void post(const ComponentByte &c) = 0;
    virtual void post(const ComponentCondReference &c) = 0;
    virtual void post(const ComponentEmpty &c) = 0;
    virtual void post(const ComponentEUS &c) = 0;
    virtual void post(const ComponentRepeat &c) = 0;
    virtual void post(const ComponentSequence &c) = 0;
    virtual void post(const ComponentWordBoundary &c) = 0;
    virtual void post(const UTF8ComponentClass &c) = 0;
};

/**
 * \brief Concrete subclass of ConstComponentVisitor with default behaviour,
 * allowing you to just implement the member functions you need.
 */
class DefaultConstComponentVisitor : public ConstComponentVisitor {
public:
    DefaultConstComponentVisitor();
    ~DefaultConstComponentVisitor() override;

    void pre(const AsciiComponentClass &c) override;
    void pre(const ComponentAlternation &c) override;
    void pre(const ComponentAssertion &c) override;
    void pre(const ComponentAtomicGroup &c) override;
    void pre(const ComponentBackReference &c) override;
    void pre(const ComponentBoundary &c) override;
    void pre(const ComponentByte &c) override;
    void pre(const ComponentCondReference &c) override;
    void pre(const ComponentEmpty &c) override;
    void pre(const ComponentEUS &c) override;
    void pre(const ComponentRepeat &c) override;
    void pre(const ComponentSequence &c) override;
    void pre(const ComponentWordBoundary &c) override;
    void pre(const UTF8ComponentClass &c) override;

    void during(const AsciiComponentClass &c) override;
    void during(const ComponentAlternation &c) override;
    void during(const ComponentAssertion &c) override;
    void during(const ComponentAtomicGroup &c) override;
    void during(const ComponentBackReference &c) override;
    void during(const ComponentBoundary &c) override;
    void during(const ComponentByte &c) override;
    void during(const ComponentCondReference &c) override;
    void during(const ComponentEmpty &c) override;
    void during(const ComponentEUS &c) override;
    void during(const ComponentRepeat &c) override;
    void during(const ComponentSequence &c) override;
    void during(const ComponentWordBoundary &c) override;
    void during(const UTF8ComponentClass &c) override;

    void post(const AsciiComponentClass &c) override;
    void post(const ComponentAlternation &c) override;
    void post(const ComponentAssertion &c) override;
    void post(const ComponentAtomicGroup &c) override;
    void post(const ComponentBackReference &c) override;
    void post(const ComponentBoundary &c) override;
    void post(const ComponentByte &c) override;
    void post(const ComponentCondReference &c) override;
    void post(const ComponentEmpty &c) override;
    void post(const ComponentEUS &c) override;
    void post(const ComponentRepeat &c) override;
    void post(const ComponentSequence &c) override;
    void post(const ComponentWordBoundary &c) override;
    void post(const UTF8ComponentClass &c) override;
};

} // namespace ue2

#endif // CONSTCOMPONENTVISITOR_H
