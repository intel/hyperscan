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

#ifndef COMPONENTVISITOR_H
#define COMPONENTVISITOR_H

namespace ue2 {

class AsciiComponentClass;
class Component;
class ComponentAlternation;
class ComponentAssertion;
class ComponentAtomicGroup;
class ComponentBackReference;
class ComponentBoundary;
class ComponentByte;
class ComponentClass;
class ComponentCondReference;
class ComponentEmpty;
class ComponentEUS;
class ComponentRepeat;
class ComponentSequence;
class ComponentWordBoundary;
class UTF8ComponentClass;

/**
 * \brief Visitor base class for working with the component tree.
 *
 * Our approach to implementing the visitor pattern for traversing (and
 * optionally mutating) the Component tree for a pattern. Each _visit_ function
 * takes a Component subclass pointer in and returns a Component pointer. That
 * pointer can have several values, dictating what the containing Component
 * should do:
 *
 * 1. If ptr == c, then do nothing.
 * 2. If ptr == nullptr, then remove c from the tree.
 * 3. If ptr != c && ptr != nullptr, then replace c with ptr.
 *
 * Traversal order is pre-order.
 *
 * After a Component's subcomponents have been visited, the _post_ function for
 * that Component will be called.
 */
class ComponentVisitor {
public:
    virtual ~ComponentVisitor();

    virtual Component *visit(AsciiComponentClass *c) = 0;
    virtual Component *visit(ComponentAlternation *c) = 0;
    virtual Component *visit(ComponentAssertion *c) = 0;
    virtual Component *visit(ComponentAtomicGroup *c) = 0;
    virtual Component *visit(ComponentBackReference *c) = 0;
    virtual Component *visit(ComponentBoundary *c) = 0;
    virtual Component *visit(ComponentByte *c) = 0;
    virtual Component *visit(ComponentCondReference *c) = 0;
    virtual Component *visit(ComponentEmpty *c) = 0;
    virtual Component *visit(ComponentEUS *c) = 0;
    virtual Component *visit(ComponentRepeat *c) = 0;
    virtual Component *visit(ComponentSequence *c) = 0;
    virtual Component *visit(ComponentWordBoundary *c) = 0;
    virtual Component *visit(UTF8ComponentClass *c) = 0;

    virtual void post(AsciiComponentClass *c) = 0;
    virtual void post(ComponentAlternation *c) = 0;
    virtual void post(ComponentAssertion *c) = 0;
    virtual void post(ComponentAtomicGroup *c) = 0;
    virtual void post(ComponentBackReference *c) = 0;
    virtual void post(ComponentBoundary *c) = 0;
    virtual void post(ComponentByte *c) = 0;
    virtual void post(ComponentCondReference *c) = 0;
    virtual void post(ComponentEmpty *c) = 0;
    virtual void post(ComponentEUS *c) = 0;
    virtual void post(ComponentRepeat *c) = 0;
    virtual void post(ComponentSequence *c) = 0;
    virtual void post(ComponentWordBoundary *c) = 0;
    virtual void post(UTF8ComponentClass *c) = 0;
};

/**
 * \brief Concrete subclass of ComponentVisitor with default behaviour,
 * allowing you to just implement the member functions you need.
 */
class DefaultComponentVisitor : public ComponentVisitor {
public:
    DefaultComponentVisitor();
    ~DefaultComponentVisitor() override;

    Component *visit(AsciiComponentClass *c) override;
    Component *visit(ComponentAlternation *c) override;
    Component *visit(ComponentAssertion *c) override;
    Component *visit(ComponentAtomicGroup *c) override;
    Component *visit(ComponentBackReference *c) override;
    Component *visit(ComponentBoundary *c) override;
    Component *visit(ComponentByte *c) override;
    Component *visit(ComponentCondReference *c) override;
    Component *visit(ComponentEmpty *c) override;
    Component *visit(ComponentEUS *c) override;
    Component *visit(ComponentRepeat *c) override;
    Component *visit(ComponentSequence *c) override;
    Component *visit(ComponentWordBoundary *c) override;
    Component *visit(UTF8ComponentClass *c) override;

    void post(AsciiComponentClass *c) override;
    void post(ComponentAlternation *c) override;
    void post(ComponentAssertion *c) override;
    void post(ComponentAtomicGroup *c) override;
    void post(ComponentBackReference *c) override;
    void post(ComponentBoundary *c) override;
    void post(ComponentByte *c) override;
    void post(ComponentCondReference *c) override;
    void post(ComponentEmpty *c) override;
    void post(ComponentEUS *c) override;
    void post(ComponentRepeat *c) override;
    void post(ComponentSequence *c) override;
    void post(ComponentWordBoundary *c) override;
    void post(UTF8ComponentClass *c) override;
};

} // namespace ue2

#endif // COMPONENTVISITOR_H
