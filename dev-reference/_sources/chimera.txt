.. _chimera:

#######
Chimera
#######

This section describes Chimera library.

************
Introduction
************

Chimera is a software regular expression matching engine that is a hybrid of
Hyperscan and PCRE. The design goals of Chimera are to fully support PCRE
syntax as well as to take advantage of the high performance nature of Hyperscan.

Chimera inherits the design guideline of Hyperscan with C APIs for compilation
and scanning.

The Chimera API itself is composed of two major components:

===========
Compilation
===========

These functions take a group of regular expressions, along with identifiers and
option flags, and compile them into an immutable database that can be used by
the Chimera scanning API. This compilation process performs considerable
analysis and optimization work in order to build a database that will match
the given expressions efficiently.

See :ref:`chcompile` for more details

========
Scanning
========

Once a Chimera database has been created, it can be used to scan data in memory.
Chimera only supports block mode in which we scan a single contiguous block in
memory.

Matches are delivered to the application via a user-supplied callback function
that is called synchronously for each match.

For a given database, Chimera provides several guarantees:

* No memory allocations occur at runtime with the exception of scratch space
  allocation, it should be done ahead of time for performance-critical
  applications:

  - **Scratch space**: temporary memory used for internal data at scan time.
    Structures in scratch space do not persist beyond the end of a single scan
    call.

* The size of the scratch space required for a given database is fixed and
  determined at database compile time. This means that the memory requirement
  of the application are known ahead of time, and the scratch space can be
  pre-allocated if required for performance reasons.

* Any pattern that has successfully been compiled by the Chimera compiler can
  be scanned against any input. There could be internal resource limits or
  other limitations caused by PCRE at runtime that could cause a scan call to
  return an error.

.. note:: Chimera is designed to have the same matching behavior as PCRE,
   including greedy/ungreedy, capturing, etc. Chimera reports both
   **start offset** and **end offset** for each match like PCRE. Different
   from the fashion of reporting all matches in Hyperscan, Chimera only reports
   non-overlapping matches. For example, the pattern :regexp:`/foofoo/` will
   match ``foofoofoofoo`` at offsets (0, 6) and (6, 12).

.. note:: Since Chimera is a hybrid of Hyperscan and PCRE in order to support
   full PCRE syntax, there will be extra performance overhead compared to
   Hyperscan-only solution. Please always use Hyperscan for better performance
   unless you must need full PCRE syntax support.

See :ref:`chruntime` for more details

************
Requirements
************

The PCRE library (http://pcre.org/) version 8.41 is required for Chimera.

.. note:: Since Chimera needs to reference PCRE internal function, please place PCRE source
   directory under Hyperscan root directory in order to build Chimera.

Beside this, both hardware and software requirements of Chimera are the same to Hyperscan.
See :ref:`hardware` and :ref:`software` for more details.

.. note:: Building Hyperscan will automatically generate Chimera library.
   Currently only static library is supported for Chimera, so please
   use static build type when configure CMake build options.

.. _chcompile:

******************
Compiling Patterns
******************

===================
Building a Database
===================

The Chimera compiler API accepts regular expressions and converts them into a
compiled pattern database that can then be used to scan data.

The API provides two functions that compile regular expressions into
databases:

#. :c:func:`ch_compile`: compiles a single expression into a pattern database.

#. :c:func:`ch_compile_multi`: compiles an array of expressions into a pattern
   database. All of the supplied patterns will be scanned for concurrently at
   scan time, with user-supplied identifiers returned when they match.

#. :c:func:`ch_compile_ext_multi`: compiles an array of expressions as above,
   but allows PCRE match limits to be specified for each expression.

Compilation allows the Chimera library to analyze the given pattern(s) and
pre-determine how to scan for these patterns in an optimized fashion using
Hyperscan and PCRE.

===============
Pattern Support
===============

Chimera fully supports the pattern syntax used by the PCRE library ("libpcre"),
described at <http://www.pcre.org/>.The version of PCRE used to validate
Chimera's interpretation of this syntax is 8.41.

=========
Semantics
=========

Chimera supports the exact same semantics of PCRE library. Moreover, it supports
multiple simultaneous pattern matching like Hyperscan and the multiple matches
will be reported in order by end offset.

.. _chruntime:

*********************
Scanning for Patterns
*********************

Chimera provides scan function with ``ch_scan``.

================
Handling Matches
================

``ch_scan`` will call a user-supplied callback function when a match
is found. This function has the following signature:

  .. doxygentypedef:: ch_match_event_handler
       :outline:
       :no-link:

The *id* argument will be set to the identifier for the matching expression
provided at compile time, and the *from* argument will be set to the
start-offset of the match the *to* argument will be set to the end-offset
of the match. The *captured* stores offsets of entire pattern match as well as
captured subexpressions. The *size* will be set to the number of valid entries in
the *captured*.

The match callback function has the capability to continue or halt scanning
by returning different values.

See :c:type:`ch_match_event_handler` for more information.

=======================
Handling Runtime Errors
=======================

``ch_scan`` will call a user-supplied callback function when a runtime error
occurs in libpcre. This function has the following signature:

  .. doxygentypedef:: ch_error_event_handler
       :outline:
       :no-link:

The *id* argument will be set to the identifier for the matching expression
provided at compile time.

The match callback function has the capability to either halt scanning or
continue scanning for the next pattern.

See :c:type:`ch_error_event_handler` for more information.

=============
Scratch Space
=============

While scanning data, Chimera needs a small amount of temporary memory to store
on-the-fly internal data. This amount is unfortunately too large to fit on the
stack, particularly for embedded applications, and allocating memory dynamically
is too expensive, so a pre-allocated "scratch" space must be provided to the
scanning functions.

The function :c:func:`ch_alloc_scratch` allocates a large enough region of
scratch space to support a given database. If the application uses multiple
databases, only a single scratch region is necessary: in this case, calling
:c:func:`ch_alloc_scratch` on each database (with the same ``scratch`` pointer)
will ensure that the scratch space is large enough to support scanning against
any of the given databases.

While the Chimera library is re-entrant, the use of scratch spaces is not.
For example, if by design it is deemed necessary to run recursive or nested
scanning (say, from the match callback function), then an additional scratch
space is required for that context.

In the absence of recursive scanning, only one such space is required per thread
and can (and indeed should) be allocated before data scanning is to commence.

In a scenario where a set of expressions are compiled by a single "master"
thread and data will be scanned by multiple "worker" threads, the convenience
function :c:func:`ch_clone_scratch` allows multiple copies of an existing
scratch space to be made for each thread (rather than forcing the caller to pass
all the compiled databases through :c:func:`ch_alloc_scratch` multiple times).

For example:

.. code-block:: c

    ch_error_t err;
    ch_scratch_t *scratch_prototype = NULL;
    err = ch_alloc_scratch(db, &scratch_prototype);
    if (err != CH_SUCCESS) {
        printf("ch_alloc_scratch failed!");
        exit(1);
    }

    ch_scratch_t *scratch_thread1 = NULL;
    ch_scratch_t *scratch_thread2 = NULL;

    err = ch_clone_scratch(scratch_prototype, &scratch_thread1);
    if (err != CH_SUCCESS) {
        printf("ch_clone_scratch failed!");
        exit(1);
    }
    err = ch_clone_scratch(scratch_prototype, &scratch_thread2);
    if (err != CH_SUCCESS) {
        printf("ch_clone_scratch failed!");
        exit(1);
    }

    ch_free_scratch(scratch_prototype);

    /* Now two threads can both scan against database db,
       each with its own scratch space. */


=================
Custom Allocators
=================

By default, structures used by Chimera at runtime (scratch space, etc) are
allocated with the default system allocators, usually
``malloc()`` and ``free()``.

The Chimera API provides a facility for changing this behaviour to support
applications that use custom memory allocators.

These functions are:

- :c:func:`ch_set_database_allocator`, which sets the allocate and free functions
  used for compiled pattern databases.
- :c:func:`ch_set_scratch_allocator`, which sets the allocate and free
  functions used for scratch space.
- :c:func:`ch_set_misc_allocator`, which sets the allocate and free functions
  used for miscellaneous data, such as compile error structures and
  informational strings.

The :c:func:`ch_set_allocator` function can be used to set all of the custom
allocators to the same allocate/free pair.


************************
API Reference: Constants
************************

===========
Error Codes
===========

.. doxygengroup:: CH_ERROR
   :content-only:
   :no-link:

=============
Pattern flags
=============

.. doxygengroup:: CH_PATTERN_FLAG
   :content-only:
   :no-link:

==================
Compile mode flags
==================

.. doxygengroup:: CH_MODE_FLAG
   :content-only:
   :no-link:


********************
API Reference: Files
********************

==========
File: ch.h
==========

.. doxygenfile:: ch.h

=================
File: ch_common.h
=================

.. doxygenfile:: ch_common.h

==================
File: ch_compile.h
==================

.. doxygenfile:: ch_compile.h

==================
File: ch_runtime.h
==================

.. doxygenfile:: ch_runtime.h
