.. _runtime:

#####################
Scanning for Patterns
#####################

Hyperscan provides three different scanning modes, each with its own scan
function beginning with ``hs_scan``. In addition, streaming mode has a number
of other API functions for managing stream state.

****************
Handling Matches
****************

All of these functions will call a user-supplied callback function when a match
is found. This function has the following signature:

  .. doxygentypedef:: match_event_handler
     :outline:
     :no-link:

The *id* argument will be set to the identifier for the matching expression
provided at compile time, and the *to* argument will be set to the end-offset
of the match. If SOM was requested for the pattern (see :ref:`som`), the
*from* argument will be set to the leftmost possible start-offset for the match.

The match callback function has the capability to halt scanning
by returning a non-zero value.

See :c:type:`match_event_handler` for more information.

**************
Streaming Mode
**************

The streaming runtime API consists of functions to open, scan, and close
Hyperscan data streams -- these functions being :c:func:`hs_open_stream`,
:c:func:`hs_scan_stream`, and :c:func:`hs_close_stream`. Any matches detected
in the written data are returned to the calling application via a function
pointer callback.

The match callback function has the capability to halt scanning of the current
data stream by returning a non-zero value. In streaming mode, the result of
this is that the stream is then left in a state where no more data can be
scanned, and any subsequent calls to :c:func:`hs_scan_stream` for that stream
will return immediately with :c:member:`HS_SCAN_TERMINATED`. The caller must
still call :c:func:`hs_close_stream` to complete the clean-up process for that
stream.

Streams exist in the Hyperscan library so that pattern matching state can be
maintained across multiple blocks of target data -- without maintaining this
state, it would not be possible to detect patterns that span these blocks of
data. This, however, does come at the cost of requiring an amount of storage
per-stream (the size of this storage is fixed at compile time), and a slight
performance penalty in some cases to manage the state.

While Hyperscan does always support a strict ordering of multiple matches,
streaming matches will not be delivered at offsets before the current stream
write, with the exception of zero-width asserts, where constructs such as
:regexp:`\\b` and :regexp:`$` can cause a match on the final character of a
stream write to be delayed until the next stream write or stream close
operation.

=================
Stream Management
=================

In addition to :c:func:`hs_open_stream`, :c:func:`hs_scan_stream`, and
:c:func:`hs_close_stream`, the Hyperscan API provides a number of other
functions for the management of streams:

* :c:func:`hs_reset_stream`: resets a stream to its initial state; this is
  equivalent to calling :c:func:`hs_close_stream` but will not free the memory
  used for stream state.

* :c:func:`hs_copy_stream`: constructs a (newly allocated) duplicate of a
  stream.

* :c:func:`hs_reset_and_copy_stream`: constructs a duplicate of a stream into
  another, resetting the destination stream first. This call avoids the
  allocation done by :c:func:`hs_copy_stream`.

**********
Block Mode
**********

The block mode runtime API consists of a single function: :c:func:`hs_scan`. Using
the compiled patterns this function identifies matches in the target data,
using a function pointer callback to communicate with the application.

This single :c:func:`hs_scan` function is essentially equivalent to calling
:c:func:`hs_open_stream`, making a single call to :c:func:`hs_scan_stream`, and
then :c:func:`hs_close_stream`, except that block mode operation does not
incur all the stream related overhead.

*************
Vectored Mode
*************

The vectored mode runtime API, like the block mode API, consists of a single
function: :c:func:`hs_scan_vector`. This function accepts an array of data
pointers and lengths, facilitating the scanning in sequence of a set of data
blocks that are not contiguous in memory.

From the caller's perspective, this mode will produce the same matches as if
the set of data blocks were (a) scanned in sequence with a series of streaming
mode scans, or (b) copied in sequence into a single block of memory and then
scanned in block mode.

*************
Scratch Space
*************

While scanning data, Hyperscan needs a small amount of temporary memory to store
on-the-fly internal data. This amount is unfortunately too large to fit on the
stack, particularly for embedded applications, and allocating memory dynamically
is too expensive, so a pre-allocated "scratch" space must be provided to the
scanning functions.

The function :c:func:`hs_alloc_scratch` allocates a large enough region of
scratch space to support a given database. If the application uses multiple
databases, only a single scratch region is necessary: in this case, calling
:c:func:`hs_alloc_scratch` on each database (with the same ``scratch`` pointer)
will ensure that the scratch space is large enough to support scanning against
any of the given databases.

While the Hyperscan library is re-entrant, the use of scratch spaces is not.
For example, if by design it is deemed necessary to run recursive or nested
scanning (say, from the match callback function), then an additional scratch
space is required for that context.

In the absence of recursive scanning, only one such space is required per thread
and can (and indeed should) be allocated before data scanning is to commence.

In a scenario where a set of expressions are compiled by a single "master"
thread and data will be scanned by multiple "worker" threads, the convenience
function :c:func:`hs_clone_scratch` allows multiple copies of an existing
scratch space to be made for each thread (rather than forcing the caller to pass
all the compiled databases through :c:func:`hs_alloc_scratch` multiple times).

For example:

.. code-block:: c

    hs_error_t err;
    hs_scratch_t *scratch_prototype = NULL;
    err = hs_alloc_scratch(db, &scratch_prototype);
    if (err != HS_SUCCESS) {
        printf("hs_alloc_scratch failed!");
        exit(1);
    }

    hs_scratch_t *scratch_thread1 = NULL;
    hs_scratch_t *scratch_thread2 = NULL;

    err = hs_clone_scratch(scratch_prototype, &scratch_thread1);
    if (err != HS_SUCCESS) {
        printf("hs_clone_scratch failed!");
        exit(1);
    }
    err = hs_clone_scratch(scratch_prototype, &scratch_thread2);
    if (err != HS_SUCCESS) {
        printf("hs_clone_scratch failed!");
        exit(1);
    }

    hs_free_scratch(scratch_prototype);

    /* Now two threads can both scan against database db,
       each with its own scratch space. */

*****************
Custom Allocators
*****************

By default, structures used by Hyperscan at runtime (scratch space, stream
state, etc) are allocated with the default system allocators, usually
``malloc()`` and ``free()``.

The Hyperscan API provides a facility for changing this behaviour to support
applications that use custom memory allocators.

These functions are:

- :c:func:`hs_set_database_allocator`, which sets the allocate and free functions
  used for compiled pattern databases.
- :c:func:`hs_set_scratch_allocator`, which sets the allocate and free
  functions used for scratch space.
- :c:func:`hs_set_stream_allocator`, which sets the allocate and free functions
  used for stream state in streaming mode.
- :c:func:`hs_set_misc_allocator`, which sets the allocate and free functions
  used for miscellaneous data, such as compile error structures and
  informational strings.

The :c:func:`hs_set_allocator` function can be used to set all of the custom
allocators to the same allocate/free pair.
