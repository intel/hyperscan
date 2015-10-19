Hyperscan Example Code
======================

Copyright (C) 2015 Intel Corporation. All rights reserved.

The files in this directory contain example code demonstrating the use of the
Hyperscan regular expression matching library. The examples have been
constructed to be useful utility programs, but they have been simplified
somewhat, so generally contain "shortcuts" that one would not take if building
a "real" system.

The examples each contain a short description in a comment at the top of the
file, including build instructions.

---


Example 1: simplegrep
---------------------

The first example program (`simplegrep.c`) is modelled on the ubiquitous grep
tool to search a file for a single regular expression. 'simplegrep' does the
same, but eschews a lot of grep's complexity: it is unable to read data from
`stdin`, and doesn't support grep's plethora of command-line arguments.

This code is intended to be simple portable C99.

simplegrep demonstrates the following Hyperscan concepts:

- Single pattern compilation: As simplegrep can scan for one pattern only, it
  uses the `hs_compile` function instead of the multi-pattern variant:
  `hs_compile_multi`.

- Block mode pattern-matching: simplegrep will search a single data buffer
  for the given pattern, so it has no need to set up and tear down streams.
  (See the next section for a streaming mode example)

- Scratch space allocation and use: Hyperscan requires a small amount of
  temporary memory that is used in the `hs_scan` call. The caller needs to
  guarantee that only one instance of `hs_scan` is using the scratch space at a
  time, but there is no requirement that the same scratch area be used on
  consecutive calls to `hs_scan`. Given that it is expensive to allocate the
  scratch space, one would typically allocate all necessary scratch space at
  system startup and reuse it throughout execution of the program.


Example 2: pcapscan
-------------------

The second example program (`pcapscan.cc`) is a very simple packet scanning
benchmark. It scans a given PCAP file full of network traffic against a group
of regular expressions and returns some coarse performance measurements.  This
example provides a quick way to examine the performance achievable on a
particular combination of platform, pattern set and input data.

In block mode, pcapscan scans each packet individually against a Hyperscan
database. In streaming mode, pcapscan assigns packets to flows using a
rudimentary connection tracker, then scans the packets in each flow with
Hyperscan's streaming mode interface. This demonstrates the use of streaming
mode operation to detect matches that straddle packet boundaries.

**Note**: the flow assignment implemented here is intended as a simple demo; it
merely ensures that packets with the same 5-tuple are written to the same
stream in the order in which they appear in the PCAP file.  No packet
re-ordering or connection state tracking (as you would expect to find in a real
network scanning application) is done.

pcapscan introduces the following Hyperscan concepts:

- Multi-pattern compilation: Unlike simplegrep, pcapscan requires a file of
  expressions as input instead of a single pattern. pcapscan will read this
  file in, one pattern per line, and use it as input to the `hs_compile_multi`
  function. This function generates a pattern database that will match all the
  input patterns in parallel.

- Streamed pattern-matching: pcapscan uses the `hs_scan_stream` function
  (instead of the block-mode `hs_scan` call) to allow it to identify matches
  that occur in a stream of data, even if they straddle the boundaries between blocks.
  Streaming mode operation has a number of unique properties:

  - Stream state that persists for the lifetime of the stream must be allocated
    with the `hs_open_stream` function before scanning can take place.
    Similarly, it must be freed with `hs_close_stream` after it is no longer
    needed. Each stream being scanned concurrently requires its own stream
    state.

  - In streaming mode, a non-zero return from the user-specified event-handler
    function has consequences for the rest of that stream's lifetime: when a
    non-zero return occurs, it signals that no more of the stream should be
    scanned. Consequently if the user makes a subsequent call to
    `hs_scan_stream` on a stream whose processing was terminated in this way,
    hs_scan_stream will return `HS_SCAN_TERMINATED`. This case has not been
    demonstrated in pcapscan, as its callback always returns 0.

  - Match handling during stream shutdown: As matches may occur when the
    `hs_close_stream` function is called, it too must be provided with scratch
    space in order to perform this match processing. Similarly, the user must
    be prepared to be issued match event callbacks during the `hs_close_stream`
    call. For this reason, we advise that stream shutdown be an integral part
    of the system design.


Example 3: patbench
-------------------

This program allows users to detect which signatures may be the most expensive
in a set of patterns. It is designed for use with small to medium pattern set
sizes (e.g. 5-500). If used with very large pattern sets it may take a very
long time - the number of recompiles done is `g * O(lg2(n))` where `g` is the
number of generations and `n` is the number of patterns (assuming that `n >>
g`).

This utility will return a cumulative series of removed patterns. The first
generation will find and remove a single pattern. The second generation will
begin with the first pattern removed and find another pattern to remove, etc.
So if we have 100 patterns and 15 generations, the final generation's score
will be a run over 85 patterns.

This utility is probabilistic. It is possible that the pattern removed in a
generation is not a particularly expensive pattern. To reduce noise in the
results use 'taskset' and set the number of repeats to a level that still
completes in reasonable time (this will reduce the effect of random measurement
noise).

The criterion for performance can be altered by use of the `-C<x>` flag where
`<x>` can be `t,r,s,c,b`, selecting pattern matching throughput, scratch size,
stream state size (only available in streaming mode), compile time and bytecode
size respectively.

This utility will also not produce good results if all the patterns are roughly
equally expensive.

### Factor Group Size:

If there are multiple expensive patterns that are very similar on the
left-hand-side or identical, this utility will typically not find these groups
unless the `-F` flag is used to search for a group size that is equal to or
larger than the size of the group of similar patterns.

Otherwise, removing a portion of the similar patterns will have no or almost no
effect, and the search procedure used relies on the ability to remove all of
the similar patterns in at least one search case, something which will only
happen if the `factor_group_size` is large enough.

This alters the operation of the tool so that instead of trying to find the
single pattern whose removal has the most effect by binary search (the default
with `factor_group_size == 1`), we attempt to find the N patterns whose removal
has the most effect by searching over `N + 1` evenly sized groups, removing
only `1/(N + 1)` of the search signatures per iteration.

Note that the number of recompiles done greatly increases with increased factor
group size.  For example, with `factor_group_size = 1`, we do `g * 2 * lg2(n)`
recompiles, while with `factor_group_size = 4`, we do `g * 4 * log(5/4)(n)`.
Informally the number of generations we require goes up as we eliminate a
smaller number of signatures and the we have to do more work per generation.
