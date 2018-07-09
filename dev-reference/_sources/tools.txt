.. _tools:

#####
Tools
#####

This section describes the set of utilities included with the Hyperscan library.

********************
Quick Check: hscheck
********************

The ``hscheck`` tool allows the user to quickly check whether Hyperscan supports
a group of patterns. If a pattern is rejected by Hyperscan's compiler, the
compile error is provided on standard output.

For example, given the following three patterns (the last of which contains a
syntax error) in a file called ``/tmp/test``::

    1:/foo.*bar/
    2:/abc|def|ghi/
    3:/((foo|bar)/

... the ``hscheck`` tool will produce the following output::

    $ bin/hscheck -e /tmp/test

    OK: 1:/foo.*bar/
    OK: 2:/abc|def|ghi/
    FAIL (compile): 3:/((foo|bar)/: Missing close parenthesis for group started at index 0.
    SUMMARY: 1 of 3 failed.

********************
Benchmarker: hsbench
********************

The ``hsbench`` tool provides an easy way to measure Hyperscan's performance
for a particular set of patterns and corpus of data to be scanned.

Patterns are supplied in the format described below in
:ref:`tools_pattern_format`, while the corpus must be provided in the form of a
`corpus database`: this is a simple SQLite database format intended to allow for
easy control of how a corpus is broken into blocks and streams.

.. note:: A group of Python scripts for constructing corpora databases from
   various input types, such as PCAP network traffic captures or text files, can
   be found in the Hyperscan source tree in ``tools/hsbench/scripts``.

Running hsbench
===============

Given a file full of patterns specified with ``-e`` and a corpus database
specified with ``-c``, ``hsbench`` will perform a single-threaded benchmark and
produce output like this::

    $ hsbench -e /tmp/patterns -c /tmp/corpus.db

    Signatures:        /tmp/patterns
    Hyperscan info:    Version: 4.3.1 Features:  AVX2 Mode: STREAM
    Expression count:  200
    Bytecode size:     342,540 bytes
    Database CRC:      0x6cd6b67c
    Stream state size: 252 bytes
    Scratch size:      18,406 bytes
    Compile time:      0.153 seconds
    Peak heap usage:   78,073,856 bytes

    Time spent scanning:     0.600 seconds
    Corpus size:             72,138,183 bytes (63,946 blocks in 8,891 streams)
    Scan matches:            81 (0.001 matches/kilobyte)
    Overall block rate:      2,132,004.45 blocks/sec
    Overall throughput:      19,241.10 Mbit/sec

By default, the corpus is scanned twenty times, and the overall performance
reported is computed based the total number of bytes scanned in the time it
takes to perform all twenty scans. The number of repeats can be changed with the
``-n`` argument, and the results of each scan will be displayed if the
``--per-scan`` argument is specified.

To benchmark Hyperscan on more than one core, you can supply a list of cores
with the ``-T`` argument, which will instruct ``hsbench`` to start one
benchmark thread per core given and compute the throughput from the time taken
to complete all of them.

.. tip:: For single-threaded benchmarks on multi-processor systems, we recommend
   using a utility like ``taskset`` to lock the hsbench process to one core and
   minimize jitter due to the operating system's scheduler.

*******************************
Correctness Testing: hscollider
*******************************

The ``hscollider`` tool, or Pattern Collider, provides a way to verify
Hyperscan's matching behaviour. It does this by compiling and scanning patterns
(either singly or in groups) against known corpora and comparing the results
against another engine (the "ground truth"). Two sources of ground truth for
comparison are available:

 * The PCRE library (http://pcre.org/).
 * An NFA simulation run on Hyperscan's compile-time graph representation. This
   is used if PCRE cannot support the pattern or if PCRE execution fails due to
   a resource limit.

Much of Hyperscan's testing infrastructure is built on ``hscollider``, and the
tool is designed to take advantage of multiple cores and provide considerable
flexibility in controlling the test. These options are described in the help
(``hscollider -h``) and include:

 * Testing in streaming, block or vectored mode.
 * Testing corpora at different alignments in memory.
 * Testing patterns in groups of varying size.
 * Manipulating stream state or scratch space between tests.
 * Cross-compilation and serialization/deserialization of databases.
 * Synthetic generation of corpora given a pattern set.

Using hscollider to debug a pattern
===================================

One common use-case for ``hscollider`` is to determine whether Hyperscan will
match a pattern in the expected location, and whether this accords with PCRE's
behaviour for the same case.

Here is an example. We put our pattern in a file in Hyperscan's pattern
format::

    $ cat /tmp/pat
    1:/hatstand.*badgerbrush/

We put the corpus to be scanned in another file, with the same numeric
identifier at the start to indicate that it should match pattern 1::

    $ cat /tmp/corpus
    1:__hatstand__hatstand__badgerbrush_badgerbrush

Then we can run ``hscollider`` with its verbosity turned up (``-vv``) so that
individual matches are displayed in the output::

    $ bin/ue2collider -e /tmp/pat -c /tmp/corpus -Z 0 -T 1 -vv
    ue2collider: The Pattern Collider Mark II

    Number of threads:  1 (1 scanner, 1 generator)
    Expression path:    /tmp/pat
    Signature files:    none
    Mode of operation:  block mode
    UE2 scan alignment: 0
    Corpora read from file: /tmp/corpus

    Running single-pattern/single-compile test for 1 expressions.

    PCRE Match @ (2,45)
    PCRE Match @ (2,33)
    PCRE Match @ (12,45)
    PCRE Match @ (12,33)
    UE2 Match @ (0,33) for 1
    UE2 Match @ (0,45) for 1
    Scan call returned 0
    PASSED: id 1, alignment 0, corpus 0 (matched pcre:2, ue2:2)
    Thread 0 processed 1 units.

    Summary:
    Mode:                           Single/Block
    =========
    Expressions processed:          1
    Corpora processed:              1
    Expressions with failures:      0
      Corpora generation failures:  0
      Compilation failures:         pcre:0, ng:0, ue2:0
      Matching failures:            pcre:0, ng:0, ue2:0
      Match differences:            0
      No ground truth:              0
    Total match differences:        0

    Total elapsed time: 0.00522815 secs.

We can see from this output that both PCRE and Hyperscan find matches ending at
offset 33 and 45, and so ``hscollider`` considers this test case to have
passed.

(In the example command line above, ``-Z 0`` instructs us to only test at
corpus alignment 0, and ``-T 1`` instructs us to only use one thread.)

.. note:: In default operation, PCRE produces only one match for a scan, unlike
  Hyperscan's automata semantics. The ``hscollider`` tool uses libpcre's
  "callout" functionality to match Hyperscan's semantics.

Running a larger scan test
==========================

A set of patterns for testing purposes are distributed with Hyperscan, and these
can be tested via ``hscollider`` on an in-tree build. Two CMake targets are
provided to do this easily:

================================= =====================================
Make Target                       Description
================================= =====================================
``make collide_quick_test``       Tests all patterns in streaming mode.
``make collide_quick_test_block`` Tests all patterns in block mode.
================================= =====================================

*****************
Debugging: hsdump
*****************

When built in debug mode (using the CMake directive ``CMAKE_BUILD_TYPE`` set to
``Debug``), Hyperscan includes support for dumping information about its
internals during pattern compilation with the ``hsdump`` tool.

This information is mostly of use to Hyperscan developers familiar with the
library's internal structure, but can be used to diagnose issues with patterns
and provide more information in bug reports.

.. _tools_pattern_format:

**************
Pattern Format
**************

All of the Hyperscan tools accept patterns in the same format, read from plain
text files with one pattern per line. Each line looks like this:

* ``<integer id>:/<regex>/<flags>``

For example::

    1:/hatstand.*teakettle/s
    2:/(hatstand|teakettle)/iH
    3:/^.{10,20}hatstand/m

The integer ID is the value that will be reported when a match is found by
Hyperscan and must be unique.

The pattern itself is a regular expression in PCRE syntax; see
:ref:`compilation` for more information on supported features.

The flags are single characters that map to Hyperscan flags as follows:

=========   =================================    ===========
Character   API Flag                             Description
=========   =================================    ===========
``i``       :c:member:`HS_FLAG_CASELESS`         Case-insensitive matching
``s``       :c:member:`HS_FLAG_DOTALL`           Dot (``.``) will match newlines
``m``       :c:member:`HS_FLAG_MULTILINE`        Multi-line anchoring
``H``       :c:member:`HS_FLAG_SINGLEMATCH`      Report match ID at most once
``V``       :c:member:`HS_FLAG_ALLOWEMPTY`       Allow patterns that can match against empty buffers
``8``       :c:member:`HS_FLAG_UTF8`             UTF-8 mode
``W``       :c:member:`HS_FLAG_UCP`              Unicode property support
``P``       :c:member:`HS_FLAG_PREFILTER`        Prefiltering mode
``L``       :c:member:`HS_FLAG_SOM_LEFTMOST`     Leftmost start of match reporting
``C``       :c:member:`HS_FLAG_COMBINATION`      Logical combination of patterns
``Q``       :c:member:`HS_FLAG_QUIET`            Quiet at matching
=========   =================================    ===========

In addition to the set of flags above, :ref:`extparam` can be supplied
for each pattern. These are supplied after the flags as ``key=value`` pairs
between braces, separated by commas. For example::

    1:/hatstand.*teakettle/s{min_offset=50,max_offset=100}

All Hyperscan tools will accept a pattern file (or a directory containing
pattern files) with the ``-e`` argument. If no further arguments constraining
the pattern set are given, all patterns in those files are used.

To select a subset of the patterns, a single ID can be supplied with the ``-z``
argument, or a file containing a set of IDs can be supplied with the ``-s``
argument.
