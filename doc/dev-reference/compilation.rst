.. include:: <isonum.txt>
.. _compilation:

##################
Compiling Patterns
##################

*******************
Building a Database
*******************

The Hyperscan compiler API accepts regular expressions and converts them into a
compiled pattern database that can then be used to scan data.

The API provides three functions that compile regular expressions into
databases:

#. :c:func:`hs_compile`: compiles a single expression into a pattern database.

#. :c:func:`hs_compile_multi`: compiles an array of expressions into a pattern
   database. All of the supplied patterns will be scanned for concurrently at
   scan time, with user-supplied identifiers returned when they match.

#. :c:func:`hs_compile_ext_multi`: compiles an array of expressions as above,
   but allows :ref:`extparam` to be specified for each expression.

Compilation allows the Hyperscan library to analyze the given pattern(s) and
pre-determine how to scan for these patterns in an optimized fashion that would
be far too expensive to compute at run-time.

When compiling expressions, a decision needs to be made whether the resulting
compiled patterns are to be used in a streaming, block or vectored mode:

- **Streaming mode**: the target data to be scanned is a continuous stream, not
  all of which is available at once; blocks of data are scanned in sequence and
  matches may span multiple blocks in a stream. In streaming mode, each stream
  requires a block of memory to store its state between scan calls.

- **Block mode**: the target data is a discrete, contiguous block which can be
  scanned in one call and does not require state to be retained.

- **Vectored mode**: the target data consists of a list of non-contiguous
  blocks that are available all at once. As for block mode, no retention of
  state is required.

To compile patterns to be used in streaming mode, the ``mode`` parameter of
:c:func:`hs_compile` must be set to :c:member:`HS_MODE_STREAM`; similarly,
block mode requires the use of :c:member:`HS_MODE_BLOCK` and vectored mode
requires the use of :c:member:`HS_MODE_VECTORED`. A pattern database compiled
for one mode (streaming, block or vectored) can only be used in that mode. The
version of Hyperscan used to produce a compiled pattern database must match the
version of Hyperscan used to scan with it.

Hyperscan provides support for targeting a database at a particular CPU
platform; see :ref:`instr_specialization` for details.

***************
Pattern Support
***************

Hyperscan supports the pattern syntax used by the PCRE library ("libpcre"),
described at <http://www.pcre.org/>. However, not all constructs available in
libpcre are supported. The use of unsupported constructs will result in
compilation errors.

The version of PCRE used to validate Hyperscan's interpretation of this syntax
is 8.41.

====================
Supported Constructs
====================

The following regex constructs are supported by Hyperscan:

* Literal characters and strings, with all libpcre quoting and character
  escapes.

* Character classes such as :regexp:`.` (dot), :regexp:`[abc]`, and
  :regexp:`[^abc]`, as well as the predefined character classes :regexp:`\\s`,
  :regexp:`\\d`, :regexp:`\\w`, :regexp:`\\v`, and :regexp:`\\h` and their
  negated counterparts (:regexp:`\\S`, :regexp:`\\D`, :regexp:`\\W`,
  :regexp:`\\V`, and :regexp:`\\H`).

* The POSIX named character classes :regexp:`[[:xxx:]]` and negated named
  character classes :regexp:`[[:^xxx:]]`.

* Unicode character properties, such as :regexp:`\\p{L}`, :regexp:`\\P{Sc}`,
  :regexp:`\\p{Greek}`.

* Quantifiers:

  * Quantifiers such as :regexp:`?`, :regexp:`*` and :regexp:`+` are supported
    when applied to arbitrary supported sub-expressions.

  * Bounded repeat qualifiers such as :regexp:`{n}`, :regexp:`{m,n}`,
    :regexp:`{n,}` are supported with limitations.

    * For arbitrary repeated sub-patterns: *n* and *m* should be either small
      or infinite, e.g. :regexp:`(a|b}{4}`, :regexp:`(ab?c?d){4,10}` or
      :regexp:`(ab(cd)*){6,}`.

    * For single-character width sub-patterns such as :regexp:`[^\\a]` or
      :regexp:`.` or :regexp:`x`, nearly all repeat counts are supported, except
      where repeats are extremely large (maximum bound greater than 32767).
      Stream states may be very large for large bounded repeats, e.g.
      :regexp:`a.{2000}b`. Note: such sub-patterns may be considerably
      cheaper if at the beginning or end of patterns and especially if the
      :c:member:`HS_FLAG_SINGLEMATCH` flag is on for that pattern.

  * Lazy modifiers (:regexp:`?` appended to another quantifier, e.g.
    :regexp:`\\w+?`) are supported but ignored (as Hyperscan reports all
    matches).

* Parenthesization, including the named and unnamed capturing and
  non-capturing forms. However, capturing is ignored.

* Alternation with the :regexp:`|` symbol, as in :regexp:`foo|bar`.

* The anchors :regexp:`^`, :regexp:`$`, :regexp:`\\A`, :regexp:`\\Z` and
  :regexp:`\\z`.

* Option modifiers:

  These allow behaviour to be switched on (with :regexp:`(?<option>)`) and off
  (with :regexp:`(?-<option>)`) for a sub-pattern. The supported options are:

    * :regexp:`i`: Case-insensitive matching, as per
      :c:member:`HS_FLAG_CASELESS`.
    * :regexp:`m`: Multi-line matching, as per :c:member:`HS_FLAG_MULTILINE`.
    * :regexp:`s`: Interpret ``.`` as "any character", as per
      :c:member:`HS_FLAG_DOTALL`.
    * :regexp:`x`: Extended syntax, which will ignore most whitespace in the
      pattern for compatibility with libpcre's ``PCRE_EXTENDED`` option.

  For example, the expression :regexp:`foo(?i)bar(?-i)baz` will switch on
  case-insensitive matching *only* for the ``bar`` portion of the match.

* The :regexp:`\\b` and :regexp:`\\B` zero-width assertions (word boundary and
  'not word boundary', respectively).

* Comments in :regexp:`(?# comment)` syntax.

* The :regexp:`(*UTF8)` and :regexp:`(*UCP)` control verbs at the beginning of a
  pattern, used to enable UTF-8 and UCP mode.

.. note:: Bounded-repeat quantifiers with large repeat counts of arbitrary
   expressions (e.g. :regexp:`([a-z]|bc*d|xy?z){1000,5000}`) will result in a
   "Pattern too large" error at pattern compile time.

.. note:: At this time, not all patterns can be successfully compiled with the
  :c:member:`HS_FLAG_SOM_LEFTMOST` flag, which enables per-pattern support for
  :ref:`som`. The patterns that support this flag are a subset of patterns that
  can be successfully compiled with Hyperscan; notably, many bounded repeat
  forms that can be compiled with Hyperscan without the Start of Match flag
  enabled cannot be compiled with the flag enabled.

======================
Unsupported Constructs
======================

The following regex constructs are not supported by Hyperscan:

* Backreferences and capturing sub-expressions.
* Arbitrary zero-width assertions.
* Subroutine references and recursive patterns.
* Conditional patterns.
* Backtracking control verbs.
* The :regexp:`\\C` "single-byte" directive (which breaks UTF-8 sequences).
* The :regexp:`\\R` newline match.
* The :regexp:`\\K` start of match reset directive.
* Callouts and embedded code.
* Atomic grouping and possessive quantifiers.

.. _semantics:

*********
Semantics
*********

While Hyperscan follows libpcre syntax, it provides different semantics. The
major departures from libpcre semantics are motivated by the requirements of
streaming and multiple simultaneous pattern matching.

The major departures from libpcre semantics are:

#. **Multiple pattern matching**: Hyperscan allows matches to be reported for
   several patterns simultaneously. This is not equivalent to separating the
   patterns by :regexp:`|` in libpcre, which evaluates alternations
   left-to-right.

#. **Lack of ordering**: the multiple matches that Hyperscan produces are not
   guaranteed to be ordered, although they will always fall within the bounds of
   the current scan.

#. **End offsets only**: Hyperscan's default behaviour is only to report the end
   offset of a match. Reporting of the start offset can be enabled with
   per-expression flags at pattern compile time. See :ref:`som` for details.

#. **"All matches" reported**: scanning :regexp:`/foo.*bar/` against
   ``fooxyzbarbar`` will return two matches from Hyperscan -- at the points
   corresponding to the ends of ``fooxyzbar`` and ``fooxyzbarbar``. In contrast,
   libpcre semantics by default would report only one match at ``fooxyzbarbar``
   (greedy semantics) or, if non-greedy semantics were switched on, one match at
   ``fooxyzbar``. This means that switching between greedy and non-greedy
   semantics is a no-op in Hyperscan.

To support libpcre quantifier semantics while accurately reporting streaming
matches at the time they occur is impossible. For example, consider the pattern
above, :regexp:`/foo.*bar/`, in streaming mode, against the following
stream (three blocks scanned in sequence):

    =============   =======     ========
    block 1         block 2     block 3
    =============   =======     ========
    ``fooxyzbar``   ``baz``     ``qbar``
    =============   =======     ========

Since the :regexp:`.*` repeat in the pattern is a *greedy* repeat in libpcre, it
must match as much as possible without causing the rest of the pattern to fail.
However, in streaming mode, this would require knowledge of data in the stream
beyond the current block being scanned.

In this example, the match at offset 9 in the first block is only the correct
match (under libpcre semantics) if there is no ``bar`` in a subsequent block --
as in block 3 -- which would constitute a better match for the pattern.

.. _som:

==============
Start of Match
==============

In standard operation, Hyperscan will only provide the end offset of a match
when the match callback is called. If the :c:member:`HS_FLAG_SOM_LEFTMOST` flag
is specified for a particular pattern, then the same set of matches is
returned, but each match will also provide the leftmost possible start offset
corresponding to its end offset.

Using the SOM flag entails a number of trade-offs and limitations:

* Reduced pattern support: For many patterns, tracking SOM is complex and can
  result in Hyperscan failing to compile a pattern with a "Pattern too
  large" error, even if the pattern is supported in normal operation.
* Increased stream state: At scan time, state space is required to track
  potential SOM offsets, and this must be stored in persistent stream state in
  streaming mode. Accordingly, SOM will generally increase the stream state
  required to match a pattern.
* Performance overhead: Similarly, there is generally a performance cost
  associated with tracking SOM.
* Incompatible features: Some other Hyperscan pattern flags (such as
  :c:member:`HS_FLAG_SINGLEMATCH` and :c:member:`HS_FLAG_PREFILTER`) can not be
  used in combination with SOM. Specifying them together with
  :c:member:`HS_FLAG_SOM_LEFTMOST` will result in a compilation error.

In streaming mode, the amount of precision delivered by SOM can be controlled
with the SOM horizon flags. These instruct Hyperscan to deliver accurate SOM
information within a certain distance of the end offset, and return a special
start offset of :c:member:`HS_OFFSET_PAST_HORIZON` otherwise. Specifying a
small or medium SOM horizon will usually reduce the stream state required for a
given database.

.. note:: In streaming mode, the start offset returned for a match may refer to
   a point in the stream *before* the current block being scanned. Hyperscan
   provides no facility for accessing earlier blocks; if the calling application
   needs to inspect historical data, then it must store it itself.

.. _extparam:

===================
Extended Parameters
===================

In some circumstances, more control over the matching behaviour of a pattern is
required than can be specified easily using regular expression syntax. For
these scenarios, Hyperscan provides the :c:func:`hs_compile_ext_multi` function
that allows a set of "extended parameters" to be set on a per-pattern basis.

Extended parameters are specified using an :c:type:`hs_expr_ext_t` structure,
which provides the following fields:

* ``flags``: Flags governing which of the other fields in the structure are
  used.
* ``min_offset``: The minimum end offset in the data stream at which this
  expression should match successfully.
* ``max_offset``: The maximum end offset in the data stream at which this
  expression should match successfully.
* ``min_length``: The minimum match length (from start to end) required to
  successfully match this expression.
* ``edit_distance``: Match this expression within a given Levenshtein distance.
* ``hamming_distance``: Match this expression within a given Hamming distance.

These parameters either allow the set of matches produced by a pattern to be
constrained at compile time (rather than relying on the application to process
unwanted matches at runtime), or allow matching a pattern approximately (within
a given edit distance) to produce more matches.

For example, the pattern :regexp:`/foo.*bar/` when given a ``min_offset`` of 10
and a ``max_offset`` of 15 will not produce matches when scanned against
``foobar`` or ``foo0123456789bar`` but will produce a match against the data
streams ``foo0123bar`` or ``foo0123456bar``.

Similarly, the pattern :regexp:`/foobar/` when given an ``edit_distance`` of 2
will produce matches when scanned against ``foobar``, ``f00bar``, ``fooba``,
``fobr``, ``fo_baz``, ``foooobar``, and anything else that lies within edit
distance of 2 (as defined by Levenshtein distance).

When the same pattern :regexp:`/foobar/` is given a ``hamming_distance`` of 2,
it will produce matches when scanned against ``foobar``, ``boofar``,
``f00bar``, and anything else with at most two characters substituted from the
original pattern. For more details, see the :ref:`approximate_matching`
section.

=================
Prefiltering Mode
=================

Hyperscan provides a per-pattern flag, :c:member:`HS_FLAG_PREFILTER`, which can
be used to implement a prefilter for a pattern than Hyperscan would not
ordinarily support.

This flag instructs Hyperscan to compile an "approximate" version of this
pattern for use in a prefiltering application, even if Hyperscan does not
support the pattern in normal operation.

The set of matches returned when this flag is used is guaranteed to be a
superset of the matches specified by the non-prefiltering expression.

If the pattern contains pattern constructs not supported by Hyperscan (such as
zero-width assertions, back-references or conditional references) these
constructs will be replaced internally with broader constructs that may match
more often.

For example, the pattern :regexp:`/(\\w+) again \\1/` contains the
back-reference :regexp:`\\1`. In prefiltering mode, this pattern might be
approximated by having its back-reference replaced with its referent, forming
:regexp:`/\\w+ again \\w+/`.

Furthermore, in prefiltering mode Hyperscan may simplify a pattern that would
otherwise return a "Pattern too large" error at compile time, or for performance
reasons (subject to the matching guarantee above).

It is generally expected that the application will subsequently confirm
prefilter matches with another regular expression matcher that can provide exact
matches for the pattern.

.. note:: The use of this flag in combination with Start of Match mode (using
   the :c:member:`HS_FLAG_SOM_LEFTMOST` flag) is not currently supported and
   will result in a pattern compilation error.

.. _instr_specialization:

******************************
Instruction Set Specialization
******************************

Hyperscan is able to make use of several modern instruction set features found
on x86 processors to provide improvements in scanning performance.

Some of these features are selected when the library is built; for example,
Hyperscan will use the native ``POPCNT`` instruction on processors where it is
available and the library has been optimized for the host architecture.

.. note:: By default, the Hyperscan runtime is built with the ``-march=native``
   compiler flag and (where possible) will make use of all instructions known by
   the host's C compiler.

To use some instruction set features, however, Hyperscan must build a
specialized database to support them. This means that the target platform must
be specified at pattern compile time.

The Hyperscan compiler API functions all accept an optional
:c:type:`hs_platform_info_t` argument, which describes the target platform
for the database to be built. If this argument is NULL, the database will be
targeted at the current host platform.

The :c:type:`hs_platform_info_t` structure has two fields:

#. ``tune``: This allows the application to specify information about the target
   platform which may be used to guide the optimisation process of the compile.
   Use of this field does not limit the processors that the resulting database
   can run on, but may impact the performance of the resulting database.

#. ``cpu_features``: This allows the application to specify a mask of CPU
   features that may be used on the target platform. For example,
   :c:member:`HS_CPU_FEATURES_AVX2` can be specified for Intel\ |reg| Advanced
   Vector Extensions 2 (Intel\ |reg| AVX2) instruction set support. If a flag
   for a particular CPU feature is specified, the database will not be usable on
   a CPU without that feature.

An :c:type:`hs_platform_info_t` structure targeted at the current host can be
built with the :c:func:`hs_populate_platform` function.

See :ref:`api_constants` for the full list of CPU tuning and feature flags.

.. _approximate_matching:

********************
Approximate matching
********************

Hyperscan provides an experimental approximate matching mode, which will match
patterns within a given edit distance. The exact matching behavior is defined as
follows:

#. **Edit distance** is defined as Levenshtein distance. That is, there are
   three possible edit types considered: insertion, removal and substitution.
   A more formal description can be found on
   `Wikipedia <https://en.wikipedia.org/wiki/Levenshtein_distance>`__.

#. **Hamming distance** is the number of positions by which two strings of
   equal length differ. That is, it is the number of substitutions required to
   convert one string to the other. There are no insertions or removals when
   approximate matching using a Hamming distance. A more formal description can
   be found on
   `Wikipedia <https://en.wikipedia.org/wiki/Hamming_distance>`__.

#. **Approximate matching** will match all *corpora* within a given edit or
   Hamming distance. That is, given a pattern, approximate matching will match
   anything that can be edited to arrive at a corpus that exactly matches the
   original pattern.

#. **Matching semantics** are exactly the same as described in :ref:`semantics`.

Here are a few examples of approximate matching:

* Pattern :regexp:`/foo/` can match ``foo`` when using regular Hyperscan
  matching behavior. With approximate matching within edit distance 2, the
  pattern will produce matches when scanned against ``foo``, ``foooo``, ``f00``,
  ``f``, and anything else that lies within edit distance 2 of matching corpora
  for the original pattern (``foo`` in this case).

* Pattern :regexp:`/foo(bar)+/` with edit distance 1 will match ``foobarbar``,
  ``foobarb0r``, ``fooarbar``, ``foobarba``, ``f0obarbar``, ``fobarbar`` and
  anything else that lies within edit distance 1 of matching corpora for the
  original pattern (``foobarbar`` in this case).

* Pattern :regexp:`/foob?ar/` with edit distance 2 will match ``fooar``,
  ``foo``, ``fabar``, ``oar`` and anything else that lies within edit distance 2
  of matching corpora for the original pattern (``fooar`` in this case).

Currently, there are trade-offs and limitations that come with approximate
matching support. Here they are, in a nutshell:

* Reduced pattern support:

  * For many patterns, approximate matching is complex and can result in
    Hyperscan failing to compile a pattern with a "Pattern too large" error,
    even if the pattern is supported in normal operation.
  * Additionally, some patterns cannot be approximately matched because they
    reduce to so-called "vacuous" patterns (patterns that match everything). For
    example, pattern :regexp:`/foo/` with edit distance 3, if implemented,
    would reduce to matching zero-length buffers. Such patterns will result in a
    "Pattern cannot be approximately matched" compile error. Approximate
    matching within a Hamming distance does not remove symbols, so will not
    reduce to a vacuous pattern.
  * Finally, due to the inherent complexities of defining matching behavior,
    approximate matching implements a reduced subset of regular expression
    syntax. Approximate matching does not support UTF-8 (and other
    multibyte character encodings), and word boundaries (that is, ``\b``, ``\B``
    and other equivalent constructs). Patterns containing unsupported constructs
    will result in "Pattern cannot be approximately matched" compile error.
  * When using approximate matching in conjunction with SOM, all of the
    restrictions of SOM also apply. See :ref:`som` for more
    details.
* Increased stream state/byte code size requirements: due to approximate
  matching byte code being inherently larger and more complex than exact
  matching, the corresponding requirements also increase.
* Performance overhead: similarly, there is generally a performance cost
  associated with approximate matching, both due to increased matching
  complexity, and due to the fact that it will produce more matches.

Approximate matching is always disabled by default, and can be enabled on a
per-pattern basis by using an extended parameter described in :ref:`extparam`.

.. _logical_combinations:

********************
Logical Combinations
********************

For situations when a user requires behaviour that depends on the presence or
absence of matches from groups of patterns, Hyperscan provides support for the
logical combination of patterns in a given pattern set, with three operators:
``NOT``, ``AND`` and ``OR``.

The logical value of such a combination is based on each expression's matching
status at a given offset. The matching status of any expression has a boolean
value: *false* if the expression has not yet matched or *true* if the expression
has already matched. In particular, the value of a ``NOT`` operation at a given
offset is *true* if the expression it refers to is *false* at this offset.

For example, ``NOT 101`` means that expression 101 has not yet matched at this
offset.

A logical combination is passed to Hyperscan at compile time as an expression.
This combination expression will raise matches at every offset where one of its
sub-expressions matches and the logical value of the whole expression is *true*.

To illustrate, here is an example combination expression: ::

    ((301 OR 302) AND 303) AND (304 OR NOT 305)

If expression 301 matches at offset 10, the logical value of 301 is *true*
while the other patterns' values are *false*. Hence, the whole combination's value is
*false*.

Then expression 303 matches at offset 20. Now the values of 301 and 303 are
*true* while the other patterns' values are still *false*. In this case, the
combination's value is *true*, so the combination expression raises a match at
offset 20.

Finally, expression 305 has matches at offset 30. Now the values of 301, 303 and 305
are *true* while the other patterns' values are still *false*. In this case, the
combination's value is *false* and no match is raised.

**Using Logical Combinations**

In logical combination syntax, an expression is written as infix notation, it
consists of operands, operators and parentheses. The operands are expression
IDs, and operators are ``!`` (NOT), ``&`` (AND) or ``|`` (OR). For example, the
combination described in the previous section would be written as: ::

    ((301 | 302) & 303) & (304 | !305)

In a logical combination expression:

 * The priority of operators are ``!`` > ``&`` > ``|``. For example:
    - ``A&B|C`` is treated as ``(A&B)|C``,
    - ``A|B&C`` is treated as ``A|(B&C)``,
    - ``A&!B`` is treated as ``A&(!B)``.
 * Extra parentheses are allowed. For example:
    - ``(A)&!(B)`` is the same as ``A&!B``,
    - ``(A&B)|C`` is the same as ``A&B|C``.
 * Whitespace is ignored.

To use a logical combination expression, it must be passed to one of the
Hyperscan compile functions (:c:func:`hs_compile_multi`,
:c:func:`hs_compile_ext_multi`) along with the :c:member:`HS_FLAG_COMBINATION` flag,
which identifies the pattern as a logical combination expression. The patterns
referred to in the logical combination expression must be compiled together in
the same pattern set as the combination expression.

When an expression has the :c:member:`HS_FLAG_COMBINATION` flag set, it ignores
all other flags except the :c:member:`HS_FLAG_SINGLEMATCH` flag and the
:c:member:`HS_FLAG_QUIET` flag.

Hyperscan will reject logical combination expressions at compile time that
evaluate to *true* when no patterns have matched; for example: ::

    !101
    !101|102
    !101&!102
    !(101&102)

Patterns that are referred to as operands within a logical combination (for
example, 301 through 305 in the examples above) may also use the
:c:member:`HS_FLAG_QUIET` flag to silence the reporting of individual matches
for those patterns. In the absence of this flag, all matches (for
both individual patterns and their logical combinations) will be reported.

When an expression has both the :c:member:`HS_FLAG_COMBINATION` flag and the
:c:member:`HS_FLAG_QUIET` flag set, no matches for this logical combination
will be reported.
