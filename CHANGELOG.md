# Hyperscan Change Log

This is a list of notable changes to Hyperscan, in reverse chronological order.

## [5.3.0] 2020-05-15
- Improvement on literal matcher "Teddy" performance, including support for
  Intel(R) AVX-512 Vector Byte Manipulation Instructions (Intel(R) AVX-512
  VBMI).
- Improvement on single-byte/two-byte matching performance, including support
  for Intel(R) Advanced Vector Extensions 512 (Intel(R) AVX-512).
- hsbench: add hyphen support for -T option.
- tools/fuzz: add test scripts for synthetic pattern generation.
- Bugfix for acceleration path analysis in LimEx NFA.
- Bugfix for duplicate matches for Small-write engine.
- Bugfix for UTF8 checking problem for hscollider.
- Bugfix for issue #205: avoid crash of `hs_compile_lit_multi()` with clang and
  ASAN.
- Bugfix for issue #211: fix error in `db_check_platform()` function.
- Bugfix for issue #217: fix cmake parsing issue of CPU arch for non-English
  locale.
- Bugfix for issue #228: avoid undefined behavior when calling `close()` after
  `fdopendir()` in `loadExpressions()`.
- Bugfix for issue #239: fix hyperscan compile issue under gcc-10.
- Add VLAN packets processing capability in pcap analysis script. (#214)
- Avoid extra convert instruction for "Noodle". (#221)
- Add Hyperscan version marcro in `hs.h`. (#222)

## [5.2.1] 2019-10-13
- Bugfix for issue #186: fix compile issue when `BUILD_SHARED_LIBS` is on in
  release mode.
- Disable redundant move check for older compiler versions.

## [5.2.0] 2019-07-12
- Literal API: add new API `hs_compile_lit()` and `hs_compile_lit_multi()` to
  process pure literal rule sets. The 2 literal APIs treat each expression text
  in a literal sense without recognizing any regular grammers.
- Logical combination: add support for purely negative combinations, which
  report match at EOD in case of no sub-expressions matched.
- Windows porting: support shared library (DLL) on Windows with available tools
  hscheck, hsbench and hsdump.
- Bugfix for issue #148: fix uninitialized use of `scatter_unit_uX` due to
  padding.
- Bugfix for issue #155: fix numerical result out of range error.
- Bugfix for issue #165: avoid corruption of pending combination report in
  streaming mode.
- Bugfix for issue #174: fix scratch free issue when memory allocation fails.

## [5.1.1] 2019-04-03
- Add extra detection and handling when invalid rose programs are triggered.
- Bugfix for issue #136: fix CMake parsing of CPU architecure for GCC-9.
- Bugfix for issue #137: avoid file path impact on fat runtime build.
- Bugfix for issue #141: fix rose literal programs for multi-pattern
  matching when no pattern ids are provided.
- Bugfix for issue #144: fix library install path in pkg-config files.

## [5.1.0] 2019-01-17
- Improve DFA state compression by wide-state optimization to reduce bytecode
  size.
- Create specific interpreter runtime handling to boost the performance of pure
  literal matching.
- Optimize original presentation of interpreter (the "Rose" engine ) to
  increase overall performance.
- Bugfix for logical combinations: fix error reporting combination's match in
  case of sub-expression has EOD match under streaming mode.
- Bugfix for logical combinations: fix miss reporting combination's match under
  vacuous input.
- Bugfix for issue #104: fix compile error with Boost 1.68.0.
- Bugfix for issue #127: avoid pcre error for hscollider with installed PCRE
  package.
- Update version of PCRE used by testing tools as a syntax and semantic
  reference to PCRE 8.41 or above.
- Fix github repo address in doc.

## [5.0.0] 2018-07-09
- Introduce chimera hybrid engine of Hyperscan and PCRE, to fully support
  PCRE syntax as well as to take advantage of the high performance nature of
  Hyperscan.
- New API feature: logical combinations (AND, OR and NOT) of patterns in a
  given pattern set.
- Windows porting: hsbench, hscheck, hscollider and hsdump tools now available
  on Windows 8 or newer.
- Improve undirected graph implementation to avoid graph copy and reduce
  compile time.
- Bugfix for issue #86: enable hscollider for installed PCRE package.

## [4.7.0] 2018-01-24
- Introduced hscollider pattern testing tool, for validating Hyperscan match
  behaviour against PCRE.
- Introduced hscheck pattern compilation tool.
- Introduced hsdump development tool for producing information about Hyperscan
  pattern compilation.
- New API feature: extended approximate matching support for Hamming distance.
- Bugfix for issue #69: Force C++ linkage in Xcode.
- Bugfix for issue #73: More documentation for `hs_close_stream()`.
- Bugfix for issue #78: Fix for fat runtime initialisation when used as a
  shared library.

## [4.6.0] 2017-09-22
- New API feature: stream state compression. This allows the user to compress
  and restore state for streams to reduce memory usage.
- Many improvements to literal matching performance, including more support
  for Intel(R) Advanced Vector Extensions 512 (Intel(R) AVX-512).
- Compile time improvements, mainly reducing compiler memory allocation.
  Also results in reduced compile time for some pattern sets.
- Bugfix for issue #62: fix error building Hyperscan using older versions of
  Boost.
- Small updates to fix warnings identified by Coverity.

## [4.5.2] 2017-07-26
- Bugfix for issue #57: Treat characters between `\Q.\E` as codepoints in
  UTF8 mode.
- Bugfix for issue #60: Use a portable flag for mktemp for fat runtime builds.
- Bugfix for fat runtime builds on AVX-512 capable machines with Hyperscan's
  AVX-512 support disabled.

## [4.5.1] 2017-06-16
- Bugfix for issue #56: workaround for gcc-4.8 C++11 defect.
- Bugfix for literal matching table generation, reversing a regression in
  performance for some literal matching cases.
- Bugfixes for hsbench, related to multicore benchmarking, portability fixes
  for FreeBSD, and clarifying output results.
- CMake: removed a duplicate else branch that causes very recent (v3.9) builds
  of CMake to fail.

## [4.5.0] 2017-06-09
- New API feature: approximate matching using the "edit distance" extended
  parameter. This allows the user to request all matches that are a given edit
  distance from an exact match for a pattern.
- Initial support for Intel(R) Advanced Vector Extensions 512 (Intel(R)
  AVX-512), disabled by default. To enable it, pass `-DBUILD_AVX512=1` to
  `cmake`.
- Major compile time improvements in many subsystems, reducing compile time
  significantly for many large pattern sets.
- Internal reworking of literal matchers to operate on literals of at
  most eight characters, with subsequent confirmation done in the Rose
  interpreter. This reduces complexity and bytecode size and improves
  performance for many pattern sets.
- Improve performance of the FDR literal matcher front end.
- Improve bucket assignment and other heuristics governing the FDR literal
  matcher.
- Improve optimisation passes that take advantage of extended parameter
  constraints (`min_offset`, etc).
- Introduce further lookaround specialisations to improve scanning performance.
- Optimise Rose interpreter construction to reduce the length of programs
  generated in some situations.
- Remove the old "Rose" pattern decomposition analysis pass in favour of the
  new "Violet" pass introduced in Hyperscan 4.3.0.
- In streaming mode, allow exhaustion (where the stream can no longer produce
  matchers) to be detected in more situations, improving scanning performance.
- Improve parsing of control verbs (such as `(*UTF8)`) that can only occur at
  the beginning of the pattern. Combinations of supported verbs in any order
  are now permitted.
- Update version of PCRE used by testing tools as a syntax and semantic
  reference to PCRE 8.40.
- Tuning support for Intel(R) microarchitecture code names Skylake, Skylake
  Server, Goldmont.
- CMake: when building a native build with a version of GCC that doesn't
  recognise the host compiler, tune for the microarch selected by
  `-march=native`.
- CMake: don't fail if SQLite (which is only required to build the `hsbench`
  tool) is not present.
- CMake: detect libc++ directly and use that to inform the Boost version
  requirement.
- Bugfix for issue #51: make the fat runtime build wrapper less fragile.
- Bugfix for issues #46, #52: use `sqlite3_errmsg()` to allow SQLite 3.6.x to
  be used. Thanks to @EaseTheWorld for the PR.

## [4.4.1] 2017-02-28
- Bugfixes to fix issues where stale data was being referenced in scratch
  memory. In particular this may have resulted in `hs_close_stream()`
  referencing data from other previously scanned streams. This may result in
  incorrect matches being been reported.

## [4.4.0] 2017-01-20
- Introduce the "fat runtime" build. This will build several variants of the
  Hyperscan scanning engine specialised for different processor feature sets,
  and use the appropriate one for the host at runtime. This uses the "ifunc"
  indirect function attribute provided by GCC and is currently available on
  Linux only, where it is the default for release builds.
- New API function: add the `hs_valid_platform()` function. This function tests
  whether the host provides the SSSE3 instruction set required by Hyperscan.
- Introduce a new standard benchmarking tool, "hsbench". This provides an easy
  way to measure Hyperscan's performance for a particular set of patterns and
  corpus of data to be scanned.
- Introduce a 64-bit GPR LimEx NFA model, which uses 64-bit GPRs on 64-bit
  hosts and SSE registers on 32-bit hosts.
- Introduce a new DFA model ("McSheng") which is a hybrid of the existing
  McClellan and Sheng models. This improves scanning performance for some
  cases.
- Introduce lookaround specialisations to improve scanning performance.
- Improve the handling of long literals by moving confirmation to the Rose
  interpreter and simplifying the hash table used to track them in streaming
  mode.
- Improve compile time optimisation for removing redundant paths from
  expression graphs.
- Build: improve support for building with MSVC toolchain.
- Reduce the size of small write DFAs used for small scans in block mode.
- Introduce a custom graph type (`ue2_graph`) used in place of the Boost Graph
  Library's `adjacency_list` type. Improves compile time performance and type
  safety.
- Improve scanning performance of the McClellan DFA.
- Bugfix for a very unusual SOM case where the incorrect start offset was
  reported for a match.
- Bugfix for issue #37, removing execute permissions from some source files.
- Bugfix for issue #41, handle Windows line endings in pattern files.

## [4.3.2] 2016-11-15
- Bugfix for issue #39. This small change is a workaround for an issue in
  Boost 1.62. The fix has been submitted to Boost for inclusion in a future
  release.

## [4.3.1] 2016-08-29
- Bugfix for issue #30. In recent versions of Clang, a write to a variable was
  being elided, resulting in corrupted stream state after calling
  `hs_reset_stream()`.

## [4.3.0] 2016-08-24
- Introduce a new analysis pass ("Violet") used for decomposition of patterns
  into literals and smaller engines.
- Introduce a new container engine ("Tamarama") for infix and suffix engines
  that can be proven to run exclusively of one another. This reduces stream
  state for pattern sets with many such engines.
- Introduce a new shuffle-based DFA engine ("Sheng"). This improves scanning
  performance for pattern sets where small engines are generated.
- Improve the analysis used to extract extra mask information from short
  literals.
- Reduced compile time spent in equivalence class analysis.
- Build: frame pointers are now only omitted for 32-bit release builds.
- Build: Workaround for C++ issues reported on FreeBSD/libc++ platforms.
  (github issue #27)
- Simplify the LimEx NFA with a unified "variable shift" model, which reduces
  the number of different NFA code paths to one per model size.
- Allow some anchored prefixes that may squash the literal to which they are
  attached to run eagerly. This improves scanning performance for some
  patterns.
- Simplify and improve EOD ("end of data") matching, using the interpreter for
  all operations.
- Elide unnecessary instructions in the Rose interpreter at compile time.
- Reduce the number of inlined instantiations of the Rose interpreter in order
  to reduce instruction cache pressure.
- Small improvements to literal matcher acceleration.
- Parser: ignore `\E` metacharacters that are not preceded by `\Q`. This
  conforms to PCRE's behaviour, rather than returning a compile error.
- Check for misaligned memory when allocating an error structure in Hyperscan's
  compile path and return an appropriate error if detected.

## [4.2.0] 2016-05-31
- Introduce an interpreter for many complex actions to replace the use of
  internal reports within the core of Hyperscan (the "Rose" engine). This
  improves scanning performance and reduces database size for many pattern
  sets.
- Many enhancements to the acceleration framework used by NFA and DFA engines,
  including more flexible multibyte implementations and more AVX2 support. This
  improves scanning performance for many pattern sets.
- Improved prefiltering support for complex patterns containing very large
  bounded repeats (`R{M,N}` with large `N`).
- Improve scanning performance of pattern sets with a very large number of
  EOD-anchored patterns.
- Improve scanning performance of large pattern sets that use the
  `HS_FLAG_SINGLEMATCH` flag.
- Improve scanning performance of pattern sets that contain a single literal by
  improving the "Noodle" literal matcher.
- Small reductions in total stream state for many pattern sets.
- Improve runtime detection of AVX2 support.
- Disable -Werror for release builds, in order to behave better for packagers
  and users with different compiler combinations than those that we test.
- Improve support for building on Windows with MSVC 2015 (github issue #14).
  Support for Hyperscan on Windows is still experimental.
- Small updates to fix warnings identified by Coverity.
- Remove Python codegen for the "FDR" and "Teddy" literal matchers. These are
  now implemented directly in C code.
- Remove the specialist "Sidecar" engine in favour of using our more general
  repeat engines.
- New API function: add the `hs_expression_ext_info()` function. This is a
  variant of `hs_expression_info()` that can accept patterns with extended
  parameters.
- New API error value: add the `HS_SCRATCH_IN_USE` error, which is returned
  when Hyperscan detects that a scratch region is already in use on entry to an
  API function.

## [4.1.0] 2015-12-18
- Update version of PCRE used by testing tools as a syntax and semantic
  reference to PCRE 8.38.
- Small updates to fix warnings identified by Coverity.
- Clean up and unify exception handling behaviour across GPR and SIMD NFA
  models.
- Fix bug in handling of bounded repeat triggers with large gaps between them
  for sparse repeat model.
- Correctly reject POSIX collating elements (`[.ch.]`, `[=ch=]`) in the parser.
  These are not supported by Hyperscan.
- Add support for quoted sequences (`\Q...\E`) inside character classes.
- Simplify FDR literal matcher runtime by removing some static specialization.
- Fix handling of the POSIX `[:graph:]`, `[:print:]` and `[:punct:]` character
  classes to match the behaviour of PCRE 8.38 in both standard operation and
  with the UCP flag set. (Note: some bugs were fixed in this area in PCRE
  8.38.) Previously Hyperscan's behaviour was the same as versions of PCRE
  before 8.34.
- Improve performance when compiling pattern sets that include a large number
  of similar bounded repeat constructs. (github issue #9)

## [4.0.1] 2015-10-30
- Minor cleanups to test code.
- CMake and other build system improvements.
- API update: allow `hs_reset_stream()` and `hs_reset_and_copy_stream()` to be
  supplied with a NULL scratch pointer if no matches are required. This is in
  line with the behaviour of `hs_close_stream()`.
- Disallow bounded repeats with a very large minimum repeat but no maximum,
  i.e. {N,} for very large N.
- Reduce compile memory usage in literal set explansion for some large cases.

## [4.0.0] 2015-10-20
- Original release of Hyperscan as open-source software.
