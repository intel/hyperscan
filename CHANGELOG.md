# Hyperscan Change Log

This is a list of notable changes to Hyperscan, in reverse chronological order.

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
