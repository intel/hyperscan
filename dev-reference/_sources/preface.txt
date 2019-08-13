#######
Preface
#######

********
Overview
********

Hyperscan is a regular expression engine designed to offer high performance, the
ability to match multiple expressions simultaneously and flexibility in
scanning operation.

Patterns are provided to a compilation interface which generates an immutable
pattern database. The scan interface then can be used to scan a target data
buffer for the given patterns, returning any matching results from that data
buffer. Hyperscan also provides a streaming mode, in which matches that span
several blocks in a stream are detected.

This document is designed to facilitate code-level integration of the Hyperscan
library with existing or new applications.

:ref:`intro` is a short overview of the Hyperscan library, with more detail on
the Hyperscan API provided in the subsequent sections: :ref:`compilation` and
:ref:`runtime`.

:ref:`perf` provides details on various factors which may impact the
performance of a Hyperscan integration.

:ref:`api_constants` and :ref:`api_files` provides a detailed summary of the
Hyperscan Application Programming Interface (API).

********
Audience
********

This guide is aimed at developers interested in integrating Hyperscan into an
application. For information on building the Hyperscan library, see the Quick
Start Guide.

***********
Conventions
***********

* Text in a ``fixed-width font`` refers to a code element, e.g. type name;
  function or method name.
* Text in a :regexp:`coloured fixed-width font` refers to a regular
  expression or a part of a regular expression.
