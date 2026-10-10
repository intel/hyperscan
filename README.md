# Hyperscan

Hyperscan is a high-performance multiple regex matching library. It follows the
regular expression syntax of the commonly-used libpcre library, but is a
standalone library with its own C API.

Hyperscan uses hybrid automata techniques to allow simultaneous matching of
large numbers (up to tens of thousands) of regular expressions and for the
matching of regular expressions across streams of data.

Hyperscan is typically used in a DPI library stack.

This open source version of the library is available at https://github.com/intel/hyperscan. 
Intel also has an upgraded library version that is available through your Intel sales representative.

# Documentation

Information on building the Hyperscan library and using its API is available in
the [Developer Reference Guide](http://intel.github.io/hyperscan/dev-reference/).

# License

Hyperscan is licensed under the BSD License. See the LICENSE file in the
project repository.

# Versioning

The `master` branch on Github will always contain the most recent release of
the open source version of Hyperscan. Each version released to `master` goes 
through QA and testing before it is released; if you're a user, rather than 
a developer, this is the version you should be using.

Further development towards the next release takes place on the `develop`
branch.

Intel also has an upgraded library version that is available through your Intel sales representative.

# Get Involved

The official homepage for Hyperscan is at [www.hyperscan.io](https://www.hyperscan.io).

If you have questions or comments, we encourage you to reach out ot the developers on Github.

If you wish to contact the Hyperscan team at Intel directly, send email to
[hyperscan@intel.com](mailto:hyperscan@intel.com).
