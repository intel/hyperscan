.. _serialization:

#############
Serialization
#############

For some applications, compiling Hyperscan pattern databases immediately prior
to use is not an appropriate design. Some users may wish to:

* Compile pattern databases on a different host;

* Persist compiled databases to storage and only re-compile pattern databases
  when the patterns change;

* Control the region of memory in which the compiled database is located.

Hyperscan pattern databases are not completely flat in memory: they contain
pointers and have specific alignment requirements. Therefore, they cannot be
copied (or otherwise relocated) directly. To enable these use cases, Hyperscan
provides functionality for serializing and deserializing compiled pattern
databases.

The API provides the following functions:

#. :c:func:`hs_serialize_database`: serializes a pattern database into a
   flat relocatable buffer of bytes.

#. :c:func:`hs_deserialize_database`: reconstructs a newly allocated pattern
   database from the output of :c:func:`hs_serialize_database`.

#. :c:func:`hs_deserialize_database_at`: reconstructs a pattern
   database at a given memory location from the output of
   :c:func:`hs_serialize_database`.

#. :c:func:`hs_serialized_database_size`: given a serialized pattern database,
   returns the size of the memory block required by the database when
   deserialized.

#. :c:func:`hs_serialized_database_info`: given a serialized pattern database,
   returns a string containing information about the database. This call is
   analogous to :c:func:`hs_database_info`.

.. note:: Hyperscan performs both version and platform compatibility checks
   upon deserialization. The :c:func:`hs_deserialize_database` and
   :c:func:`hs_deserialize_database_at` functions will only permit the
   deserialization of databases compiled with (a) the same version of Hyperscan
   and (b) platform features supported by the current host platform. See
   :ref:`instr_specialization` for more information on platform specialization.

===================
The Runtime Library
===================

The main Hyperscan library (``libhs``) contains both the compiler and runtime
portions of the library. This means that in order to support the Hyperscan
compiler, which is written in C++, it requires C++ linkage and has a
dependency on the C++ standard library.

Many embedded applications require only the scanning ("runtime") portion of the
Hyperscan library. In these cases, pattern compilation generally takes place on
another host, and serialized pattern databases are delivered to the application
for use.

To support these applications without requiring the C++ dependency, a
runtime-only version of the Hyperscan library, called ``libhs_runtime``, is also
distributed. This library does not depend on the C++ standard library and
provides all Hyperscan functions other that those used to compile databases.
