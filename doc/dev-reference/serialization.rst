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
   These compatibility checks are not a security mechanism; see
   :ref:`db_integrity` below.

.. _db_integrity:

==========================================
Database Integrity and Untrusted Databases
==========================================

Each compiled database carries two HMAC-SHA256 digests: one over the database
header fields (magic, version, length and platform) and one over the database
bytecode. These digests are computed when the database is compiled and are
fully verified (header and bytecode) by :c:func:`hs_deserialize_database`,
:c:func:`hs_deserialize_database_at`, :c:func:`hs_serialize_database` and
:c:func:`hs_alloc_scratch`. A database whose digests do not match is rejected
with :c:member:`HS_INVALID`.

The version and platform checks described above are performed first, so a
database from a different version of Hyperscan is reported as
:c:member:`HS_DB_VERSION_ERROR`, and a database built for a platform the host
does not support is reported as :c:member:`HS_DB_PLATFORM_ERROR`. A database
with a bad magic number, an inconsistent length or a truncated buffer is
reported as :c:member:`HS_INVALID`.

.. note:: :c:func:`hs_serialized_database_size`,
   :c:func:`hs_serialized_database_info`, :c:func:`hs_database_size` and
   :c:func:`hs_database_info` only check the header fields (magic, version and
   length). They do not verify the digests, so a successful return from these
   functions does not show that the database is intact or authentic.

.. warning:: **The default HMAC key is public.** Unless overridden at build
   time, the key used for these digests is a fixed constant
   (``HS_DB_HMAC_KEY``) defined in ``src/hs_db_hmac_key.h`` and published with
   the Hyperscan source code. With the default key, the integrity check only
   detects accidental corruption (for example, truncation or bit errors in
   storage or transit). It does **not** authenticate the origin of a
   database: anyone with access to the Hyperscan source can produce a modified
   serialized database that passes the check.

   A database that passes deserialization must therefore not be assumed to be
   a database produced by a trusted compiler. Loading a crafted database may
   cause memory-safety failures (such as out-of-bounds reads or crashes) during
   scanning.

Applications must treat serialized databases as **trusted input only**. If
serialized databases are stored on, or delivered through, any medium that may
be modified by a party other than the application owner (files, network
transfers, shared caches, update packages, etc.), the application is
responsible for authenticating the bytes *before* passing them to
:c:func:`hs_deserialize_database` or :c:func:`hs_deserialize_database_at`. For
example:

* sign serialized databases with an asymmetric key on the build host and
  verify the signature in the application before deserialization;

* deliver databases only over an authenticated, integrity-protected channel
  (for example, mutually authenticated TLS) from a trusted source;

* restrict write access to stored databases to the application owner.

Overriding the HMAC Key
-----------------------

The built-in key can be replaced with a site-specific 32-byte key by defining
``HS_DB_HMAC_KEY_OVERRIDE`` at build time. The macro must expand to a
brace-enclosed initializer of exactly 32 bytes. Because the key is used by
both the C and C++ portions of the library, it must be passed to both
``CMAKE_C_FLAGS`` and ``CMAKE_CXX_FLAGS``::

    KEY='{0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20}'
    cmake -DCMAKE_C_FLAGS="\"-DHS_DB_HMAC_KEY_OVERRIDE=$KEY\"" \
          -DCMAKE_CXX_FLAGS="\"-DHS_DB_HMAC_KEY_OVERRIDE=$KEY\"" \
          [...]

The inner escaped quotes (``\"``) are required: without them, a build tool that
runs commands through ``bash`` (for example ``make`` on Fedora/RHEL, where
``/bin/sh`` is ``bash``) expands the braces and the build fails.

The key above is an example only; generate a random key from a
cryptographically secure source and keep it out of version control.

Note the following when overriding the key:

* The library that compiles a database and the library (``libhs`` or
  ``libhs_runtime``) that deserializes it must be built with the same key.
  Databases compiled with a different key are rejected with
  :c:member:`HS_INVALID`.

* The key is embedded in the resulting library binary. Anyone who can read
  that binary can recover the key and forge databases accepted by it. A
  custom key prevents forgery by parties who only have the public Hyperscan
  source, but it is not a substitute for the application-level authentication
  described above.

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
