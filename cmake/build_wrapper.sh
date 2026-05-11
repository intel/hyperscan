#!/bin/sh -e
# This is used for renaming symbols for the fat runtime, don't call directly
# TODO: make this a lot less fragile!
cleanup () {
    rm -f ${SYMSFILE} ${KEEPSYMS}
}

PREFIX=$1
KEEPSYMS_IN=$2
shift 2
# $@ contains the actual build command
OUT=$(echo "$@" | rev | cut -d ' ' -f 2- | rev | sed 's/.* -o \(.*\.o\).*/\1/')
trap cleanup INT QUIT EXIT
SYMSFILE=$(mktemp -p /tmp ${PREFIX}_rename.syms.XXXXX)
KEEPSYMS=$(mktemp -p /tmp keep.syms.XXXXX)
# find the libc used by gcc
LIBC_SO=$("$@" --print-file-name=libc.so.6)
cp ${KEEPSYMS_IN} ${KEEPSYMS}
# get all symbols from libc and turn them into patterns
nm -f p -g -D ${LIBC_SO} | sed -s 's/\([^ @]*\).*/^\1$/' >> ${KEEPSYMS}
# get all symbols from libcrypto (OpenSSL) and keep them unprefixed
LIBCRYPTO_SO=$("$@" --print-file-name=libcrypto.so)
if [ -f "${LIBCRYPTO_SO}" ]; then
    nm -f p -g -D ${LIBCRYPTO_SO} | sed -s 's/\([^ @]*\).*/^\1$/' >> ${KEEPSYMS}
fi
# build the object
"$@"
# rename the symbols in the object
nm -f p -g ${OUT} | cut -f1 -d' ' | grep -v -f ${KEEPSYMS} | sed -e "s/\(.*\)/\1\ ${PREFIX}_\1/" >> ${SYMSFILE}
# Also rename local COMDAT group key symbols (nm type 'n') so each
# micro-arch variant gets a unique COMDAT signature and the linker
# does not deduplicate sections across variants.
nm -f p ${OUT} | grep " n " | cut -f1 -d' ' | grep -v -f ${KEEPSYMS} | sed -e "s/\(.*\)/\1\ ${PREFIX}_\1/" >> ${SYMSFILE}
# ASan generates per-TU __odr_asan_gen_* globals for ODR checks.
# These start with __ and are excluded by the ^_ keep pattern,
# but must be prefixed to avoid multiple-definition errors.
nm -f p -g ${OUT} | cut -f1 -d' ' | grep '^__odr_asan_gen' | sed -e "s/\(.*\)/\1\ ${PREFIX}_\1/" >> ${SYMSFILE}
if test -s ${SYMSFILE}
then
    objcopy --redefine-syms=${SYMSFILE} ${OUT}
fi
