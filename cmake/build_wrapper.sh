#!/bin/sh -e
# This is used for renaming symbols for the fat runtime, don't call directly
# TODO: make this a lot less fragile!
PREFIX=$1
KEEPSYMS_IN=$2
shift 2
BUILD=$@
OUT=$(echo $BUILD | sed 's/.* -o \(.*\.o\).*/\1/')
SYMSFILE=/tmp/${PREFIX}_rename.syms.$$
KEEPSYMS=/tmp/keep.syms.$$
# grab the command without the target obj or src file flags
# we don't just call gcc directly as there may be flags modifying the arch
CC_CMD=$(echo $BUILD | sed 's/ -o .*\.o//;s/ -c //;s/ .[^ ]*\.c//;')
# find me a libc
LIBC_SO=$(${CC_CMD} --print-file-name=libc.so.6)
cp ${KEEPSYMS_IN} ${KEEPSYMS}
# get all symbols from libc and turn them into patterns
nm -f p -g -D ${LIBC_SO} | sed -s 's/\([^ ]*\).*/^\1$/' >> ${KEEPSYMS}
# build the object
${BUILD}
# rename the symbols in the object
nm -f p -g ${OUT} | cut -f1 -d' ' | grep -v -f ${KEEPSYMS} | sed -e "s/\(.*\)/\1\ ${PREFIX}_\1/" >> ${SYMSFILE}
if test -s ${SYMSFILE}
then
    objcopy --redefine-syms=${SYMSFILE} ${OUT}
fi
rm -f ${SYMSFILE} ${KEEPSYMS}
