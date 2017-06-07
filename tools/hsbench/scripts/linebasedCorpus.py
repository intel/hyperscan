#!/usr/bin/env python

'''
Simple script to take a file full of lines of text and push them into a
Hyperscan benchmarking corpus database, one block per line.
'''

import sys, getopt, os.path
from CorpusBuilder import CorpusBuilder

def lineCorpus(inFN, outFN):
    '''
    Read lines from file name @inFN and write them as blocks to a new db with
    name @outFN.
    '''

    if not os.path.exists(inFN):
        print >> sys.stderr, "Input file '%s' does not exist. Exiting." % outFN
        sys.exit(-1)

    lines = open(inFN).readlines()

    if len(lines) == 0:
        print >> sys.stderr, "Input file contained no lines. Exiting."
        sys.exit(0)

    builder = CorpusBuilder(outFN)

    # write a single stream to contain everything
    streamId = 0

    for l in lines:
        builder.add_chunk(streamId, l.rstrip())

    builder.finish()

def usage(exeName):
    errmsg = "Usage: %s -i <input file> -o <output file>"
    errmsg = errmsg % exeName
    print >> sys.stderr, errmsg
    sys.exit(-1)

if __name__ == '__main__':
    args = getopt.getopt(sys.argv[1:], 'i:o:c:')
    args = dict(args[0])

    requiredKeys = [ '-i', '-o' ]
    for k in requiredKeys:
        if not args.has_key(k):
            usage(os.path.basename(sys.argv[0]))

    fnArgs = tuple([args[k] for k in requiredKeys])
    lineCorpus(*fnArgs)
