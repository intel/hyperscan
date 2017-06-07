#!/usr/bin/env python

'''
This script creates a Hyperscan benchmarking corpus database from a supplied
group of Project Gutenberg texts.
'''

import sys, getopt, os.path
import gutenberg.acquire, gutenberg.cleanup, gutenberg.query
from CorpusBuilder import CorpusBuilder

stream_id = 0
stream_bytes = 0

def addBlocks(builder, block_size, stream_size, text_id, text):
    global stream_id
    global stream_bytes

    print "text", text_id, "len", len(text)
    i = 0
    while i < len(text):
        chunk = text[i:min(len(text), i + block_size)]
        builder.add_chunk(stream_id, chunk)
        i += block_size
        stream_bytes += len(chunk)
        if stream_bytes >= stream_size:
            stream_id += 1
            stream_bytes = 0
    print "Text", text_id, ": added", i/block_size, "blocks of", block_size, "bytes."

def buildCorpus(outFN, block_size, stream_size, text_ids):
    if len(text_ids) == 0:
        print >>sys.stderr, "Must provide at least one input ID"
        sys.exit(0)

    builder = CorpusBuilder(outFN)

    total_bytes = 0
    stream_id = 0
    stream_bytes = 0

    for text_id in text_ids:
        text_id = int(text_id)
        text = gutenberg.acquire.load_etext(text_id)
        text = gutenberg.cleanup.strip_headers(text).strip()
        addBlocks(builder, block_size, stream_size, text_id, text)
        total_bytes += len(text)

    builder.finish()

    print "Total:", total_bytes, "bytes."

def usage(exeName):
    errmsg = "Usage: %s -o <output file> -b <block size> -s <max stream size> <gutenberg text id>..."
    errmsg = errmsg % exeName
    print >> sys.stderr, errmsg
    sys.exit(-1)

if __name__ == '__main__':
    opts, args = getopt.getopt(sys.argv[1:], 'o:b:s:')
    opts = dict(opts)

    requiredKeys = [ '-o', '-b', '-s' ]
    for k in requiredKeys:
        if not opts.has_key(k):
            usage(os.path.basename(sys.argv[0]))

    buildCorpus(opts['-o'], int(opts['-b']), int(opts['-s']), args)
