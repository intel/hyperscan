#!/usr/bin/env python

from random import choice,randint
from optparse import OptionParser

def generateRandomOptions():
    if options.hybrid:
        allflags = "smiH8W"
    else:
        # Maintain an ordering for consistency.
        allflags = "smiHV8WLP"
    flags = ""
    for f in allflags:
        flags += choice(['', f])
    return flags

parser = OptionParser()
parser.add_option("-d", "--depth",
                  action="store", type="int", dest="depth", default=200,
                  help="Depth of generation (akin to maximum length)")
parser.add_option("-c", "--count",
                  action="store", type="int", dest="count", default=1000,
                  help="Number of expressions to generate")
parser.add_option("-f", "--full",
                  action="store_true", dest="full", default=False,
                  help="Use a full character set including unprintables")
parser.add_option("-H", "--hybrid",
                  action="store_true", dest="hybrid",
                  help="Generate random flags for hybrid mode")

(options, args) = parser.parse_args()
if len(args) != 0:
    parser.error("incorrect number of arguments")

if (options.full):
    crange = range(0,256)
    crange.remove(ord('\n'))
else:
    crange = range(32, 127)

for i in xrange(0, options.count):
    len = randint(1, options.depth)
    s = [ chr(choice(crange)) for x in xrange(len) ]
    line = str(i) + ":/" + "".join(s) + "/" + generateRandomOptions()
    print line
