#!/usr/bin/env python

from optparse import OptionParser
from random import *
import string
import sys

def generateRandomOptions():
    if options.rlit:
        allflags = "iHL"
    elif options.hybrid:
        allflags = "smiH8W"
    else:
        # Maintain an ordering for consistency.
        allflags = "smiHV8WLP"
    flags = ""
    for f in allflags:
        flags += choice(['', f])
    return flags

# return a random non-degenerate (ie not [10]) partition of nChildren 
def chooseLeafWidth(nChildren):
    width = randint(1, 5)
    width = min(width, nChildren-1)
    s = sample(range(1, nChildren), width)
    s.sort()
    s = [0] + s + [nChildren]
    v = [ s[i+1] - s[i] for i in range(0, len(s)-1) if s[i+1] != s[i] ]
    return v

def generateLogicalId(nChildren, atTop = False):
    return str(randint(0, options.count-1))

def makeLogicalGroup(s):
    return "(" + s + ")"

def generateLogicalNot(nChildren, atTop):
    r = generateCombination(nChildren, [generateLogicalNot], atTop = False)
    return "!" + makeLogicalGroup(r)

def generateLogicalAnd(nChildren, atTop):
    v = [ generateCombination(w, [generateLogicalAnd], atTop = False) for w in chooseLeafWidth(nChildren) ]
    v = [ r for r in v if r != '' ]
    s = string.join(v, "&")
    if len(v) == 1:
        return s
    else:
        return makeLogicalGroup(s)

def generateLogicalOr(nChildren, atTop):
    v = [ generateCombination(w, [generateLogicalOr], atTop = False) for w in chooseLeafWidth(nChildren) ]
    v = [ r for r in v if r != '' ]
    s = string.join(v, "|")
    if len(v) == 1:
        return s
    else:
        return makeLogicalGroup(s)

weightsLogicalTree = [
    (generateLogicalNot, 1),
    (generateLogicalAnd, 5),
    (generateLogicalOr, 5),
    ]

weightsLogicalLeaf = [
    (generateLogicalId, 1),
    ]

def genChoices(weighted):
    r = []
    for (f, w) in weighted:
        r = r + [f] * w
    return r

choicesLogicalTree = genChoices(weightsLogicalTree)
choicesLogicalLeaf = genChoices(weightsLogicalLeaf)

def generateCombination(nChildren, suppressList = [], atTop = False):
    nChildren -= 1
    if nChildren == 0:
        res = choice(choicesLogicalLeaf)(nChildren, atTop)
    else:
        c = [ ch for ch in choicesLogicalTree if ch not in suppressList ]
        res = choice(c)(nChildren, atTop)

    return res

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
parser.add_option("-l", "--logical",
                  action="store_true", dest="logical",
                  help="Generate logical combination expressions")
parser.add_option("-L", "--rlit",
                  action="store_true", dest="rlit",
                  help="Use RCL literal API with less supported flags")

(options, args) = parser.parse_args()
if len(args) != 0:
    parser.error("incorrect number of arguments")

if (options.full):
    crange = range(0,256)
    crange.remove(ord('\n'))
else:
    crange = range(32, 127)

for i in xrange(0, options.count):
    length = randint(1, options.depth)
    s = [ chr(choice(crange)) for x in xrange(length) ]
    line = str(i) + ":/" + "".join(s) + "/" + generateRandomOptions()
    print line

if options.logical:
    for i in xrange(options.count, options.count + 100):
        print "%08d:/%s/C" % (i, generateCombination(randint(1, options.depth), atTop = True))

