#!/usr/bin/env python

from optparse import OptionParser
from random import *
import string
import sys

# return a random non-degenerate (ie not [10]) partition of nChildren 
def chooseLeafWidth(nChildren):
    width = randint(1, 5)
    width = min(width, nChildren-1)
    s = sample(range(1, nChildren), width)
    s.sort()
    s = [0] + s + [nChildren]
    v = [ s[i+1] - s[i] for i in range(0, len(s)-1) if s[i+1] != s[i] ]
    return v

def generateConcat(nChildren, atTopIgnored):
    v = [ generateRE(w, atTop = False) for w in chooseLeafWidth(nChildren) ]
    v = [ r for r in v if r != '' ]	
    return string.join(v, "")

def makeGroup(s):
    # Parenthesise either in normal parens or a non-capturing group.
    if randint(0, 1) == 0:
        return "(" + s + ")"
    else:
        return "(?:" + s + ")"

def generateAlt(nChildren, atTop):
    v = [ generateRE(w, [generateAlt], atTop) for w in chooseLeafWidth(nChildren) ]
    v = [ r for r in v if r != '' ]	
    s = string.join(v, "|")
    if len(v) == 1:
	    return s
    else:
        return makeGroup(s)

def generateQuant(nChildren, atTopIgnored):
    lo = int(round(expovariate(0.2)))
    hi = lo + int(round(expovariate(0.2)))
    q = choice(["*", "?", "+", "{%d}"%lo, "{%d,}"%lo, "{%d,%d}"%(lo,hi)])
    r = generateRE(nChildren, [generateQuant], atTop = False)
    if (len(r) == 1) or (r[0] != '(' and r[-1] != ")"):  
        return r + q
    else:
        return makeGroup(r) + q

def generateChar(nChildren, atTop = False):
    return chr(choice(alphabet))

def generateNocaseChar(nChildren, atTop = False):
    'Either generate an uppercase char from the alphabet or a nocase class [Aa]'
    c = generateChar(nChildren, atTop)
    if random() < 0.5:
        return c.upper()
    else:
        return '[' + c.upper() + c.lower() + ']'

def generateDot(nChildren, atTop = False):
    return "."

def generateBoundary(nChildren, atTop = False):
    # \b, \B in parens so that we can repeat them and still be accepted by 
    # libpcre
    return makeGroup('\\' + choice('bB'))

def generateCharClass(nChildren, atTop = False):
    s = ""
    if random() < 0.2:
        s = "^"
        nChars = randint(1,4)
    else:
        nChars = randint(2,4)

    for i in xrange(nChars):
        s += generateChar(1)
    return "[" + s + "]"

def generateOptionsFlags(nChildren, atTop = False):
    allflags = "smix"
    pos_flags = sample(allflags, randint(1, len(allflags)))
    neg_flags = sample(allflags, randint(1, len(allflags)))
    s = '(?' + ''.join(pos_flags) + '-' + ''.join(neg_flags) + ')'
    return s 

def generateLogicalId(nChildren, atTop = False):
    return str(randint(0, options.count))

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

weightsTree = [
    (generateConcat, 10),
    (generateAlt, 3),
    (generateQuant, 2),
    ]

weightsLeaf = [
    (generateChar, 30),
    (generateCharClass, 5),
    (generateDot, 5),
    (generateNocaseChar, 2),
    (generateBoundary, 1),
    (generateOptionsFlags, 1)
    ]

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

choicesTree = genChoices(weightsTree)
choicesLeaf = genChoices(weightsLeaf)
choicesLogicalTree = genChoices(weightsLogicalTree)
choicesLogicalLeaf = genChoices(weightsLogicalLeaf)

weightsAnchor = [
    ("\\A%s\\Z", 1),
    ("\\A%s\\z", 1),
    ("\\A%s",  4),
    ("%s\\Z", 2),
    ("%s\\z", 2),
    ("^%s$", 1),
    ("^%s",  4),
    ("%s$", 2),
    ("%s", 25)
    ]
choicesAnchor = genChoices(weightsAnchor)

def generateRE(nChildren, suppressList = [], atTop = False):
    if atTop:
        anchorSubstituteString = choice(choicesAnchor)
    else:
        anchorSubstituteString = "%s"

    nChildren -= 1
    if nChildren == 0:
        res = choice(choicesLeaf)(nChildren, atTop)
    else:
        c = [ ch for ch in choicesTree if ch not in suppressList ]
        res = choice(c)(nChildren, atTop)

    return anchorSubstituteString % res

def generateCombination(nChildren, suppressList = [], atTop = False):
    nChildren -= 1
    if nChildren == 0:
        res = choice(choicesLogicalLeaf)(nChildren, atTop)
    else:
        c = [ ch for ch in choicesLogicalTree if ch not in suppressList ]
        res = choice(c)(nChildren, atTop)

    return res

def generateRandomOptions():
    if options.hybrid:
        allflags = "smiH8W"
    else:
        # Maintain an ordering for consistency.
        allflags = "smiHV8WLP"
    flags = ""
    for f in allflags:
        flags += choice(['', f])
    if options.logical:
        flags += choice(['', 'Q'])
    return flags

def generateRandomExtParam(depth, extparam):
    if not extparam:
        return ""
    params = []
    if choice((False, True)):
        params.append("min_length=%u" % randint(1, depth))
    if choice((False, True)):
        params.append("min_offset=%u" % randint(1, depth))
    if choice((False, True)):
        params.append("max_offset=%u" % randint(1, depth*3))
    if choice((False, True)):
        dist = randint(1, 3)
        if choice((False, True)):
            params.append("edit_distance=%u" % dist)
        else:
            params.append("hamming_distance=%u" % dist)
    if params:
        return "{" + ",".join(params) + "}"
    else:
        return ""

parser = OptionParser()
parser.add_option("-d", "--depth",
                  action="store", type="int", dest="depth", default=200,
                  help="Depth of generation (akin to maximum length)")
parser.add_option("-c", "--count",
                  action="store", type="int", dest="count", default=1000,
                  help="Number of expressions to generate")
parser.add_option("-a", "--alphabet",
                  action="store", type="int", dest="alphabet", default=26,
                  help="Size of alphabet to generate character expressions over (starting with lowercase 'a')")
parser.add_option("-i", "--nocase",
                  action="store_true", dest="nocase",
                  help="Use a caseless alphabet for character generation")
parser.add_option("-x", "--extparam",
                  action="store_true", dest="extparam",
                  help="Generate random extended parameters")
parser.add_option("-l", "--logical",
                  action="store_true", dest="logical",
                  help="Generate logical combination expressions")
parser.add_option("-H", "--hybrid",
                  action="store_true", dest="hybrid",
                  help="Generate random flags for hybrid mode")

(options, args) = parser.parse_args()
if len(args) != 0:
    parser.error("incorrect number of arguments")

alphabet = range(ord('a'), ord('a') + options.alphabet)
if options.nocase:
    alphabet += range(ord('A'), ord('A') + options.alphabet)
    
for i in xrange(0, options.count):
    print "%08d:/%s/%s%s" % (i, generateRE(randint(1, options.depth), atTop = True), generateRandomOptions(), generateRandomExtParam(options.depth, options.extparam))

if options.logical:
    for i in xrange(options.count, options.count + 3000):
        print "%08d:/%s/C" % (i, generateCombination(randint(1, options.depth), atTop = True))
