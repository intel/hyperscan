#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import datetime

def usage():
    print("Usage:", os.path.basename(sys.argv[0]), "<seconds from epoch>")

if len(sys.argv) != 2:
    usage()
    sys.exit(1)

ts = sys.argv[1]

build_date = datetime.datetime.utcfromtimestamp(int(ts))

print(build_date.strftime("%Y-%m-%d"))
