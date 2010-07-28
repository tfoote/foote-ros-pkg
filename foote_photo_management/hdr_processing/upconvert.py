#!/usr/bin/env python

import subprocess
import os

from optparse import OptionParser


def run(cmd_str):
    cmd = cmd_str.split()
    print "Running cmd: %s"%cmd
    subprocess.check_call(cmd)



parser = OptionParser()
parser.add_option("-o", "--outdir", dest="outdir",
                  help="write files to outdir", metavar="DIRECTORY")
#parser.add_option("-q", "--quiet",
#                  action="store_false", dest="verbose", default=True,
#                  help="don't print status messages to stdout")

(options, args) = parser.parse_args()

if not options.outdir:
    parser.warn("Using outdir of '.'")

os.makedirs(options.outdir)

for a in args:
    if not os.path.exists(a):
        print "Argument '%s' invalid"%a
        continue
    basename = os.path.basename(a)
    (filename, ext) = os.path.splitext(basename)
    outfile = "%s"%os.path.join(options.outdir, filename+".tiff")
    cmd = "convert %s %s"%(a, outfile)
    run(cmd)

    cmd = "exiftool -tagsfromfile %s -overwrite_original_in_place -exif:all %s"%(a, outfile)
    #run(cmd)
