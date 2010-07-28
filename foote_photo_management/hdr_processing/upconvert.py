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
parser.add_option("-i", "--indir", dest="indir",
                  help="read files from indir", metavar="DIRECTORY")
#parser.add_option("-q", "--quiet",
#                  action="store_false", dest="verbose", default=True,
#                  help="don't print status messages to stdout")

(options, args) = parser.parse_args()

if not options.outdir:
    parser.warn("Using outdir of '.'")

if not os.path.exists(options.outdir):
    os.makedirs(options.outdir)

files = []

for a in args:
    if not os.path.exists(a):
        print "Argument '%s' invalid"%a
        continue
    files.append(a)

if options.indir:
    for a in [os.path.join(options.indir, e) for e in os.listdir(options.indir)]:
        if os.path.isfile(a):
            files.append(a)
        else:
            print a, "is not a file"

for f in files:
    basename = os.path.basename(f)
    (filename, ext) = os.path.splitext(basename)
    outfile = "%s"%os.path.join(options.outdir, filename+".tiff")

    try:
        cmd = "convert %s %s"%(f, outfile)
        run(cmd)
        
        cmd = "exiftool -tagsfromfile %s -overwrite_original_in_place -exif:all %s"%(f, outfile)
        run(cmd)
    except CalledProcessError, ex:
        print "Failed %s"%ex
