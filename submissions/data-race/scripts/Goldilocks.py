#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in benchmarks:
        ind = benchmarks.index(sys.argv[1])
        benchmarks = [benchmarks[ind]]

cls = Goldilocks_class
clspath = rapid_classpath

command_pre = ['java', '-Xmx' + str(javaMemory) + 'm', '-Xms' + str(javaMemory) + 'm', '-classpath', clspath]
command_pre = command_pre + [cls]
ld = test_folder + 'bin/'
ad = test_folder + cls + '/'
make_dir(ad)
for test in benchmarks:
    binpath     = ld + test + '/'
    command     = command_pre + ['-f=rv', '-p=' + binpath, '-s']
    outfile = ad + test + '.txt'
    errfile = ad + test + '.err'
    print("Goldilocks analysis for " + test + ' ... ' )
    call(command, stdout = open(outfile, 'w'), stderr = open(errfile, 'w'))