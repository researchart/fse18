#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in benchmarks:
        ind = benchmarks.index(sys.argv[1])
        benchmarks = [benchmarks[ind]]

cls = ziptrack_HB_class
clspath = ziptrack_classpath

command_pre = ['java', '-Xmx' + str(javaMemory) + 'm', '-Xms' + str(javaMemory) + 'm', '-classpath', clspath]
command_pre = command_pre + [cls]
ld = test_folder + 'bin/'
ad = test_folder + cls + '/'
make_dir(ad)
for test in benchmarks:
	ad_test = ad + test + '/'
	make_dir(ad_test)
	binpath 	= ld + test + '/'
	tracefile 	= binpath + 'grammar.txt'
	mapfile 	= binpath + 'map.txt'
	command 	= command_pre + ['-m', mapfile, '-t', tracefile]
	outfile 	= ad_test + cls + '.txt'
	errfile 	= ad_test + cls + '.err'
	print("Ziptrack's HB analysis for " + test + ' ... ' )
	call(command, stdout = open(outfile, 'w'), stderr = open(errfile, 'w'))