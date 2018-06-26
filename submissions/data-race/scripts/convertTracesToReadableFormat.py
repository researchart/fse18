#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in benchmarks:
        ind = benchmarks.index(sys.argv[1])
        benchmarks = [benchmarks[ind]]

command_pre = ["java", '-Xmx' + str(javaMemory) + 'm', '-Xms' + str(javaMemory) + 'm',"-classpath", ziptrack_classpath]
folder_prefix = 'bin/'

ld = test_folder + folder_prefix
for test in benchmarks:
	path = ld + test + '/'
	command = command_pre + [print_class]
	command = command + ["-p="+path]

	outfile = path + 'trace.txt'
	errfile = path + 'trace.err'
	timfile = path + 'trace.tim'

	print("Printing trace for " + test + ' ... ')
	start_time = time.time()
	call(command, stdout = open(outfile, 'w'), stderr = open(errfile, 'w'))
	end_time = time.time()
	with open(timfile, 'w') as tmfl:
		tmfl.write("====== " + str(end_time - start_time) + " seconds elapsed ======\n" )
	call(['mv'] + [path + "map.shared.txt"] + [path + "map.txt"])
