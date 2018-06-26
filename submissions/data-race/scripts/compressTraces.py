#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in benchmarks:
        ind = benchmarks.index(sys.argv[1])
        benchmarks = [benchmarks[ind]]

command = [sequitur_command, '-d', '-p', '-m', str(sequitur_memory)]

ld = test_folder + 'bin/'
for test in benchmarks:
	path = ld + test + '/'

	infile 	= path + 'trace.txt'
	outfile = path + 'grammar.txt'
	timfile = path + 'grammar.tim'

	print("Compressing trace for " + test + " ... " )
	start_time = time.time()
	call(command, stdin = open(infile, 'r'), stdout = open(outfile, 'w'))
	end_time = time.time()
	with open(timfile, 'w') as tmfl:
		print("====== " + str(end_time - start_time) + " seconds elapsed ======\n" )
		tmfl.write("====== " + str(end_time - start_time) + " seconds elapsed ======\n" )
