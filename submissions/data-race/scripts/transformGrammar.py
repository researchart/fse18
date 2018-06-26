#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in benchmarks:
        ind = benchmarks.index(sys.argv[1])
        benchmarks = [benchmarks[ind]]

cls = grammar_transform_class
clspath = ziptrack_classpath

command_pre = ['java', '-Xmx' + str(javaMemory) + 'm', '-Xms' + str(javaMemory) + 'm', '-classpath', clspath]
command_pre = command_pre + [cls]
ld = test_folder + 'bin/'
for test in benchmarks:
    binpath     = ld + test + '/'
    tracefile   = binpath + 'grammar.txt'
    mapfile     = binpath + 'map.txt'
    command     = command_pre + ['-m', mapfile, '-t', tracefile]
    outfile = binpath + 'grammar.new.txt'
    print("Transforming grammar for " + test + ' ... ' )
    call(command, stdout = open(outfile, 'w'))
    print(" ".join(command))
    call(['mv', tracefile, binpath + 'original_grammar.txt'])
    call(['mv', outfile, tracefile])