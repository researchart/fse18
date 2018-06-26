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
ld_std = test_folder + 'std/'
for test in benchmarks:
    binpath     = ld + test + '/'
    stdpath     = ld_std + test + '/' 
    make_dir(binpath)
    make_dir(stdpath)

    outfile = stdpath + 'rvtrace.out'
    errfile = stdpath + 'rvtrace.err'
    timfile = stdpath + 'rvtrace.tim'

    if (test in small_benchmarks):
        command = command = [rvpredict, '--log',  '--base-log-dir', ld, '--log-dirname', test, '-cp', small_jar, testclass_map[test]]
    elif (test in medium_benchmarks):
        command = command = [rvpredict, '--log',  '--base-log-dir', ld, '--log-dirname', test, '-cp', medium_jar, testclass_map[test]]
    elif (test in large_misc_benchmarks):
        command = ['java', '-cp', '.:' + large_misc_classpath[test], '-javaagent:' + rvjar + '="--log  --base-log-dir '+ ld + ' --log-dirname ' + test + '"', testclass_map[test]]
    elif (test in large_dacapo_benchmarks):
        command = ['java', '-javaagent:' + rvjar + '="--log  --base-log-dir '+ ld + ' --log-dirname ' + test + '"', '-jar', dacapojar, '-t', numThreads, '--size', size ,test]

    print("Logging " + test + ' ... ' )
    start_time = time.time()
    call(command, stdout = open(outfile, 'w'), stderr = open(errfile, 'w'))
    end_time = time.time()
    with open(timfile, 'w') as tmfl:
        tmfl.write("====== " + str(end_time - start_time) + " seconds elapsed ======\n" )