#! /usr/bin/env python

from util import *

if len(sys.argv) > 1:
    if sys.argv[1] in traces:
        ind = traces.index(sys.argv[1])
        traces = [traces[ind]]

command_pre = ["java", '-Xmx' + str(javaMemory) + 'm', '-Xms' + str(javaMemory) + 'm','-classpath', tracer_classpath]

for cn in LockSet_class_names.keys():
	class_suffix = LockSet_class_names[cn]
	for tf in testFolders:
		ld = tf + 'bin/'
		ad = tf + 'ls' + '/'
		make_dir(ad)
		for test in traces:
			for singleRace in [True, False]:
				binpath 	= ld + test + '/'
				command 	= command_pre + [cn, '-p=' + binpath, '-f=rv']
				
				outfile = ad + test + '.' + class_suffix
				if singleRace:
					command = command + ['-s']
					outfile = outfile + '.singleRace.'
				else:
					outfile = outfile + '.multipleRace.'
				
				errfile = outfile
				timfile = outfile
				
				outfile = outfile + 'txt'
				errfile = errfile + 'err'
				timfile = timfile + 'tim'
				
				#print(' '.join(command))
				print('LockSet analysis for ' + test + ' (' + class_suffix + ')')
				
				start_time = time.time()
				call(command, stdout = open(outfile, 'w'), stderr = open(errfile, 'w'))
				end_time = time.time()
				
				with open(timfile, 'w') as tmfl:
					tmfl.write("====== " + str(end_time - start_time) + " seconds elapsed ======\n" )