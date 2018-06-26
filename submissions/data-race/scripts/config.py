#! /usr/bin/env python

base_folder = "../"
# You can change this to any other directory
# for which you have write permissions
test_folder = "../logs/"
javaMemory = 30000
javaStack = 1000
sequitur_memory = 10000

rapid_classpath = base_folder + "rapid/bin:" + base_folder + "rapid/lib/*"
ziptrack_classpath = base_folder + "ziptrack/bin:" + base_folder + "ziptrack/lib/*"
sequitur_command = base_folder + "/sequitur/sequitur"

small_benchmarks = ['account', 'airlinetickets', 'array', 'boundedbuffer', 'bufwriter', 'bubblesort', 'critical', 'mergesort', 'pingpong']
medium_benchmarks = ['moldyn', 'montecarlo', 'raytracer']
large_misc_benchmarks = ['derby', 'ftpserver', 'jigsaw']
large_dacapo_benchmarks = ['xalan', 'lusearch', 'eclipse']

benchmarks = small_benchmarks + medium_benchmarks + large_misc_benchmarks + large_dacapo_benchmarks

testclass_map = {
        'mergesort': 'mergesort.MergeSort', 'account': 'account.Account', 'pingpong': 'pingpong.PingPong', 'bufwriter': 'bufwriter.BufWriter', 'bubblesort': 'bubblesort.BubbleSort', 'critical': 'critical.Critical', 'boundedbuffer': 'boundedbuffer.BoundedBuffer', 'array': 'array.Test', 'airlinetickets': 'airlinetickets.Airlinetickets'
        ,'montecarlo': 'JGFMonteCarloBenchSizeA', 'moldyn': 'JGFMolDynBenchSizeA', 'raytracer': 'JGFRayTracerBenchSizeA'
        ,'ftpserver': 'driver.FTPMainDriver', 'jigsaw': 'driver.JigsawDriver', 'derby': 'derby2861.Derby2861'
}
large_misc_classpath = {'derby':'../jar/derby.jar', 'ftpserver':'../jar/ftpserver.jar:../lib/lib_ftpserver/*', 'jigsaw':'../jar/jigsaw.jar:../lib/lib_jigsaw/*'}

rvjar = '../RV-Predict/rv-predict.jar'
logger = 'com.runtimeverification.rvpredict.util.ReadLogFile'
rvpredict = '../RV-Predict/bin/rv-predict'

small_jar = '../jar/small.jar'
medium_jar = '../jar/medium.jar'
dacapojar='../jar/dacapo-9.12-bach.jar'
numThreads='5'
size = 'small'

print_class = 'PrintTrace'
grammar_transform_class = 'TransformGrammar'
ziptrack_HB_class = 'ZipHB'
ziptrack_LockSet_class = 'ZipLockSet'
ziptrack_MetaInfo_class = 'ZipMetaInfo'
HB_class = 'HB'
HBEpoch_class = 'HBEpoch'
LockSet_class = 'LockSet'
Goldilocks_class = 'Goldilocks'
MetaInfo_class = 'MetaInfo'