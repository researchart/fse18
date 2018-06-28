This artifact package symsc.zip consists of:
1. The tarball file symsc.tar.gz (https://drive.google.com/drive/folders/1ePWyP_cdwmCzVeWQpa2SqHkBhdL0mrId):
   a. The SymSC symbolic execution tool for the cache side-channel leak detection.
   b. The benchmarks used in Table 3, Table 4 and Table 5 in our paper.
   c. The scripts running these benchmarks for the experimental data collection.
2. The supporting documents such as README.md, INSTALL.md and etc.


Before the evaluation: 
Please prepare a clean Ubuntu 12.04 64-bit Desktop Operating System. You can either download an iso file from http://releases.ubuntu.com/12.04/, or use VirtualBox to run a prepared Ubuntu image from https://www.osboxes.org/ubuntu/.


Assume you’ve installed the Ubuntu system and set the user/pwd as “user/admin”.
Then download the the symsc.tar.gz file and place it into a folder under your ubuntu home directory:
$ cd
$ mkdir symsc
$ cd symsc
$ mv /path/to/symsc.tar.gz /home/user/symsc


Next, please read the INSTALL.md file to install our tool from the tarball file.


After the installation, go to the benchmark directory and compile the benchmarks:
$ csc
$ ../build/gyp_testing_targets
$ ../build/make_all

Some benchmarks have the same name in our paper and they are differentiated with name-suffixes as following:

Benchmark               Directory
AES[6]       ===>      aes-openssl
AES[19]      ===>      aes
CAST5[4]     ===>      cast5-tom
CAST5[19]    ===>      cast5
DES[3]       ===>      des-libgcrypt
DES[19]      ===>      des


I also created five script files (also in the benchmark directory) for the experiment data collection. 
You can directly run them (no parameters required) to generate data tables:

genTable3.sh generates table3.csv which provides the data  for Table 3 in the paper.
genTable4-precise.sh will generate the data (table4-precise.csv) for the “Precise” columns in Table 4.
genTable4-twostep.sh will generate the data (table4-twostep.csv) for the “Two-Step” columns in Table 4.
genTable5-precise.sh will generate the data (table5-precise.csv) for the “Precise” columns in Table 5.
genTable5-twostep.sh will generate the data (table5-twostep.csv) for the “Two-Step” columns in Table 5.


If any execution of a benchmark goes wrong, you can simply enter the corresponding directory and run it individually.
E.g., assume “aes” goes wrong, then you can run it with the following commands: 

$ csc; 
$ cd aes

# for precise approach with fixed-address
$ cp aes.c.precise aes.c
$ ./usc-run  -precise  -fixed  -max-time=96000


# for precise approach with symbolic-address
$ cp aes.c.precise aes.c
$ ./usc-run  -precise  -max-time=96000


# for two-step approach with fixed-address
$ cp aes.c.twostep aes.c
$ ./usc-run  -fixed  -max-time=96000


# for two-step approach with symbolic-address
$ cp aes.c.twostep aes.c
$ ./usc-run  -max-time=96000


Note: 
1. -max-time option specifies timeout time for an execution in second. The side-channel analysis is extremely heavy in symbolic-address mode, and I set 1600 minutes as the timeout threshold in experiment.
2. In the provided 5 scripts, you can also freely alternate the -max-time value to set global timeout threshold for each benchmark.
3. Please assign the virtual machine as much computing resources as possible to finish all the executions.


For each execution, our tool generates several log files which contains the execution information:
1. log.precise-fixed: log file for the precise approach with fixed-address.
2. log.precise-symbolic: log file for the precise approach with symbolic-address.
3. log.twostep-fixed: log file for the two-step approach with fixed-address.
4. log.twostep-symbolic: log file for the two-step approach with symbolic-address.
5. log.Inter: the number of the generated interleavings.
6. log.Fail: the number of failed 1st-step tries in the two-step approach.
7. log.Test: the number of valid tests.
8. log.maxaccs: the actual number of memory accesses during execution.
9. log.time: the execution start/end time.
10. log.ks: the size of the symbolic key in execution.


All the benchmarks are pre-configured with gyp and the path of the global gyp file is:
/path/to/cloud9_root/src/testing_targets/build/all_llvm.gyp


