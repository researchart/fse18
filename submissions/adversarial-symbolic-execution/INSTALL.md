
Assume your Ubuntu user account/password is user/admin, and the tarball file symsc.tar.gz is downloaded to /home/user/symsc. 
Next, please use the following steps to compile our tool before running the experiments.


1. Install third-party packages
sudo apt-get install -y dejagnu flex bison protobuf-compiler libprotobuf-dev libboost-thread-dev libboost-system-dev build-essential libcrypto++-dev libfreetype6-dev gawk


2. Install glog
tar xzvf symsc.tar.gz
export CLOUD9_ROOT=/home/user/symsc/cloud9_root
cd /home/user/symsc/glog-0.3.3
./configure 2>&1 | tee -a $CLOUD9_ROOT/build.log
make -j3 2>&1 | tee -a $CLOUD9_ROOT/build.log
sudo make install 2>&1 | tee -a $CLOUD9_ROOT/build.log
sudo ldconfig 2>&1 | tee -a $CLOUD9_ROOT/build.log


Note: 
a. Please use the path information on your own computer to update the scripts in this document accordingly.
b. Here I use make -j3 on my computer, and please use a proper number in terms of the CPU cores on your own computer.


3. Install binutil-gold
cd $CLOUD9_ROOT/src/third_party
mkdir binutils-install
cd binutils
./configure --prefix=/home/user/symsc/cloud9_root/src/third_party/binutils-install --enable-gold --enable-plugins
make all -j3
make install
export BINUTILS_DIR=/home/user/symsc/cloud9_root/src/third_party/binutils
export BINUTILS_BUILD_DIR=/home/user/symsc/cloud9_root/src/third_party/binutils-install
rm -f ${BINUTILS_BUILD_DIR}/bin/ld
ln ${BINUTILS_BUILD_DIR}/bin/ld.gold ${BINUTILS_BUILD_DIR}/bin/ld


4. Build llvm
cd $CLOUD9_ROOT/src/third_party
mkdir llvm-build
cd llvm-build
../llvm/configure  --enable-optimized  --enable-assertions --with-binutils-include="$(readlink -f ../binutils-install/include)"
make -j3

Note: you can take a break here, since building llvm always takes a lot of time.


5. Build the uClibc:
cd $CLOUD9_ROOT/src/klee-uclibc
echo "Build Klee's uClibc:" 2>&1 | tee -a $CLOUD9_ROOT/build.log
make -j3 2>&1 | tee -a $CLOUD9_ROOT/build.log


6. Build STP:
cd $CLOUD9_ROOT/src/third_party/stp
echo "Build STP:" 2>&1 | tee -a $CLOUD9_ROOT/build.log
./scripts/configure --with-prefix=$(pwd) 2>&1 | tee -a $CLOUD9_ROOT/build.log
make -j3 2>&1 | tee -a $CLOUD9_ROOT/build.log


7. Build Cloud9 itself:
cd $CLOUD9_ROOT/src
echo "Build Cloud9 itself:" 2>&1 | tee -a $CLOUD9_ROOT/build.log
make -j3 2>&1 | tee -a $CLOUD9_ROOT/build.log


cd $CLOUD9_ROOT/src/cloud9
./configure --with-llvmsrc=../third_party/llvm --with-llvmobj=../third_party/llvm-build --with-uclibc=../klee-uclibc --enable-posix-runtime --with-stp=../third_party/stp  CXXFLAGS="-I$CLOUD9_ROOT/src/cloud9/include"
make -j3


8. Update the environment variables
Append the following three lines to ~/.bashrc:
export PATH=$PATH:”/home/user/symsc/cloud9_root/src/cloud9/Release+Asserts/bin”
export PATH=$PATH:”/home/user/symsc/cloud9_root/src/third_party/llvm-build/Release+Asserts/bin”
alias csc=”cd /home/user/symsc/cloud9_root/src/testing_targets/cache_side_channel”


And reload the terminal profile by the following command:
source ~/.bashrc


Now, you can test klee with the --help option:
$ klee --help
If everything works correctly you will see a list of klee options.
