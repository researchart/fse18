# TL;DR

Run the following commands on your Unix based machine:

```
export export rvpredict=`pwd`/RV-Predict

cd sequitur; make sequitur; cd ..
export sequitur=`pwd`/sequitur

cd rapid; ant; cd ..
export rapid=`pwd`/rapid

cd ziptrack; ant; cd ..
export ziptrack=`pwd`/ziptrack
```

Download `dacapo-9.12-bach.jar` from `https://uofi.box.com/v/dacapojar` and put it in the folder `jar` . 

# Longer version:

## Software Requirements :

	1. Machine with Unix based operating system. The artifact has been tested on a Mac OS.
	2. Java-1.8 or higher.
	3. Apache Ant - For compiling the provided source code.
	4. Python 2.7 or higher.

## Installation

The installation steps can be briefly outlined below:
	
	1. Obtain RVPredict
	2. Obtain and compile Sequitur
	3. Obtain and compile ZipTrack
	4. Obtain and compile RAPID
	5. Obtain `dacapo-9.12-bach.jar`.

### RVPredict

RVPredict is a commercial software tool to detect data races in concurrent programs.

For ESEC/FSE-2018 Artifact reviewers, we provide a copy of the commercial software 
RVPredict with academic license, and no additional installation step is required.

**Note**: 
The copy of the software provided is only meant to evaluate our artifact and should not be redistributed or modified.
A fresh copy of the software with appropriate license can be obtained from [RVPredict's website](https://runtimeverification.com/predict/).

Export the RV-Predict's base directory to your bash environment:
```
export rvpredict=`pwd`/RV-Predict
```

### Sequitur

Sequitur, proposed by Craig G. Nevill-Manning and Ian H. Witten, is an algorithm to compress
strings as SLPs (straight line programs). The provided software implements this algorithm.

For ESEC/FSE-2018 Artifact reviewers, we provide a copy of Sequitur, 
originally obtained from [Sequitur's GitHub repository](https://github.com/craignm/sequitur)
Our copy of Sequitur is slightly modified from the original version.

In order to build the binaries, execute the following commands:
```
cd sequitur
make sequitur
cd ..
```

Also export sequitur's path:
```
export sequitur=`pwd`/sequitur
```

### RAPID

RAPID is an open-source tool that implements some of the common race dynamic analysis techniques for concurrent programs.

For ESEC/FSE-2018 Artifact reviewers, we provide a copy of RAPID, 
originally obtained from [RAPID GitHub repository](https://github.com/umangm/rapid).

In order to build the project, execute the following commands:
```
cd rapid
ant
cd ..
```

Also export RAPID's path:
```
export rapid=`pwd`/rapid
```

### ZipTrack

ZipTrack analyzes traces compressed as an SLP and determines the 
presence of HB races and violations of lockset principle.

For ESEC/FSE-2018 Artifact reviewers, we provide a copy of ZipTrack. 
ZipTrack is also available as a free open-source [GitHub repository](https://github.com/umangm/ziptrack).

In order to build the project, execute the following commands:
```
cd ziptrack
ant
cd ..
```

Also export ZipTrack's path:
```
export ziptrack=`pwd`/ziptrack
```

### Dacapo Jar

The dacapo jar file `dacapo-9.12-bach.jar` is a large file (more than 150 MB) and the FSE Artifact Evaluation Trak Chairs recommended that it be removed from the package.

The jar file can be downloaded from [this link](https://uofi.box.com/v/dacapojar).
After it has been downloaded, it should be moved to the `jar` directory.