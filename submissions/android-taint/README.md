# ReproDroid
<p align="center">
	<img src="doc/images/logo.png" width="500px"/>
</p>

ReproDroid is a framework which can be used to create, refine and execute reproducible benchmarks for Android app analysis tools.

In the paper, the framework has been used to check if certain promises for analysis tools hold.
Therefore, six different analysis tools (*Amandroid, DIALDroid, DidFail, DroidSafe, FlowDroid, IccTA*) where evaluated on three different benchmarks (*DroidBench, ICCBench, DIALDroidBench*) as well as on some additional benchmark cases contributed along with ReproDroid.

All results are described in the paper.
Furthermore, the result data is publicly available and ready for download: [https://FoelliX.github.io/ReproDroid](https://FoelliX.github.io/ReproDroid)

## Toolchain
The complete ReproDroid framework consists of BREW and its underlying AQL-System which uses the AQL.
The picture below summarizes how the framework works.
BREW takes a set of apps or a complete benchmark as input and issues one AQL-Query per benchmark case.
Then, one query after another arrives at an AQLSystem which produces one AQL-Answer per query.
To do so, it uses analysis tools specified in BREW's configuration file.
All AQL-Answers are gathered by BREW. Based on these answers a final report for e.g. a benchmark is carried out.

<p align="center">
	<img src="doc/images/overview.jpg" width="800px"/>
</p>

## Installation & Usage
Instructions on how to install, test and use ReproDroid as well as instructions on how to reproduce/review paper results can be found [here](INSTALL.md).

## Badge Application
The badge we appply for as well as the reasons why our artifact may deserve this badge can be found [here](STATUS.md).