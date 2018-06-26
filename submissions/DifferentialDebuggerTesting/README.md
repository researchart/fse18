# Differential Debugger Testing

This is our implementation of _differential testing of interactive debuggers_, specifically for the JavaScript debuggers 
of Firefox and Chromium. 

Differential testing is an automatic testing technique where generated inputs are given to two different 
programs whose behavior is then compared. We apply this idea to debuggers, where the inputs are a _program-to-debug_
and generated _debugging actions_, e.g., "set breakpoint in line X" or "step into" and the compared _debugger behavior_ 
is, for example, where the debugger paused or the names and values of local variables.

For more details on our approach, see our paper (to appear):

    Feedback-Directed Differential Testing of Interactive Debuggers  
    Daniel Lehmann and Michael Pradel  
    The 26th ACM Joint European Software Engineering Conference and Symposium on the Foundations of Software Engineering (ESEC/FSE 2018)

# Directory Structure and Overview

This repository is structured into several subdirectories, some of which are only created upon building the tool or 
during execution (i.e., when the automated tests on Firefox and Chromium are performed).

- ```./```:  At the top level you find our implementation as an NPM project (see the ```package.json``` file and the 
```yarn.lock``` file which captures the dependencies' exact version numbers), this readme, 
installation instructions (```INSTALL.md```) and a script to install all prerequisites (```install.sh```, tested for Ubuntu 18.04 LTS), and 
the artifact status for which we apply for the FSE artifact evaluation (```STATUS```).

- ```browsers/```: This directory contains a ```download-and-extract.sh``` script you can use this to get the exact 
versions of Firefox and Chromium we used for testing. They will be downloaded and extracted into this directory and not 
installed system wide (i.e., your installations of Chromium and Firefox are unaffected).  
It also contains a pre-setup Firefox profile ```firefox-profile/prefs.js``` which (among other things) enables the 
remote debugging protocol. The browsers are run with temporary profiles, i.e., should not affect your settings of the
locally installed browsers.

- ```bugreports/```: In this directory we collected all bug reports we filed in the bug trackers of Firefox and 
Chromium. In particular, the ```README.md``` lists all bugs with a short description/intuition when/how it manifests, a
link to the actual issue entry, and its status (one of _Reported_/_Confirmed_/_Fixed_).  
For each of the listed bugs, there is also a subdirectory, e.g., ```bugreports/ff-pauses-dead-code/```. Each 
subdirectory contains a **minimal example JavaScript program** (embedded into a HTML document) that showcases the bug in 
the affected browser versions. Additionally, each directory contains a **screen-captured video** that demonstrates the
bug in the debugger GUI. This helps in understanding how the bug misrepresents the program execution and how it thus
might confuse a developer.  
Note that, as described in the paper, between automatic finding of differences between the debuggers 
(i.e., the differential testing implemented by our tool) and filing of the bug reports some manual effort was involved; 
namely condensing the program-to-debug into a small script, ideally with a short example where the bug appears vs. 
a short example where it doesn't but that is very similar.
This is one aspect where we would like to further improve/automate our approach.

- ```build/``` is created when compiling our implementation (generate JavaScript from the 
TypeScript source files). To run the tool, execute ```build/eval/run-tests.js``` with Node.js (also see the installation 
instructions).

- ```node_modules/``` is created by the NPM or yarn package managers when installing our tool and 
contains all runtime/build dependencies (e.g., the Chrome Debugging Protocol implementation in JavaScript, 
or the TypeScript compiler).

- ```programs-to-debug/``` contains the (fixed, not generated) set of JavaScript programs we used for testing the 
debuggers (see our paper). It contains three subdirectories, one per group of programs-to-debug
    * ```sunspider-determ/```: The largest group (26 programs), taken from the SunSpider benchmarks, version 1.0.2.   
    Since some SunSpider benchmarks use non-deterministic operations that would show up as differences when comparing
    the debugger outputs (e.g., ```string-validate-input.js```), we fixed these sources of non-determinism. The changes
    are minor, namely replacing ```Math.random()``` with a deterministic function and constructing a fixed ```Date``` 
    instead of ```Date.now()```.
    * ```pta-warm-up-quizzes/```: JavaScript puzzlers, taken from the _program testing and analysis_ 
    lecture at TU Darmstadt.   
    The intuition behind these programs is, that if it is challenging for JavaScript users to understand the
    runtime behavior of those puzzlers, it might also be a source of confusion for developers of the debuggers. And we 
    have in fact found a bug by generating debugger actions on these programs, namely 
    [Firefox bug 1362432](https://bugzilla.mozilla.org/show_bug.cgi?id=1362432). It appears in some cases when 
    generating actions for ```9.js```, where a function appears with multiple parameters of the same name.
    * ```manual/```: Programs we wrote ourselves, mostly to test JavaScript features beyond EcmaScript 5 
    (since SunSpider predates that standard and thus does not contain code with these language features).

- ```results/```: This directory is generated upon execution of our tool and contains the output of the testing. It 
contains one subdirectory per set of testing parameters, e.g., ```results/breakpointsPerLine0.1-breakpointRemoveProb0.2-maxActions20/```
with the default setting of 0.1 breakpoints per line, a probability of 0.2 of removing a just-set breakpoint, and a 
maximum number of 20 actions (i.e., continue or steps).  
Inside, there are again directories for each program-to-debug, e.g., ```sunspider-determ/3d-cube/``` corresponding to
the JavaScript test program in ```/programs-to-debug/sunspider-determ/3d-cube.js```. These directories finally contain
the traces in the format described in the paper, e.g., the recorded trace of the first generated test is ```seed0.log``` 
and contains (excerpt)
    ```
    ...
    set breakpoint @ line 309
      Chrome:  set @ line 311, id: 31:308:0
      Firefox: set @ line 311, id: server1.conn0.child1/breakpoint39
    remove breakpoints from previous step
      Chrome:  removed @ line 311, id: 31:308:0
      Firefox: removed @ line 311, id: server1.conn0.child1/breakpoint39
    ...
    set breakpoint @ line 275
      Chrome:  set @ line 279, id: 31:274:0
      Firefox: set @ line 275, id: server1.conn0.child1/breakpoint44
    breakpoint difference
    ```
    which indicates that a difference in the location of a set breakpoint was found between Firefox and Chromium.  
    An overview of all runs performed with the given parameter set, can be found in ```runs.csv```. This also contains 
    a column for the type of difference that was found (cf. Figure 7 in the paper).  
    Note again that some manual efforts were involved in filing bug reports out of these found warnings. The basic workflow
    for this part is opening a trace, looking for the affected lines in the program-to-debug and the executed actions,
    reproducing the difference in the debugger GUIs of Firefox and Chromium, and then manually minimizing the 
    program-to-debug to find the root cause. See ```bugreports/``` for the bug reports we filed in the official issue trackers.

- ```src/```: Finally, this contains our implementation in TypeScript. Its subdirectories are:
    * ```debuggers/```, code for communicating with the browsers through their respective remote debugging protocols.
    For Chromium, we can use the official JavaScript implementation (NPM package ```chrome-remote-interface```), but for
    Firefox, we had to implement the debugging protocol ourselves (see ```firefox-protocol.ts```). Since the official
    documentation is sometimes incomplete or out of date (https://wiki.mozilla.org/Remote_Debugging_Protocol), 
    this might be a useful resource for others trying to interface with the Firefox debugger through its RDP.  
    ```interface.ts``` contains our common abstraction over the debugger protocols of Firefox and Chromium. 
    On top of this interface, we issued the debugger actions mentioned in the paper. 
    * ```utils/``` contains miscellaneous code for random number generation, working with JavaScript promises (since 
    the debugger protocols are asynchronous APIs, see ```async```/```await```), and generating the debugger 
    traces.
    * ```eval/``` the actual code for generating the debugging actions, loading the ```programs-to-debug```, and 
    comparing the debugger behavior between Firefox and Chromium. In particular, run ```run-tests.js``` with Node.js to
    start the testing (and modify the set of parameters ```paramsValues``` if you want to run with a different number
    of breakpoints or actions). Also, by editing ```testcases.ts``` you can add/change the programs-to-debug for which actions are generated.

# Installation

Please check INSTALL.md for instructions.

# License, Usage, and Modification

This project is licensed under the permissive open-source MIT license, see LICENSE.txt for the full license. If you 
have any questions on how our tool works, on how to get it to run, or how to modify it for your own purposes, please
write us a mail at ```daniel.lehmann@crisp-da.de```. We are happy to help!
