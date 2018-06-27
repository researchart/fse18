# Installation

There are two options for running our implementation of differential testing on the JavaScript debuggers of Firefox and Chromium.

## Option A: Virtual Machine

If you just want to execute the tool and reproduce our results, the easiest way is to use a prepackaged virtual machine
with Ubuntu 18.04 LTS, all dependencies installed, and this repository already cloned and built:

1. Download the virtual machine image from https://drive.google.com/open?id=1pmCOrooEjYln3WTOTzqGz9u8XLbeJ_Nm
2. Import into VirtualBox via _File_ > _Import Appliance..._
3. Boot up (user: ```user```, password: ```user```) and open a terminal (Ctrl+Alt+T).
4. Run ```node ~/DifferentialDebuggerTesting/build/eval/run-tests.js```
5. Firefox and Chromium browser windows are opening, new tabs are created, and the console should show output similar to
    ```
    > results/breakpointsPerLine0.1-breakpointRemoveProb0.2-maxActions20/sunspider-determ/3d-cube/seed0.log
    > (more lines...)
    ``` 
6. Abort the testing (Ctrl+C) and inspect the generated traces in ```~/DifferentialDebuggerTesting/results/```. See
the README for an overview and the organization of the test results.

## Option B: Install yourself

We tested the following steps on a fresh minimal installation of Ubuntu 18.04 LTS 64-bit. We also provide a script 
```install.sh``` that performs all the steps below (except for the git clone, since it assumes being run from the repo 
top-level directory).

1. Install the **prerequisites** for building and running our implementation: Git, curl, unzip, tar, Node.js, Yarn, libconf-2-4 (for Chromium)
    ```bash
    # Install Git for cloning our repository
    $ sudo apt install git
    # Verify that it works
    $ git --version
    > git version 2.14.1

    # Install curl, unzip, and tar for downloading the exact versions of Firefox and Chromium we used for evaluation
    $ sudo apt install curl unzip tar
    $ curl --version
    > curl 7.55.1 (x86_64-pc-linux-gnu) ...
    $ tar --version
    > tar (GNU tar) 1.29 ...
    $ unzip -v
    > UnZip 6.00 of 20 April 2009, by Debian. ...

    # Install Node.js for running our implementation.
    $ sudo apt install nodejs
    $ node --version
    > v8.10.0
 
    # Install Yarn, an alternative (to NPM) package manager for JavaScript, for building our implementation.
    $ curl -o- -L https://yarnpkg.com/install.sh | bash
    # Add yarn to your PATH
    $ source ~/.bashrc
    $ yarn --version
    > 1.7.0
 
    # Install libgconf library. The Chromium binary which we will download later (to get the exact same version 
    # we used for our evaluation) doesn't come with it and throws a missing shared library error otherwise.
    $ sudo apt install libgconf-2-4
    ```

2. **Clone** our repository, **install** the JavaScript dependencies, **build** JavaScript from TypeScript.
    ```bash
    $ git clone https://github.com/sola-da/DifferentialDebuggerTesting
    > Cloning into 'DifferentialDebuggerTesting'...
    > ..., done.
 
    $ cd DifferentialDebuggerTesting/
    $ yarn install
    > yarn install v1.7.0
    > [1/5] Validating package.json...
    > [2/5] Resolving packages...
    > [3/5] Fetching packages...
    > [4/5] Linking dependencies...
    > [5/5] Building fresh packages...
    > 
    > Done in 1.66s.
    # You should now have a node_modules/ directory.
    $ ls
    > ... node_modules ...

    $ yarn build
    > yarn run v1.7.0
    > tsc
    > Done in 1.51s.
    # You should now have a build/ directory.
    $ ls
    > ... build ...
    ```

3. Download and extract the exact versions of **Firefox** and **Chromium** we used for our evaluation:
    ```bash
    $ cd browsers/
    $ ./download-and-extract.sh
    >   % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
    >                                  Dload  Upload   Total   Spent    Left  Speed
    > 100 87.7M  100 87.7M    0     0  87.7M      0  0:00:01  0:00:01 --:--:-- 44.1M
    >   % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
    >                                  Dload  Upload   Total   Spent    Left  Speed
    > 100 56.8M  100 56.8M    0     0  56.8M      0  0:00:01  0:00:01 --:--:-- 30.2M
    
    # You should now have chromium-59/ and firefox-54/ directories in browsers/.
    $ ls
    > chromium-59 ... firefox-54 ...

    # Verify that Chromium and Firefox can be started with the remote debugging protocols enabled.
    $ cd ..
    
    $ yarn firefox
    > yarn run v1.7.0
    > $ browsers/firefox-54/firefox -profile browsers/firefox-profile --start-debugger-server ws:6080 -no-remote -url about:blank
    > Started debugger server on 6080
    # A new instance of Firefox should open with an empty tab.
    # You can close it again (through Ctrl+C in the terminal or just by closing the browser window).
    
    $ yarn chromium
    > yarn run v1.7.0
    > $ browsers/chromium-59/chrome --temp-profile --remote-debugging-port=9222 --no-first-run --no-default-browser-check about:blank
    # A new instance of Chromium should open with an empty tab. You can close it as well.
    ```

4. **Run** our implementation. 
It will open Chromium and Firefox, automatically open scripts in both of them for differential testing, 
and generate traces in the ```results/``` directory.
Abort the testing with Ctrl+C.
    ```bash
    # at the top level of the repo, after all dependencies+browsers were installed, and the TypeScript code compiled
    $ node build/eval/run-tests.js
    > results/breakpointsPerLine0.1-breakpointRemoveProb0.2-maxActions20/sunspider-determ/3d-cube/seed0.log
    > (more similar lines...)
    > ^C

5. **Inspect** the resulting traces in the ```results/``` directory and subdirectories.
See README for an explanation of the directory structure.
    ```bash
    # The runs.csv file contains an overview of all executed test runs with their parameters (e.g., how many 
    # breakpoints to set and which program to debug) and results (e.g., which type of difference was found or the runtime).
    $ cat results/breakpointsPerLine0.1-breakpointRemoveProb0.2-maxActions20/runs.csv
    > breakpointsPerLine,breakpointRemoveProb,maxActions,testcase,seed,breakpointCount,resumptionActionCount,result,lastAction,lastLine,lastAstNode,runtimeMs,differenceClass,differenceId,comment
    > 0.1,0.2,20,sunspider-determ/3d-cube,0,16,0,breakpointDifference,setBreakpoint,274,ArrayExpression,702,,,
    > 0.1,0.2,20,sunspider-determ/3d-morph,0,8,4,pauseLocationDifference,stepIn,35,ForStatement,911,,,
    > (more similar lines...)

    # The individual traces are in subdirectories per program-to-debug, e.g.:
    $ cat results/breakpointsPerLine0.1-breakpointRemoveProb0.2-maxActions20/sunspider-determ/3d-cube/seed0.log 
    > (lines omitted ...)
    > set breakpoint @ line 142
    >   Chrome:  set @ line 142, id: 31:141:0
    >   Firefox: set @ line 142, id: server1.conn0.child1/breakpoint43
    > set breakpoint @ line 275
    >   Chrome:  set @ line 279, id: 31:274:0
    >   Firefox: set @ line 275, id: server1.conn0.child1/breakpoint44
    > breakpoint difference
    ```
    
6. **Modify** the program, e.g., by changing some parameters in ```src/eval/run-tests.ts```, 
then recompile with ```yarn build``` and run again.
