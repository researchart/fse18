# Ecosystem-scale regexp study

Welcome to the FSE'18 artifact for the ESEC/FSE paper *"The Impact of Regular Expression Denial of Service (ReDoS) in Practice: an Empirical Study at the Ecosystem Scale"*, by J.C. Davis, C.A Coghlan, F. Servant, and D. Lee, all of Virginia Tech.

This paper describes a study in which we:
- extracted regular expressions (regexes, regexps) from npm and pypi modules
- analyzed the regexes along several dimensions

Our artifact consists of:
- Code to analyze a regex for super-linear performance (Table 1), degree of vulnerability (Table 2), semantic meaning (Table 3), and use of anti-patterns (Table 4).
- Unique regexes collected from npm and pypi modules. We are releasing these regexes raw (without analysis or source module(s)) due to security concerns.

In addition, we wrote code to statically extract regexes from npm and pypi modules.
We released this code as part of our `vuln-regex-detector` software, available [here](https://github.com/davisjam/vuln-regex-detector).
Regex extraction was uninteresting from a scientific perspective so we do not elaborate on it in this artifact.

In addition to this directory's `README.md`, each sub-tree comes with one or more READMEs describing the software and tests.

## Installation

### By hand

To install, execute the script `./configure` on an Ubuntu 16.04 machine with root privileges.
This will obtain and install the various dependencies (OS packages, REDOS detectors, npm modules, and pypi modules).
It will also initialize submodules.

The final line of this script is `echo "Configuration complete. I hope everything works!"`.
If you see this printed to the console, great!
Otherwise...alas.

### Container

To facilitate replication, we have published a [containerized version](https://hub.docker.com/r/jamiedavis/daviscoghlanservantlee-fse18-regexartifact/) of this project on hub.docker.com.
The container is based on an Ubuntu 16.04 image so it is fairly large.
  
For example, you might run:

```
docker pull jamiedavis/daviscoghlanservantlee-fse18-regexartifact
docker run -ti jamiedavis/daviscoghlanservantlee-fse18-regexartifact
> vim .env
# Set ECOSYSTEM_REGEXP_PROJECT_ROOT=/davis-fse18-artifact/EcosystemREDOS-FSE18
> . .env
> ./full-analysis/analyze-regexp.pl ./full-analysis/test/vuln-email.json
```

## Use

### Environment variables

Export the following environment variables to ensure the tools know how to find each other.
- `ECOSYSTEM_REGEXP_PROJECT_ROOT`
- `VULN_REGEX_DETECTOR_ROOT` (submodule, set it to `ECOSYSTEM_REGEXP_PROJECT_ROOT/vuln-regex-detector`)

See `.env` for examples.

### Analysis phases

Each phase of the analysis is performed by a separate set of tools.
See the description of the directory structure below for a mapping from research questions to directories.

### Running all of the phases at once

The `full-analysis/analyze-regexp.pl` program runs all of the analysis phases on a list of regexes and prints a summary for each regex.
Use this to confirm that the code is installed and working. Whether you think it does something interesting or useful is up to you.

## Directory structure

| File or Directory/    | Description | Relevance to paper |
|:---------------------:|:------------|:-------------------------------------------------------------------------------------:|
| .                     | Introductory content                                      | - |
| README.md             | You're in it                                              | - |
| LICENSE               | Terms of software release                                 | - |
| INSTALL               | "Install instructions"                                    | - |
| STATUS                | Claims of artifact quality                                | - |
| data/                 | All unique regexes we extracted from npm and pypi modules | Used to answer RQs 1-4 |
| vuln-regex-detector/  | Extract regexes from modules and test for vulnerability. (submodule) | answers RQ1 |
| degree-of-vuln/       | What is the degree of vulnerability of this regex?        | answers RQ2 |
| semantic-meaning/     | What meaning does this regex appear to capture?           | answers RQ3 |
| structural-analysis/  | Check whether a regex contains an anti-pattern            | answers RQ4 |
| visualization/        | Used to produce visualizations. Mostly for posterity.     | - |
| containerized/        | Dockerfile for building container | - |
| full-analysis/        | Run each analysis step on a regex.                        | - |

Each directory contains its own README for additional details.

## Style and file formats

### Style

Most of the scripts in this repository are written in Perl.
They tend to write status updates to STDERR and to emit useful output to STDOUT, though the more complex ones use a resultFile instead.

If you have dependencies on other scripts in the repo, require the invoker to define `ECOSYSTEM_REGEXP_PROJECT_ROOT`.
This environment variable should name the location of your clone of this repository.

### File formats

This project uses JSON to describe research data.
Files named `*.json` are generally JavaScript files that contain one JSON object per line.
This makes it easy to do a line-by-line analysis on the objects in the file, even if the file is large.

## Contact person

Contact J.C. Davis at davisjam@vt.edu with any questions.

## Related artifacts

### vuln-regex-detector

As noted earlier, we released software called [vuln-regex-detector](https://github.com/davisjam/vuln-regex-detector) on GitHub.
The vuln-regex-detector project contains:
- static regex extraction from javascript and python software
- super-linear behavior checking (we rely on those tools in this artifact, and load vuln-regex-detector as a submodule)
- a server that responds to "is this regex super-linear?" queries, plus a sample client.

We created an [npm module](https://www.npmjs.com/package/vuln-regex-detector) as a client. This module queries a service we are hosting at Virginia Tech to test regexes for super-linear behavior.

### safe-regex

We recently inherited the widely-used [safe-regex](https://www.npmjs.com/package/safe-regex) module.
We are in the process of incorporating our structural-analysis anti-pattern detection in this module.
