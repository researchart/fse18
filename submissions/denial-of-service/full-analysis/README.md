# Summary

The `analyze-regexp.pl` script runs each of the analyses in turn and produces a summary.
It sends progress information to stderr so you probably want to redirect stderr to a log file.

The summary is an object with keys:
- superLinear (opinions of various detectors on several variations of the pattern)
- degreeOfVuln (only non-empty if some detector labeled the regex vulnerable)
- semantic (guess the "language" of the regex)
- structural (check for "anti-patterns": QOA, QOD, star height > 1)

Each key maps to an object that follows a reasonably self-explanatory naming convention.

Here is an example (with `REGEX_DEBUG=1` so that the output is pretty-printed).

```
$ ./analyze-regexp.pl test/vuln-email.json 2>/dev/null
{
   "semantic" : {
      "language" : "email",
      "pattern" : ".*@.*"
   },
   "superLinear" : {
      "detectReport" : {
         "pattern" : ".*@.*",
         "memoryLimit" : 8192,
         "detectorOpinions" : [
            {
               "hasOpinion" : 1,
               "secToDecide" : "0.0192",
               "opinion" : {
                  "isSafe" : 1,
                  "canAnalyze" : 1
               },
               "patternVariant" : ".*@.*",
               "name" : "rathnayake-rxxr2"
            },
            {
               "opinion" : {
                  "evilInput" : [
                     {
                        "pumpPairs" : [
                           {
                              "prefix" : "a",
                              "pump" : "@"
                           }
                        ],
                        "suffix" : "a"
                     }
                  ],
                  "isSafe" : 0,
                  "canAnalyze" : 1
               },
               "secToDecide" : "0.6298",
               "hasOpinion" : 1,
               "patternVariant" : ".*@.*",
               "name" : "weideman-RegexStaticAnalysis"
            },
            {
               "name" : "wuestholz-RegexCheck",
               "hasOpinion" : 1,
               "secToDecide" : "0.1854",
               "opinion" : {
                  "canAnalyze" : 1,
                  "isSafe" : 1
               },
               "patternVariant" : ".*@.*"
            },
            {
               "name" : "rathnayake-rxxr2",
               "patternVariant" : "^(.*?).*@.*",
               "secToDecide" : "0.0180",
               "opinion" : {
                  "isSafe" : 1,
                  "canAnalyze" : 1
               },
               "hasOpinion" : 1
            },
            {
               "name" : "weideman-RegexStaticAnalysis",
               "patternVariant" : "^(.*?).*@.*",
               "secToDecide" : "0.9151",
               "opinion" : {
                  "canAnalyze" : 1,
                  "evilInput" : [
                     {
                        "pumpPairs" : [
                           {
                              "prefix" : "a",
                              "pump" : "a"
                           },
                           {
                              "prefix" : "a",
                              "pump" : "@"
                           }
                        ],
                        "suffix" : "a"
                     }
                  ],
                  "isSafe" : 0
               },
               "hasOpinion" : 1
            },
            {
               "name" : "wuestholz-RegexCheck",
               "opinion" : {
                  "evilInput" : [
                     "COULD-NOT-PARSE"
                  ],
                  "isSafe" : 0,
                  "canAnalyze" : 1
               },
               "secToDecide" : "0.1953",
               "hasOpinion" : 1,
               "patternVariant" : "^(.*?).*@.*"
            }
         ],
         "timeLimit" : "60",
         "detectors" : [
            "weideman-RegexStaticAnalysis",
            "wuestholz-RegexCheck",
            "rathnayake-rxxr2"
         ]
      },
      "pattern" : ".*@.*",
      "isVulnerable" : 0
   },
   "pattern" : {
      "language" : "python",
      "pattern" : ".*@.*"
   },
   "degreeOfVuln" : {},
   "structural" : {
      "options" : {
         "countQuestionMarks" : 0,
         "minimumRepetitionUpperLimit" : 25
      },
      "structuralAnalysis" : {
         "anyQOD" : 0,
         "starHeight" : 1,
         "isLinearEngineCompatible" : 1,
         "anyQOA" : 1
      },
      "pattern" : ".*@.*"
   }
}
```

# Testing

There are simple regexes in `test/` for eye-balling.
Each of the analyses has its own test suite so we just want to make sure that `full-analysis.pl` successfully queries them.
