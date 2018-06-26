# Summary

This directory has tools that attempt to define the language that a regex matches.
They use heuristics based on manually categorized regexes.

The methodology for this project is in $4.3 RQ3.
The results are visualized in Figure 5.

# Testing

I created a bunch of sample inputs starting from a hand-labeled dataset (from my spreadsheet 'Discovered REDOS vulnerabilities').
I've added to them in each iteration when I see things that are mis-classified.

To check the classification performance, run the test tool:

```bash
./run-tests.pl 2>/dev/null
```

# References

An alternative approach to categorization is using Microsoft's "Rex" tool
to generate matching input, and then try to figure out what the language is.

[Rex](https://www.microsoft.com/en-us/research/project/rex-regular-expression-exploration/)
