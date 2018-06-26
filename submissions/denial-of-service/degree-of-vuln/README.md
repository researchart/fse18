# Summary

Curve fitting tools.

Used to answer RQ2. Results in Table 2.

# Files

1. `estimate-blowup-curve.pl`
  Helper for analyze-regexp.pl.
	Call with a particular {language x pattern x evilInput} combination.
  Generates "curve" objects: type [parms r2].
	This object is described in the opening comments of `fit-curve.py`.

2. `fit-curve.py`
	Takes a CSV of 'pumps,time' and tries to fit exponential and polynomial curves
	to predict time as a function of pumps.
	See sample files in test/
