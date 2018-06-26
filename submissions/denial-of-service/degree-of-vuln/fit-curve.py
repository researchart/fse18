#!/usr/bin/env python2
# Author: Jamie Davis <davisjam@vt.edu>
# Description:
#   For a dataset exploring the performance of an algorithm,
#   estimate the algorithm's performance.
#
#   The method is to try power-law (f = a*x^b) and exponential (f = a*e^bx) curves,
#     and report the one with better accuracy (larger r^2 value).
#
# I/O:
#  Input: CSV with nPumps,matchTime
#  Output: JSON with keys type [parms r2]
#    type is: "UNKNOWN", "POWER", "EXP"
#    parms is a,b for the curve
#    r2 is the r squared value

################
# Imports
################

# Math
import scipy.optimize
import numpy
import numbers

# Arg parsing
import sys

# I/O
import csv # I
import json # O

################
# Utility
################

# f is a vectorized function: f(x, parms)
# xs and ys are data
# r^2 = 1 - SS_res/SS_tot
# cf. https://en.wikipedia.org/wiki/Coefficient_of_determination
def rSquared(f, parms, xs, ys):
	# ss_res
	residuals = ys - f(xs, parms[0], parms[1])
	ss_res = numpy.sum(residuals**2)

	# ss_tot
	y_mean = numpy.mean(ys)
	ss_tot = numpy.sum((ys - y_mean)**2)

	r_squared = 1 - (ss_res / ss_tot)
	return r_squared

# Curves. These evaluate the associated curve.

def powerCurve(x, a, b):
	if isinstance(x, numbers.Number):
		return a * (x**b)
	else: # vector
		return numpy.asarray([powerCurve(_, a, b) for _ in x])

def expCurve(x, a, b):
	if isinstance(x, numbers.Number):
		return a * numpy.exp(b*x)
	else: # vector
		return numpy.asarray([expCurve(_, a, b) for _ in x])

# Fits. Return (optimalParms, cov-of-popt, r-squared)

def fitPowerCurve(xs, ys):
	# Guess p0 from one point
	p0_guess = [1, numpy.log(ys[-1]) / numpy.log(xs[-1])]
	# Guess p0 from two points
	try:
		twoXs = [xs[i] for i in [0, -1]]
		twoYs = [ys[i] for i in [0, -1]]
		p0_guess, _ = scipy.optimize.curve_fit(powerCurve, twoXs, twoYs)
	except RuntimeError:
		pass
	log("fitPowerCurve: p0_guess {}".format(p0_guess))

	popt, pcov = scipy.optimize.curve_fit(powerCurve, xs, ys, p0=p0_guess)
	return popt, pcov, rSquared(powerCurve, popt, xs, ys)

def fitExpCurve(xs, ys):
	# Guess p0 from one point
	p0_guess = [1, numpy.log(ys[-1]) / xs[-1]]
	# Guess p0 from two points
	try:
		twoXs = [xs[i] for i in [0, -1]]
		twoYs = [ys[i] for i in [0, -1]]
		p0_guess, _ = scipy.optimize.curve_fit(expCurve, twoXs, twoYs)
	except RuntimeError:
		pass
	log("fitExpCurve: p0_guess {}".format(p0_guess))

	popt, pcov = scipy.optimize.curve_fit(expCurve, xs, ys, p0=p0_guess)
	return popt, pcov, rSquared(expCurve, popt, xs, ys)

def main(dataFile):
	xs, ys = dataFile2data(dataFile)

	log("xs {}".format(xs))
	log("ys {}".format(ys))

	couldBePow = True
	try:
		pow_parms, pow_pcov, pow_r2 = fitPowerCurve(xs, ys)
		log("pow: parms {}, r2 {}".format(pow_parms, pow_r2))
	except RuntimeError:
		couldBePow = False

	couldBeExp = True
	try:
		exp_parms, exp_pcov, exp_r2 = fitExpCurve(xs, ys)
		log("exp: parms {}, r2 {}".format(exp_parms, exp_r2))
	except RuntimeError:
		couldBeExp = False

	curveType = ''
	parms = []

	# If the curve had to pick goofy numbers a lot, disqualify it.
	minLegitimateParmValue = 1e-10
	if couldBePow:
		if min([abs(_) for _ in pow_parms]) < minLegitimateParmValue:
			couldBePow = False
	if couldBeExp:
		if min([abs(_) for _ in exp_parms]) < minLegitimateParmValue:
			couldBeExp = False

	# Make decision based on (1) legitimacy of parms, and (2) maximum r^2 value.
	result = {}
	curve = "UNKNOWN"
	if couldBePow or couldBeExp:
		if couldBeExp and couldBePow:
			if exp_r2 < pow_r2:
				curve = "POWER"
			else:
				curve = "EXP"
		elif not couldBeExp:
			curve = "POWER"
		elif not couldBePow:
			curve = "EXP"

	if curve == "POWER":
		parms = pow_parms
		r2 = pow_r2
	elif curve == "EXP":
		parms = exp_parms
		r2 = exp_r2
	else:
		curve = "UNKNOWN"
		parms = []
		r2 = -1

	result['type'] = curve
	if curve != "UNKNOWN":
		# numpy parms format is not fixed??
		if type(parms) == type([]):
			result['parms'] = parms
		else:
			result['parms'] = numpy.ndarray.tolist(parms)

	result['r2'] = r2

	print(json.dumps(result))

def dataFile2data(dataFile):
	xs = []
	ys = []
	with open(dataFile, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		for _x, _y in reader:
			try:
				x = int(_x)
				y = float(_y)
				xs.append(int(x))
				ys.append(float(y))
			except ValueError:
				pass

	if not len(xs) or not len(ys) or len(xs) != len(ys):
		print("Error, no data in {}".format(dataFile))
		sys.exit(1)

	return xs, ys

def log(msg):
  sys.stderr.write(msg + '\n')

########################

# Arg parsing
if len(sys.argv) != 2:
	print("Error, usage: {} pumpsToTime.csv".format(sys.argv[0]))
	sys.exit(1)

dataFile = sys.argv[1]

# Here we go!
main(dataFile)
