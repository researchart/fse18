#!/usr/bin/env python
# Author: Jamie Davis <davisjam@vt.edu>
# Description:
#   Internally we maintain non-anonymized mappings from patterns to the modules that use them, and vice versa.
#   This program accepts a pattern2modules file and remove the module names.
#   The effect is to "anonymize" the regexes.

import argparse
import json

parser = argparse.ArgumentParser(description='Anonymize patterns')
parser.add_argument('--file', '-f', help='Remove the "2modules" part of a "pattern2modules" file', required = True)
args = parser.parse_args()

pattern2modulesFile = args.file

with open(pattern2modulesFile, 'r') as f:
	for line in f:
		info = json.loads(line)
		obj = { "pattern": info['pattern'] }
		print json.dumps(obj)
