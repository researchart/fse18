#! /usr/bin/env python

import sys
import os
import time
from subprocess import call
import argparse
import ast

from config import *

def get_time_command(timfile):
	# return ['/usr/bin/time', '-v', '-o', timfile]
	return ['time']

def make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)
