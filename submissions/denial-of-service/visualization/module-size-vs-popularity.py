#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import json
from collections import defaultdict
import numpy as np
import matplotlib.ticker
import argparse
import sys
import random

# "random..."
random.seed(1)

def my_log(msg):
  sys.stderr.write(msg + '\n')

# For working with the full-blown JSON modules
class JSONModule:
  fields = None

  def __init__(self, jsonModule):
    self.fields = jsonModule

  def canClone(self):
    return (self.fields['canClone'] == 'yes')

  def getName(self):
    return self.fields['name']

  def getDownloadsPerMonth(self):
    pop = {}
    if 'popularity' in self.fields:
      pop = self.fields['popularity']
    elif 'popularityInfo' in self.fields:
      pop = self.fields['popularityInfo']
    else:
      return -1

    monthlyDownloads = -1
    try:
      monthlyDownloads = int(pop['downloads']['monthly'])
    except Exception:
      pass
    return monthlyDownloads

  def getLoC(self):
    loc = 0

    if 'files' not in self.fields:
#      my_log('no files in {}'.format(self.fields))
      return 0

    for f in self.fields['files']:
      if f is None:
        continue
      if not 'LoC' in f:
#        my_log('{}: f has no LoC: {}'.format(self.fields['name'], f))
        continue

#      my_log('f {}'.format(f))
      try:
        f_loc = int(f['LoC'])
        if 0 < f_loc:
          loc += f_loc
      except Exception:
        my_log('exception for {}'.format(self.fields))

    return loc

# Minimalist version of a JSONModule
class SlimModule:
  """Slimmed-down version of JSONModule.
     For loading ecosystem-scale data without blowing up memory.
  """
  fields = {}

  def __init__(self, jsonModule, extractorFunc):
    self.fields = extractorFunc(jsonModule)
    self.fields['vulnerable'] = False

  def query(self, extractorFunc):
    return extractorFunc(self.fields)

  def markVuln(self):
    self.fields['vulnerable'] = True


def main(moduleFile, vulnFile):
  plt.figure(figsize=[10,7]) 
  
  name2slimModule = {}
  
  # Get name, LoC, and downloads/month for each module
  my_log("Reading {} for name, LoC, and downloads/month".format(moduleFile))
  f = open(moduleFile, "r")
  i = 0
  for line in f:
    i += 1
    if (i % 10000 == 0):
      my_log('{}'.format(i))

    jsonModule = JSONModule(json.loads(line))
    if not jsonModule.canClone():
      continue

    name = jsonModule.getName()
    slimModule = SlimModule(jsonModule, lambda m: { "downloadsPerMonth": m.getDownloadsPerMonth(), "LoC": m.getLoC() })

    name2slimModule[name] = slimModule

  # Go through the vulnerable patterns and mark their modules vulnerable
  my_log("Reading {} for vulnerable patterns".format(vulnFile))
  f = open(vulnFile, "r")
  i = 0
  for line in f:
    i += 1
    if (i % 10000 == 0):
      my_log('{}'.format(i))

    vulnReport = json.loads(line)
    for module in vulnReport['modules']:
      if module in name2slimModule:
        name2slimModule[module].markVuln()

  # Axes
  my_log("Adding axes")
  plt.xlabel("Lines of code", fontsize=20)
  plt.ylabel("Downloads per month", fontsize=20)
  plt.title("Module size vs. popularity, with presence of vuln. regexes", fontsize=20)
  plt.xscale('log')
  plt.yscale('log')

  # suffle safe and unsafe modules
  all_slim = name2slimModule.values()
  random.shuffle(all_slim)
  colors = []
  locs = []
  pops = []
  facecolors = []

  # an easy workaround for the legend...
  last_vuln = {}
  last_safe = {}

  for point in all_slim:
    color = "0.75" 
    facecolor = 'none'
    if point.query(lambda fields: fields['vulnerable'] == True):
      color = "0.0"
      facecolor = "0.0"
    loc = int(point.query(lambda fields: fields['LoC']))
    pop = int(point.query(lambda fields: fields['downloadsPerMonth']))

    if loc > 1 and pop > 1:
      colors.append(color)
      locs.append(loc)
      pops.append(pop)
      facecolors.append(facecolor)
      if color == '0.75':
          last_safe = point
      else:
          last_vuln = point

  plt.scatter(locs,pops, s=40, facecolors=facecolors, edgecolors=colors)
  # Extra plot customization
#  plt.grid(True)
#  plt.gca().autoscale()
  
  # make the legend
  safe = plt.scatter(int(last_safe.query(lambda fields: fields['LoC'])), int(last_safe.query(lambda fields: fields['downloadsPerMonth'])), edgecolor='0.75', facecolor='none', label='Safe')
  vuln = plt.scatter(int(last_vuln.query(lambda fields: fields['LoC'])), int(last_vuln.query(lambda fields: fields['downloadsPerMonth'])), facecolor='0.0', edgecolor='0.00', label='Vuln')
  plt.legend(fontsize=20, scatterpoints=1)
  
  plt.tight_layout()

  plt.xticks(fontsize=18)
  plt.yticks(fontsize=18)
  save_location = '/tmp/module-size-vs-popularity-output.png'
  plt.savefig(save_location)
  my_log("Saved figure to {}".format(save_location))
  plt.show()

##########################################

# Parse args
parser = argparse.ArgumentParser(description='Plot module size vs. popularity')
parser.add_argument('--module-file',         help='file to read in modules from',                          required=True, dest='moduleFile')
parser.add_argument('--vuln-pattern2module', help='color the modules that use these vulnerable patterns.', required=True, dest='vulnFile')

args = parser.parse_args()

# Invoke main
moduleFile = args.moduleFile
vulnFile = args.vulnFile

main(moduleFile, vulnFile)
