#!/usr/bin/env python

import matplotlib.pyplot as plt
import json
from collections import defaultdict
import numpy as np
import matplotlib.ticker
import argparse

# parse args
parser = argparse.ArgumentParser(description='Plotting unique patterns')
parser.add_argument('--file', '-f', help='file with module/uniquePatterns objects. Format ecosystem:file. Example: npm:npm-data.json', action = 'append', required = True)
parser.add_argument('--vulns-file', '-v', help='file with the vulnerabilities. Format ecosystem:file. Example: npm:npm-data.json', action = 'append', required = True)
parser.add_argument('--no-zeroes', help='skip over files with no regexes. default = false.', required = False, default = False, action="store_true")

args = parser.parse_args()

# parsed args
NO_ZEROES = args.no_zeroes
files = args.file
vulns_files = args.vulns_file

ecosystem2file = {}
ecosystem2vulns = {}

for f in files:
    ecosystem, file_name = f.split(':')
    ecosystem2file[ecosystem] = file_name

for f in vulns_files:
    ecosystem, file_name = f.split(':')
    ecosystem2vulns[ecosystem] = file_name

def load_vulns(vulns_file, safe_file_name):
    """
    Collect the number of vulnerabilities per module.
    Returns a dict of module => # vulns.

    Keyword aguments:
    vulns_file -- name of the file that has the vulnerability reports
    """
    vulns = defaultdict(int)
    with open(vulns_file, 'r') as f:
        for line in f:
            vulnsReport = json.loads(line)
            for module in vulnsReport['modules']:
                vulns[module] += 1

    # add in the modules with non vulnerabilities
    with open(safe_file_name, 'r') as f:
        for line in f:
            module = json.loads(line)
            name = module['name']
            if not valid_repo(module):
                continue
            unique_patterns = len(module['uniquePatterns'])
            if not name in vulns and unique_patterns > 0:
                vulns[name] = 0

    return vulns

def get_vulns_to_modules(vulns_file):
    """
    Returns the frequency of x # modules
    affected by a vulnerable regex.

    Keyword args:
    vulns_file -- file with the vulnerable regexes
    """
    vulns_to_modules = defaultdict(int)
    with open(vulns_file, 'r') as f:
        for line in f:
            vulnsReport = json.loads(line)
            numModules = len(vulnsReport['modules'])
            vulns_to_modules[numModules] += 1
    return vulns_to_modules

def num_vulns_to_frequency(vulns):
    """
    Returns the frequency of x # vulnerabilities
    per module.

    Keyword arguments:
    vulns -- mapping of module => # vulns
    """

    counts = defaultdict(int)

    for num_vulns in vulns.values():
        counts[num_vulns] += 1
    return counts

def get_patterns_per_repository(repo):
    """
    Get all unique patterns in the repository.

    Keyword arguments:
    repo -- object containing properties of the repo
    """

    dynamic_count = 0

    if 'DYNAMIC-PATTERN' in repo['uniquePatterns']:
        dynamic_count += repo['uniquePatterns']['DYNAMIC-PATTERN']

    return len(repo['uniquePatterns']) + dynamic_count

def valid_repo(repo):
    """
    Quick check that the repo was parsable.

    Keyword arguments:
    repo -- object containing properties of the repo
    """
    return repo['canClone'] == 'yes'

def buildNumPatternsToFrequency (file_name):
  num_patterns_to_frequency = defaultdict(int)
  
  # read file and get counts
  f = open(file_name,"r")
  for line in f:
      repo = json.loads(line)
  
      if not valid_repo(repo):
          continue
  
      unique = get_patterns_per_repository(repo)

      # Discard projects with no regexes?
      if NO_ZEROES and unique < 1:
          continue
  
      num_patterns_to_frequency[unique] += 1
  
#      if unique > 10:
#          print(repo['name'] + "\t" + repo['repository']['url'] + "\t" + repr(unique))

  return num_patterns_to_frequency

def get_cdf(patterns_to_frequency):
    """
    Does the CDF computation.

    Keyword args:
    patterns-to-frequency: dict of # patterns -> # occurrences
    """
    lists = sorted(patterns_to_frequency.items())
    num_patterns, frequencies = zip(*lists)
    total_modules = sum(frequencies)
    cum_frequencies = reduce(lambda c, x: c + [c[-1] + x], frequencies, [0])[1:]
    percentage_cum_frequencies =  [val * 100/ total_modules for val in cum_frequencies]
    return num_patterns, percentage_cum_frequencies

################
# main

styles = [':', '--', '-']
colors = ['0.0', '0.5']

for ecosystem, color in zip(ecosystem2file.keys(), colors):
  print('Working on {}'.format(ecosystem))
  file_name = ecosystem2file[ecosystem]
  num_patterns_to_frequency = buildNumPatternsToFrequency(file_name)

  num_patterns, percentage_cum_frequencies = get_cdf(num_patterns_to_frequency)
  plt.plot(num_patterns, percentage_cum_frequencies, linewidth=4.0, label=ecosystem + " unique regexes in module", linestyle = styles[0], color=color)

  """
  print("Working on {} vulnerabilities".format(ecosystem))
  vuln_file_name = ecosystem2vulns[ecosystem]
  vulns = load_vulns(vuln_file_name, file_name)
  counts = num_vulns_to_frequency(vulns)
  num_patterns, percentage_cum_frequencies = get_cdf(counts)
  plt.plot(num_patterns, percentage_cum_frequencies, linewidth=4.0, label=ecosystem + " (vulns)", linestyle = styles[1], color=color)
  """

  print("Working on {} pattern to module".format(ecosystem))
  vulns_to_frequency = get_vulns_to_modules(ecosystem2vulns[ecosystem])
  num_modules, percentage_cum_frequencies = get_cdf(vulns_to_frequency)
  plt.plot(num_modules, percentage_cum_frequencies, linewidth=3.0, label=ecosystem + " vulnerable regex module appearances", linestyle = styles[2], color=color)

plt.xlabel("Modules/vulnerable regexes", fontsize = 14)
plt.ylabel("% less than", fontsize = 14)
plt.legend(fontsize = 14)
plt.title("Distribution of regexes in modules and\nModules impacted by vulnerable regexes.", fontsize=16)

# plot customization
plt.grid(True)
plt.gca().xaxis.grid(True, which='minor')
plt.xscale('symlog', linthreshx=20)
plt.xticks(fontsize=14)
plt.yticks(fontsize =14)

save_location = "/tmp/CDF-regex-use-npm-pypi.png"
print("saved to {}".format(save_location))
plt.savefig(save_location)
plt.show()
