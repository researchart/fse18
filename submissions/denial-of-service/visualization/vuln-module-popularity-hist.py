# TODO Not yet written, taken from Christy's gist.

import matplotlib.pyplot as plt
import json
from collections import defaultdict
import numpy as np
import matplotlib.ticker
import argparse

# parse args
parser = argparse.ArgumentParser(description='Plotting dynamic vs regular patterns')
parser.add_argument('--file', '-f', help='file to read in patterns from', default = "../data/pypi-regexps-uniquePatterns.json")
parser.add_argument('--no-zeroes', help='skip over files with no regexes. default = false.', default = False, action="store_true")
parser.add_argument('--histogram', help='plot as a histogram instead. default = false.', default = False, action="store_true")

args = parser.parse_args()

# parsed args
NO_ZEROES = args.no_zeroes
file_name = args.file
HISTOGRAM = args.histogram

def get_dynamic_per_repository(repo):
    """
    Get all dynamic patterns in the repository.

    Keyword arguments:
    repo -- object containing properties of the repo
    """

    dynamic_count = 0

    if 'DYNAMIC-PATTERN' in repo['uniquePatterns']:
        dynamic_count += repo['uniquePatterns']['DYNAMIC-PATTERN']

    return dynamic_count

def get_patterns_per_repository(repo):
    """
    Get all unique patterns in the repository.

    Keyword arguments:
    repo -- object containing properties of the repo
    """

    dynamic_count = get_dynamic_per_repository(repo)

    return len(repo['uniquePatterns']) + dynamic_count


def valid_repo(repo):
    """
    Quick check that the repo was parsable.

    Keyword arguments:
    repo -- object containing properties of the repo
    """
    return repo['canClone'] == 'yes'

percent_dynamic_list = []
unique_counts_list = []

# read file and get counts
f = open(file_name,"r")
for line in f:
    repo = json.loads(line)

    if not valid_repo(repo):
        continue

    unique = get_patterns_per_repository(repo)

    if NO_ZEROES and unique < 1:
        continue

    # irrelevant case
    if unique == 0:
        continue

    dynamic_percent = float(get_dynamic_per_repository(repo)) / unique

    percent_dynamic_list.append(dynamic_percent)
    unique_counts_list.append(unique)

if HISTOGRAM:
    num_bins = 25
    n, bins, patches = plt.hist(percent_dynamic_list,num_bins, facecolor='blue', alpha=0.5)
    plt.xlabel("% Dynamic Patterns")
    plt.ylabel("Number of modules")

    if NO_ZEROES:
        plt.title("# of modules with % of dynamic patterns (with at least one regex)")
    else:
        plt.title("# of modules with % of dynamic patterns ")

else: 
    # plotting
    plt.scatter( unique_counts_list, percent_dynamic_list)
    plt.ylabel("% Dynamic Patterns")
    plt.xlabel("Number of unique patterns in module")

    if NO_ZEROES:
        plt.title("Number of unique patterns vs % Dynamic (with at least one regex)")
    else:
        plt.title("Number of unique patterns vs % Dynamic ")

    # plot customization
    plt.grid(True)
    plt.gca().xaxis.grid(True, which='minor')
    plt.xscale('log')
    plt.axis([0, max(unique_counts_list), 0, 1.0])

plt.show()

