import matplotlib.pyplot as plt
import json
from collections import defaultdict
import numpy as np
import matplotlib.ticker
import argparse

# parse args
parser = argparse.ArgumentParser(description='Plotting unique patterns')
parser.add_argument('--file', '-f', help='file to read in patterns from', default = "../data/pypi-regexps-uniquePatterns.json")
parser.add_argument('--no-zeroes', help='skip over files with no regexes. default = false.', default = False, action = "store_true")

args = parser.parse_args()

# parsed args
NO_ZEROES = args.no_zeroes
file_name = args.file


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


def get_loc_per_repository(repo):
    """
    Get LOC in the repository.

    Keyword arguments:
    repo -- object containing properties of the repo
    """
    loc_count = 0

    for file in repo['files']:
        if file is None or not 'LoC' in file:
            return -1
        loc = file['LoC']

        if loc <= 0:
            return -1

        loc_count += loc

    return loc_count

def valid_repo(repo):
    """
    Quick check that the repo was parsable.

    Keyword arguments:
    repo -- object containing properties of the repo
    """
    return repo['canClone'] == 'yes'

loc_list = []
unique_list= []

# read file and get counts
f = open(file_name,"r")
for line in f:
    repo = json.loads(line)
    if not valid_repo(repo):
        continue

    unique = get_patterns_per_repository(repo)

    if NO_ZEROES and unique < 1:
        continue

    loc = get_loc_per_repository(repo)

    if loc <= 0:
        continue

    unique_list.append(unique)
    loc_list.append(loc)

plt.scatter( loc_list, unique_list)
plt.ylabel("Number of unique patterns in module")
plt.xlabel("LOC in repo")

plt.title("LOC in repo vs unique patterns in module")

# plot customization
plt.grid(True)
plt.xscale('log')

plt.show()
