#!/usr/bin/env python

# So it works when you're using SSH without -X. Magic.
#import matplotlib
#matplotlib.use('Agg')

import numpy as np
import matplotlib.pyplot as plt
import argparse
import json
from collections import defaultdict
from tabulate import tabulate


def raw_name_to_formatted(name):
    print name
    formatted = dict()
    formatted["error-message"] = "Error\nmessages"
    formatted["email"] = "Email"
    formatted["file/path"] = "File\nnames"
    formatted["html"] = "HTML"
    formatted["naming-convention"] = "Naming\nconvention"
    formatted['number'] = "Number"
    formatted['source-code'] = "Source\ncode"
    formatted['url'] = "URL"
    formatted['user-agent'] = "User-agent\nstrings"
    formatted['whitespace'] = "Whitespace"

    if not name in formatted:
        return name
    return formatted[name]


def get_language_counts_from_file(file_name, skip_unknown=True):
    """
    file_name : string file name
    skip_unknown : boolean. should we skip the "UNKNOWN" langauges?
    Given a file name, it will gather the language -> 
    count of regexes mapping for every language type.
    Returns a map.
    """

    counts = defaultdict(int)
    with open(file_name, 'r') as f:
        for line in f:
            language = json.loads(line)['language']

            if skip_unknown and "UNKNOWN" in language:
                continue

            # based on analyze-regexps/langauge/check-results-helper.pl,
            # we will merge similar languages
            if language == 'file' or language == 'filetype':
                language = 'file/path'
            elif language == 'js':
                language = 'source-code'

            counts[language] += 1
    return counts

def count_or_zero(the_map, key):
    """
    Returns value in map if key in map;
    otherwise 0.
    """

    if key in the_map:
        return the_map[key]
    return 0

def normalize(safe, unsafe):
    """
    Convert the raw numbers to percentages.
    """
    norm_safe = []
    norm_unsafe = []

    for s, un in zip(safe, unsafe):
        total = s + un
        if total == 0:
            percentage_vuln = 0
            percentage_safe = 0
        else:
            percentage_vuln = round(100.0 * float(un) / total)
            percentage_safe = 100 - percentage_vuln
        norm_safe.append(percentage_safe)
        norm_unsafe.append(percentage_vuln)
    return norm_safe, norm_unsafe


def get_npm_order(langs, npm_vulns, pypi_vulns):
    order = []
    for key, value in sorted(npm_vulns.iteritems(), key=lambda (k,v): (v,k)):
        order.append(key)

    for key, value in sorted(pypi_vulns.iteritems(), key=lambda (k,v ): (v, k)):
        if not key in order:
            order.append(key)

    for lang in langs:
        if not lang in order:
            order.append(lang)

    print("NPM ORDER: {}".format(order))
    return order

# parse args
parser = argparse.ArgumentParser(description='Plot regex languages vs safe/unsafe')
parser.add_argument('--npm-vulns', help='file of npm vulnerable regexes', required=True)
parser.add_argument('--npm-safe', help='file of npm safe regexes', required=True)
parser.add_argument('--pypi-vulns', help='file of pypi vulnerable regexes', required=True)
parser.add_argument('--pypi-safe', help='file of pypi safe regexes', required=True)
parser.add_argument('--raw-count-cutoff', help='discard languages with fewer than this many raw occurrences in npm', type=int, required=False)
args = parser.parse_args()

# get language : counts mappings
npm_vulns_map = get_language_counts_from_file(args.npm_vulns)
npm_safe_map = get_language_counts_from_file(args.npm_safe)
pypi_vulns_map = get_language_counts_from_file(args.pypi_vulns)
pypi_safe_map = get_language_counts_from_file(args.pypi_safe)

# create sorted lists of counts from the returned dicts
all_languages = set()
all_languages.update(npm_vulns_map.keys())
all_languages.update(npm_safe_map.keys())
all_languages.update(pypi_safe_map.keys())
all_languages.update(pypi_vulns_map.keys())
languages_list = get_npm_order(list(all_languages), npm_vulns_map, pypi_vulns_map)

raw_npm_safe = []
raw_npm_unsafe = []
raw_pypi_safe = []
raw_pypi_unsafe = []
langs_above_cutoff = []

for lang in languages_list:
    if 'raw_count_cutoff' in args:
        print('Checking we have enough {}'.format(lang))
        npm_safe_count = count_or_zero(npm_safe_map, lang)
        npm_vulns_count = count_or_zero(npm_vulns_map, lang)
        if npm_safe_count + npm_vulns_count < args.raw_count_cutoff:
            print('Not enough {} in npm: only {} safe {} vuln'.format(lang, npm_safe_count, npm_vulns_count))
            continue

    confusing_langs = ['metacharacters', 'numeric-sequence', 'hex', 'tag']
    if lang in confusing_langs:
        continue

    raw_npm_safe.append(count_or_zero(npm_safe_map, lang))
    raw_npm_unsafe.append(count_or_zero(npm_vulns_map, lang))
    raw_pypi_safe.append(count_or_zero(pypi_safe_map, lang))
    raw_pypi_unsafe.append(count_or_zero(pypi_vulns_map, lang))
    langs_above_cutoff.append(raw_name_to_formatted(lang))

# make them percentages
npm_safe, npm_unsafe = normalize(raw_npm_safe, raw_npm_unsafe)
pypi_safe, pypi_unsafe = normalize(raw_pypi_safe, raw_pypi_unsafe)

"""
# print for jamie
for idx, lang in enumerate(langs_above_cutoff):
    print("\n" + lang)

    count_npm_safe = raw_npm_safe[idx]
    count_npm_unsafe = raw_npm_unsafe[idx]
    count_npm_total = count_npm_safe + count_npm_unsafe

    count_pypi_safe = raw_pypi_safe[idx]
    count_pypi_unsafe = raw_pypi_unsafe[idx]
    count_pypi_total = count_pypi_safe + count_pypi_unsafe

    perc_npm_safe = npm_safe[idx]
    perc_npm_unsafe = npm_unsafe[idx]

    perc_pypi_safe = pypi_safe[idx]
    perc_pypi_unsafe = pypi_unsafe[idx]

    print(tabulate([['# npm', count_npm_safe, count_npm_unsafe, count_npm_total], ['# pypi', count_pypi_safe, count_pypi_unsafe, count_pypi_total], ['% npm', perc_npm_safe, perc_npm_unsafe, 100], ['% pypi', perc_pypi_safe, perc_pypi_unsafe, 100]], headers=['description', 'safe', 'unsafe', 'total']))
"""

# width setup
n = len(npm_safe)
ind = np.arange(n)
width = 0.35                            # could be changed?
fig, ax = plt.subplots()

# sort by npm
npm_unsafe, pypi_unsafe, langs_above_cutoff = zip(*sorted(zip(npm_unsafe, pypi_unsafe, langs_above_cutoff)))
npm_unsafe = list(npm_unsafe)
npm_unsafe.reverse()
pypi_unsafe = list(pypi_unsafe)
pypi_unsafe.reverse()
langs_above_cutoff = list(langs_above_cutoff)
langs_above_cutoff.reverse()

# color: 0 is black, 1 is white
# color=1: white
# set up npm
bar_npm_unsafe = ax.bar(ind, npm_unsafe, width, color='0.0', edgecolor='0.0')
#bar_npm_safe = ax.bar(ind, npm_safe, width, bottom=npm_unsafe, color='0.75', edgecolor='0.0')

# set up pypi
bar_pypi_unsafe = ax.bar(ind + width, pypi_unsafe, width, color='0.5', edgecolor='0.0')
#bar_pypi_safe = ax.bar(ind + width, pypi_safe, width, color='1.00', bottom=pypi_unsafe, edgecolor='0.0')

# legend outside of figure
box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

plt.ylabel('% regexes with this semantic meaning that are vulnerable', fontsize=10)
#plt.xlabel('Semantic type', fontsize=14)
plt.title('Proportion of vulnerable regexes within each semantic meaning', fontsize=12)
print("Langs above the cutoff: {}".format(langs_above_cutoff))
plt.xticks(ind, langs_above_cutoff, rotation=90, fontsize=10)

plt.legend((bar_npm_unsafe[0], bar_pypi_unsafe[0]), ('npm', 'pypi'))
plt.tight_layout()

# save fig
save_location = '/tmp/regex-semantic-behavior-plot.png'
fig.savefig(save_location, dpi=300)
print("Plot saved to {}".format(save_location))

# show fig
plt.show()
