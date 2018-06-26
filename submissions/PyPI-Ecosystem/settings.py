# coding:utf-8

# ========================================================
# IMPORTANT:
# complete this file before doing anything else
# ========================================================

import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

INSTALLED_APPS = [
    'common'
]

# Place to save raw data and persistent cache files
# Will take ~20Gb of space and ~1M of files
DATASET_PATH = "/tmp"

# persistent pypi storage. It will greatly speed up data collection
# The whole PyPI dataset (all versions of all packages extracted) will
# take ~2Tb of space.
# Setting to None will use /tmp by default and clean up after processing
# each package. It won't take a lot of space but will slow down the process
PYPI_SAVE_PATH = os.path.join(DATASET_PATH, "pypi")

# get one here: https://github.com/settings/tokens
SCRAPER_GITHUB_API_TOKENS = [
    "your GitHub API token",  # personal
    # more tokens will speed up data collection
]

SCRAPER_GITLAB_API_TOKENS = [
    "your GitLab tokens"  # not required for the FSE#164 reproduction
]

