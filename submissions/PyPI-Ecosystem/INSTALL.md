Replication pack, FSE2018 submission #164:
------------------------------------------

Replication guide
=================

Step 0 - prerequisites
----------------------

- Unix-compatible OS (Linux or OS X)
- Python interpreter (2.7 was used; Python 3 compatibility is highly likely)
- R 3.4 or higher (3.4.4 was used, 3.2 is known to be incompatible)

Depending on detalization level (see Step 2 for more details):
- up to 2Tb of disk space (see Step 2 detalization levels)
- at least 16Gb of RAM (64 preferable)
- few hours to few month of processing time

Step 1 - software
----------------

- unpack **ghd-0.1.0.zip**, or clone from gitlab:

      git clone https://gitlab.com/user2589/ghd.git
      git checkout 0.1.0
  
  `cd` into the extracted folder. 
  All commands below assume it as a current directory.
    
- copy `settings.py` into the extracted folder. Edit the file:
    * set `DATASET_PATH` to some newly created folder path
    * add at least one GitHub API token to `SCRAPER_GITHUB_API_TOKENS` 
- install docker. For Ubuntu Linux, the command is 
    `sudo apt-get install docker-compose`
- install libarchive and headers: `sudo apt-get install libarchive-dev`
- (optional) to replicate on NPM, install yajl: `sudo apt-get install yajl-tools`
  Without this dependency, you might get an error on the next step, 
  but it's safe to ignore.
- install Python libraries: `pip install --user -r requirements.txt` . 
- disable all APIs except GitHub (Bitbucket and Gitlab support were
  not yet implemented when this study was in progress): edit
  `scraper/init.py`, comment out everything except GitHub support
  in `PROVIDERS`.

Step 2 - obtaining the dataset
-----------------------------

The ultimate goal of this step is to get output of the Python function 
`common.utils.survival_data()` and save it into a CSV file:

    # copy and paste into a Python console
    from common import utils
    survival_data = utils.survival_data('pypi', '2008', smoothing=6)
    survival_data.to_csv('survival_data.csv')

Since full replication will take several months, here are some ways to speedup
the process:

####Option 2.a, difficulty level: easiest

Just use the precomputed data. Step 1 is not necessary under this scenario.

- extract **dataset_minimal_Jan_2018.zip**
- get `survival_data.csv`, go to the next step

####Option 2.b, difficulty level: easy

Use precomputed longitudinal feature values to build the final table.
The whole process will take 15..30 minutes.

- create a folder `<DATASET_PATH>/common.cache`, where `<DATASET_PATH>` is 
  the value of the variable `DATASET_PATH` in `settings.py`
- extract **dataset_minimal_Jan_2018** to the newly created folder
- rename files:

      mv backporting.csv monthly_data.pypi_backporting.csv
      mv cc_degree.csv monthly_data.pypi_cc_degree.csv
      mv commercial.csv monthly_data.pypi_commercial.csv
      mv commits.csv monthly_data.pypi_commits.csv
      mv contributors.csv monthly_data.pypi_contributors.csv
      mv dc_katz.csv monthly_data.pypi_dc_katz.csv
      mv downstreams.csv monthly_data.pypi_downstreams.csv
      mv d_upstreams.csv monthly_data.pypi_d_upstreams.csv
      mv github_user_info.csv user_info.pypi.csv
      mv issues.csv monthly_data.pypi_issues.csv
      mv non_dev_issues.csv monthly_data.pypi_non_dev_issues.csv
      mv non_dev_submitters.csv monthly_data.pypi_non_dev_submitters
      mv package_urls.csv package_urls.pypi.csv
      mv q90.csv monthly_data.pypi_q90.csv
      # raw_dependencies.csv is not required
      # raw_packages_info.csv is not required
      # Feel free to read README.md for more details about the data
      mv submitters.csv monthly_data.pypi_submitters.csv
      # In this scenario we'll generate a new survival_data.csv
      mv university.csv monthly_data.pypi_university.csv
      mv upstreams.csv monthly_data.pypi_upstreams.csv

- edit `common/decorators.py`, set `DEFAULT_EXPIRY` to some higher value,
  e.g. `DEFAULT_EXPIRY = float('inf')  # cache never expires`

Then, use the Python code above to obtain `survival_data.csv`.

####Option 2.c, difficulty level: medium

Use predownloaded raw data to build longitudinal feature values, 
and then the dataset. Despite most of the data is cached, some functions will
pull up updates which might take anywhere from days to couple weeks to run.


- Download **dataset_full_Jan_2018.tgz** from http://k.soberi.us/dataset_full_Jan_2018.tgz .
This file is not included in this archive because of its size 
(5.4Gb compressed, 34Gb unpacked).
- edit `common/decorators.py`, set `DEFAULT_EXPIRY` to some higher value,
  e.g. `DEFAULT_EXPIRY = float('inf')  # cache never expires`
- extract the content of this archive into `<DATASET_PATH>`.
- clean up `<DATASET_PATH>/common.cache` (otherwise you'll get Step 2.a. 
You can reproduce Step 2.b by deleting only `survival_data.pypi_2008_2017-12_6.csv`)

Run the Python code above to obtain `survival_data.csv`.

####Option 2.d, difficulty level: hard

Build the dataset from scratch. Although most of the processing is
parallelized, it will take at least couple months on a reasonably
powerful server (32 cores, 512G of RAM, 2Tb+ of HDD space in our setup).

- ensure the `<DATASET_PATH>` is empty
- add more GitHub tokens (borrow from your coworkers) to `settings.py`.

Run the Python code above to obtain `survival_data.csv`.


Step 3 - run the regression
---------------------------

install R libraries:

    install.packages(c("htmlTable", "OIsurv", "survival", "car", "survminer", 
                       "ggplot2", "sqldf", "pscl", "texreg", "xtable"))

Use `build_model.r` (e.g. in RStudio) and produced `survival_data.csv` to 
build the regressions used in the paper. This process takes at least 16Gb of RAM 
and takes few hours to run due to the gigantic size of the dataset.
