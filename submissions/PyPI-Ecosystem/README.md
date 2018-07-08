Replication pack, FSE2018 submission #164:
----------

**Working title:** Ecosystem-Level Factors Affecting the Survival of Open-Source Projects: 
A Case Study of the PyPI Ecosystem

**Note:** link to data artifacts is already included in the paper. 
Link to the code will be included in the Camera Ready version as well.

**Note2** Full dataset is a bit too big (5.4Gb compressed, 34Gb unpacked), and
is not required minimal and thorough replications (see INSTALL.md for details).
We published a separate version of this archive including the full dataset on Zenodo:
https://zenodo.org/record/1297925

Content description
===================

- **ghd-0.1.0.zip** - the code archive. This code produces the dataset files 
  described below
- **settings.py** - settings template for the code archive.
- **dataset_minimal_Jan_2018.zip** - the minimally sufficient version of the dataset.
  This dataset only includes stats aggregated by the ecosystem (PyPI)
- **dataset_full_Jan_2018.tgz** - not included in the replication set as optional,
  but avaiable here: https://zenodo.org/record/1297925 , DOI: 
  This dataset still doesn't include PyPI packages themselves, which take around 2TB.
- **build_model.r, helpers.r** - R files to process the survival data 
   (`survival_data.csv` in **dataset_minimal_Jan_2018.zip**, 
   `common.cache/survival_data.pypi_2008_2017-12_6.csv` in 
   **dataset_full_Jan_2018.tgz**)
- **Interview protocol.pdf** - approximate protocol used for semistructured interviews.
- LICENSE - text of GPL v3, under which this dataset is published
- INSTALL.md - replication guide (~2 pages)


Replication guide
=================

Please see INSTALL.md