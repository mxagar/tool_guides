# FiftyOne Guide

[FiftyOne](https://docs.voxel51.com/index.html) is a 

> an open-source tool for building high-quality datasets and computer vision models

which can be used, among others, to visualize prediction data, curate datasets, or we can perform **error analysis** with it.

This tutorial shows how to use FiftyOne mainly for **error analysis**.

Table of contents:

- [FiftyOne Guide](#fiftyone-guide)
  - [Setup](#setup)
  - [Test Dataset](#test-dataset)
  - [Tutorials](#tutorials)
    - [General Utilities for Error Analysis](#general-utilities-for-error-analysis)
    - [Tabular Datasets](#tabular-datasets)
    - [Image Classification](#image-classification)
  - [Further Information](#further-information)


## Setup

```bash
# Create or activate your preferred python environment
conda env create -f conda.yaml
conda activate label

# If not done yet, install Label Studio
pip install --user label-studio
# If on Windows, add to Path the correct URL to be able to locate the binary
# C:\Users\<User>\AppData\Roaming\Python\Python39\Scripts

# Install FiftyOne
pip install fiftyone
```

We can also:

- [Install a desktop app if we don't want to use the browser](https://docs.voxel51.com/getting_started/install.html#fiftyone-desktop-app).
- [Install `ffmpeg` to work with video datasets](https://docs.voxel51.com/getting_started/install.html#installing-fiftyone).

## Test Dataset

In some examples, for testing puposes, I used the [Flowers dataset](https://www.kaggle.com/datasets/imsparsh/flowers-dataset) from Kaggle, placed in the local folder `data/flowers` (not committed).

Additionally, instead of using the images of the dataset directly, I used image vectors created with SimCLR in my other repository [simclr_pytorch_flowers](https://github.com/mxagar/simclr_pytorch_flowers): `datasets/vectors_dataset.csv`. The CSV compiles for each image the following information:

- `filename`
- `filepath`
- `label`: ground truth flower label: daisy, dandelion, rose, sunflower, tulip
- `linear_pred`: prediction with downstream ANN
- `embedding`: vector generated with SimCLR
- `cluster`: a cluster value, which could be compared to an unsupervised class prediction.

## Tutorials

The tutorials are implemented in Jupyter notebooks, they are located in the [`notebooks`](./notebooks/) folder, and focus on the following topics:

- [`00_general.ipynb`](./notebooks/00_general.ipynb): general utilities for error analysis.
- [`01_tabular.ipynb`](./notebooks/01_tabular.ipynb): Iris dataset modelling error analysis; see also: [Pandas Queries in FiftyOne](https://docs.voxel51.com/tutorials/pandas_comparison.html)
- [`02_image_classification.ipynb`](./notebooks/02_image_classification.ipynb): [Evaluating a Classifier in FiftyOne](https://docs.voxel51.com/tutorials/evaluate_classifications.html)

### General Utilities for Error Analysis

Notebook: [`00_general.ipynb`](./notebooks/00_general.ipynb).


### Tabular Datasets

Notebook: [`01_tabular.ipynb`](./notebooks/01_tabular.ipynb) - Iris dataset modelling error analysis.

See also: [Pandas Queries in FiftyOne](https://docs.voxel51.com/tutorials/pandas_comparison.html).


### Image Classification

Notebook: [`02_image_classification.ipynb`](./notebooks/02_image_classification.ipynb) - [Evaluating a Classifier in FiftyOne](https://docs.voxel51.com/tutorials/evaluate_classifications.html)



## Further Information

Interesting links:

- [See What You Segment with SAM](https://medium.com/towards-data-science/see-what-you-sam-4eea9ad9a5de)
- [From RAGs to Riches](https://medium.com/towards-data-science/from-rags-to-riches-53ba89087966)

