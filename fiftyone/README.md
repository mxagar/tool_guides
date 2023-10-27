# FiftyOne Guide

[FiftyOne](https://docs.voxel51.com/index.html) is a 

> an open-source tool for building high-quality datasets and computer vision models

which can be used, among others, to visualize prediction data, i.e., we can perform error analysis with it.

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

## Tutorials




## Further Information

Interesting links:

- [See What You Segment with SAM](https://medium.com/towards-data-science/see-what-you-sam-4eea9ad9a5de)

