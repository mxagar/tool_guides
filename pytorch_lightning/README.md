# Pytorch Lightning

Some resources:

- [Lightning Tutorials](https://www.pytorchlightning.ai/tutorials)
- [PyTorch Lightning Masterclass](https://www.youtube.com/playlist?list=PLaMu-SDt_RB5NUm67hU2pdE75j6KaIOv2)
- [UvA Deep Learning Tutorials](https://uvadlc-notebooks.readthedocs.io/en/latest/)
- [Lightning in 15 minutes](https://lightning.ai/docs/pytorch/stable/starter/introduction.html)
- [From PyTorch to PyTorch Lightning - A gentle introduction (by William Falcon, creator of Lightning)](https://towardsdatascience.com/from-pytorch-to-pytorch-lightning-a-gentle-introduction-b371b7caaf09)

## Setup and Installation

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate ds
# Alternatively, if you have an env and wnat to install tensorboard
python -m pip install lightning

# OPTIONAL: Pytorch on Windows + CUDA 11.7
# Update your NVIDIA drivers: https://www.nvidia.com/Download/index.aspx
# I have version 12.1, but it works with older versions, e.g. 11.7
# Check your CUDA version with: nvidia-smi.exe
# In case of any runtime errors, check vrsion compatibility tables:
# https://github.com/pytorch/vision#installation
python -m pip install -U torch==1.13+cu117 torchvision==0.14+cu117 torchaudio torchtext==0.14 --index-url https://download.pytorch.org/whl/cu117
```

## Quick Introduction

