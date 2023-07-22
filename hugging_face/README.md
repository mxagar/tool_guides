# Hugging Face: A Guide

This mini-project compiles information on the usage of Hugging Face, as well as examples and material from the listed tutorials.

Source material:

- [Getting Started With Hugging Face in 15 Minutes | Transformers, Pipeline, Tokenizer, Models](https://www.youtube.com/watch?v=QEaBAZQCtwE)
- [HuggingFace Crash Course - Sentiment Analysis, Model Hub, Fine Tuning](https://www.youtube.com/watch?v=GSt00_-0ncQ)
- [An Introduction to Using Transformers and Hugging Face](https://www.datacamp.com/tutorial/an-introduction-to-using-transformers-and-hugging-face)
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/index)
- [Hugging Face NLP Course](https://huggingface.co/learn/nlp-course/chapter1/1)
- [Fine-tune a pretrained model](https://huggingface.co/docs/transformers/training)
- [Fine-tuning with custom datasets](https://huggingface.co/transformers/v3.2.0/custom_datasets.html)

Table of contents:

- [Hugging Face: A Guide](#hugging-face-a-guide)
  - [1. Introduction and Quick Start](#1-introduction-and-quick-start)
    - [Setup](#setup)
    - [Pipeline](#pipeline)
    - [Tokenizer and Models](#tokenizer-and-models)
    - [Combining the Code with PyTorch and Tensorflow](#combining-the-code-with-pytorch-and-tensorflow)
    - [Save and Load](#save-and-load)
    - [Model Hub](#model-hub)
    - [Finetuning Pre-Trained Models with Our Datasets](#finetuning-pre-trained-models-with-our-datasets)


## 1. Introduction and Quick Start

See the notebook [`hugging_face_intro.ipynb`](hugging_face_intro.ipynb).

### Setup

First install either Tensorflow/Keras or Pytorch in an environment. Then, we can install the `transformers` library, which comes from [Hugging Face](https://huggingface.co/):

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate ds

# Pytorch: Windows + CUDA 11.7
# Update your NVIDIA drivers: https://www.nvidia.com/Download/index.aspx
# I have version 12.1, but it works with older versions, e.g. 11.7
# Check your CUDA version with: nvidia-smi.exe
# In case of any runtime errors, check vrsion compatibility tables:
# https://github.com/pytorch/vision#installation
python -m pip install torch==1.13+cu117 torchvision==0.14+cu117 torchaudio torchtext==0.14 --index-url https://download.pytorch.org/whl/cu117

# Install the transformers library
pip install transformers datasets accelerate evaluate

# For CPU support only:
pip install 'transformers[torch]' datasets accelerate evaluate
```

### Pipeline



### Tokenizer and Models



### Combining the Code with PyTorch and Tensorflow



### Save and Load



### Model Hub



### Finetuning Pre-Trained Models with Our Datasets


