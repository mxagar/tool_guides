# Hugging Face Bootcamp Notes

This guide contains my notes on the [Hugging Face Bootcamp by JM Portilla, Udemy](https://www.udemy.com/course/complete-hugging-face-bootcamp).

If you are looking for a quick start guide focusing on NLP, please look at [`../README.md`](../README.md); instead, if you'd like to get a broader overview, this is your part/section.

Notes:

- I am adding modified versions of the notebooks used in the bootcamp. I tried to find the original repository to fork it, but unfortunately I didn't find it.
- The slides are public and accessible under [https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz](https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz).

Table of contents:

- [Hugging Face Bootcamp Notes](#hugging-face-bootcamp-notes)
  - [1. Introduction to Hugging Face](#1-introduction-to-hugging-face)
    - [Install Packages](#install-packages)
    - [Account Setup](#account-setup)
  - [2. NLP with Transformers](#2-nlp-with-transformers)
  - [3. Image Models: Diffusers](#3-image-models-diffusers)
  - [4. Video Models](#4-video-models)
  - [5. Audio Models](#5-audio-models)
  - [6. Gradio for User Interfaces](#6-gradio-for-user-interfaces)

## 1. Introduction to Hugging Face

Folder: [`00-Setup-HF/`](./00-Setup-HF/).  
Notebook: [`00-HF-Setup.ipynb`](./00-Setup-HF/00-HF-Setup.ipynb).

Hugging Face has:

- Models 
  - Weights
  - Many modalities: CV, NLP, etc.
  - Large files can be stored in a reporitory (LFS)
  - Model pages/cards
- Datasets
  - Many modalities: CV, NLP, etc.
  - Some models are datasets are gated: we need to fill in a form and comply/sign to some rules
- Spaces
  - Create a git repo and connect it to HW to run a demo for other users
  - Gradio is often used: library to GUI creation, similar to Streamlit.
- Open Source Libs: Transformers, Diffusers, Gradio, Accelerate, etc.
  - Source code: in Github
  - Documentation: in Hugging Face
- Community
- A CLI Tool

### Install Packages

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

# If you want to install the HuggingFace Hub conection
# This is only necessary if we need programmatic access,
# for which there is also a CLI tool
pip install huggingface_hub
```

### Account Setup

To create a model:

- Profile: New model; e.g.: `test-model`
- In the tab `Files and versions` we will the files, when added.
- Wse can add a model card and files via the web UI.
- Also, we could upload the model using git, because internally HF uses git with the LFS plugin (Large File Storage).
  ![Create new model](./assets/models_create.png)

To interact with HF using git, we need a token:

- Profile: Settings > Access Tokens; we can choose a Read/Write token. For uploading models, we need a *write* permissions, otherwise *read* would suffice. We can save it in `.env`.

To download/clone the HF repo of the model:

- Click on `...` on the right: clone repository:
  ```bash
  # Install LFS if not done
  git lfs install

  # When prompted for a password, use an access token with write permission
  # and your username
  # Generate one from your settings: https://huggingface.co/settings/tokens
  git clone https://huggingface.co/mxagar/test-model
  
  # If you want to clone without large files - just their pointers
  GIT_LFS_SKIP_SMUDGE=1 git clone https://huggingface.co/mxagar/test-model
  ```

In order to autheticate, we need to use the generated token. One way of doing that is [by setting a remote URL with it](https://huggingface.co/blog/password-git-deprecation)!

```bash
# Set origin with token
git remote set-url origin https://<user_name>:<token>@huggingface.co/<repo_path>
git pull origin

# where <repo_path> is in the form of:
# - <user_name>/<repo_name> for models
# - datasets/<user_name>/<repo_name> for datasets
# - spaces/<user_name>/<repo_name> for Spaces

# Example
git remote set-url origin https://mxagar:hf_xxx@huggingface.co/mxagar/test-model
```

Then, we can work as always using git workflows.


## 2. NLP with Transformers

Folder: []().  
Notebooks:
- A
- B

## 3. Image Models: Diffusers

Folder: []().  
Notebooks:
- A
- B

## 4. Video Models

Folder: []().  
Notebooks:
- A
- B

## 5. Audio Models

Folder: []().  
Notebooks:
- A
- B

## 6. Gradio for User Interfaces

Folder: []().  
Notebooks:
- A
- B

