# Gradio

[Gradio](https://www.gradio.app/)

> is the fastest way to demo your machine learning model with a friendly web interface so that anyone can use it, anywhere!

It is owned and extensively used by [HuggingFace](https://huggingface.co/) and it is quite similar to [Streamlit](https://streamlit.io/) &mdash; check my guide on Streamlit here: [](https://github.com/mxagar/streamlit_guide).

This current guide contains my notes on the basic functionalities of [Gradio](https://www.gradio.app/). I created the notes after following the [Hugging Face Bootcamp by JM Portilla, Udemy](https://www.udemy.com/course/complete-hugging-face-bootcamp). I have further notes on more ML/model-related topics referrng to Haggig Face at [mxagar/tool_guides/hugging_face](https://github.com/mxagar/tool_guides/tree/master/hugging_face).

Notes:

- I am adding modified versions of the notebooks used in the bootcamp. I tried to find the original repository to fork it, but unfortunately I didn't find it.
- The slides are public and accessible under [https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz](https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz).

Table of contents:

- [Gradio](#gradio)
  - [Setup](#setup)
  - [Introduction](#introduction)
  - [Gradio Components](#gradio-components)

## Setup

See [mxagar/tool_guides/hugging_face/udemy_hugging_face](https://github.com/mxagar/tool_guides/tree/master/hugging_face/udemy_hugging_face).

To set up a conda environment, you ca use the co-located [`conda.yaml`](./conda.yaml):

```bash
conda env create -f conda.yaml
conda activate hf
```

Notebooks:

- [`01-Gradio-Introduction.ipynb`](./notebooks/01-Gradio-Introduction.ipynb)
- [`02-Layout.ipynb`](./notebooks/02-Layout.ipynb)
- [`03-Interactions.ipynb`](./notebooks/03-Interactions.ipynb)
- [`04-Image-ML-Integration.ipynb`](./notebooks/04-Image-ML-Integration.ipynb)
- [`05-Text-ML-Integration.ipynb`](./notebooks/05-Text-ML-Integration.ipynb)
- [`06-Modals-Errors.ipynb`](./notebooks/06-Modals-Errors.ipynb)
- [`07-Styling-and-Themes.ipynb`](./notebooks/07-Styling-and-Themes.ipynb)

## Introduction

Key ideas:

- Gradio is made up by components: text, image, etc.
- Components can be interchangeably input or output components
- We define an interface layout where the components are set.
- Then, we define our Python code which connects to those components.

![Gradio Components](./assets/gradio_components.png)

## Gradio Components

