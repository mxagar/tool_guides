# LangChain Udemy Course Notes

> [LangChain](https://python.langchain.com) is a framework for developing applications powered by language models. It enables applications that are:
>
> - Data-aware: connect a language model to other sources of data, i.e., we integrate external data
> - Agentic: allow a language model to interact with its environment via decision making.

The idea is that we have several modules with different functionalities which are used in chains to accomplish tasks. Check the official

- [use cases](https://python.langchain.com/docs/use_cases),
- list of [modules](https://python.langchain.com/docs/modules/),
- list oof [integrations](https://python.langchain.com/docs/integrations).

Note that LangChain uses models from other frameworks/vendors, such as OpenAI, Cohere, HuggingFace, etc; these are called **integrations**. As such, we need to have the specific keys or access tokens of those vendors. This mini-tutorial uses `dotenv` to load the keys defined in the `.env` file (not committed):

```
OPENAI_API_KEY=xxx
HUGGINGFACEHUB_API_TOKEN=xxx
```

This guide part is a compilation of my notes on the [LanChain Bootcamp by JM Portilla, Udemy](https://www.udemy.com/course/langchain-with-python-bootcamp).

If you are looking for a quick start guide, please look at [`../01_introduction/`](../01_introduction/); instead, if you'd like to get a broader overview, this is your part/section.

Notes:

- I am adding modified versions of the notebooks used in the bootcamp. I tried to find the original repository to fork it, but unfortunately I didn't find it.
- The slides are public and accessible under [https://drive.google.com/drive/folders/1Kh3-3-aS_xtYhK73sVeXNTyFGREgub_U](https://drive.google.com/drive/folders/1Kh3-3-aS_xtYhK73sVeXNTyFGREgub_U).

Table of contents:

- [LangChain Udemy Course Notes](#langchain-udemy-course-notes)
  - [1. Introduction](#1-introduction)
    - [Setup](#setup)
    - [OpenAI](#openai)
  - [2. Models](#2-models)
    - [Large Language Models (LLMs)](#large-language-models-llms)
  - [3. Data Connections](#3-data-connections)
  - [4. Chains](#4-chains)
  - [5. Memory](#5-memory)
  - [6. Agents](#6-agents)

## 1. Introduction

LangChain is not only a Python library, but also a package for other languages, e.g., Javascript. If we're going to use Python, we should stick to its documentation only:

[https://python.langchain.com/](https://python.langchain.com/)

The functionalities of the library are organized in **Modules** (Python) or **Components** (general).

This guide is built to reach the ultimate goals of dealing with **Agents**. However, that's the last section; beforehand, other more basic modules are shown:

![Sections](./assets/sections.png)

- Model IO: standardized wrapper for models: local, APIs, etc. Thanks to the interface, we can easily switch the underlying model without many changes in the code.
- Data Connections: standardized wrapper for data storage and data sources. Again, vector databases can be changed easily, as well as data containers (e.g., S3, folder, etc.).
- Chains: outputs from one model can be linked as inputs for another one.
- Memory: save conversation histories, outputs, etc.
- Agents: they combine all the previous modules to autonomously perform tasks, even using tools! We can create our own custom tools, too.

### Setup

First, create a Python environment, e.e.g, with Conda:

```bash
# Create environment + activate it
conda env create --file conda.yaml
conda activate llms

# If we add packages to the YAML
conda env update --name llms --file conda.yaml --prune
```

Additionally, we'll need to set up OpenAI and HuggingFace accounts:

- Get the access tokens from both and save them in `.env`: `OPENAI_API_KEY, HUGGINGFACEHUB_API_TOKEN`
- Set a payment method in the OpenAI account; and make sure what the pricing is! 

If everything is setup correctly, we'll need to load the API keys from `.env`:

```python
import os
from dotenv import load_dotenv
load_dotenv()

hf_token = os.getenv("HUGGINGFACEHUB_API_TOKEN")
openai_token = os.getenv("OPENAI_API_KEY")
```

### OpenAI

The Playground, Docs and Examples from the OpenAI API/Platform page are very interesting

[https://platform.openai.com](https://platform.openai.com)

We can 

- test different models with different parameters
- fine-tune models
- etc.

If everything is setup correctly, we can use the `openai` library as follows:

```python
import os
from openai import OpenAI
from dotenv import load_dotenv
load_dotenv()

openai_token = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=openai_token)

response = client.completions.create(
    model="gpt-3.5-turbo-instruct",
    prompt="Translate the following English text to French: 'Hello, how are you?'",
    max_tokens=60
)

print(response.choices[0].text)
# Salut, comment vas-tu ?
```

**IMPORTANT NOTE: There seem to be some deprecations / API updates in the OpenAI library.**

## 2. Models

With the Model IO module, we can easily interchange the underlying models; we can even benchmark them before choosing one.

Contents:

- LLMs
- Prompt Templates
- Prompts and Model Exercise
- Few Shot Prompt Templates
- Parsing Outputs
- Serialization - Saving and Loading Prompts
- Models IO Exercise Project

### Large Language Models (LLMs)

Notebook: [`00-Models-IO/00-Large-Language-Models.ipynb`](./00-Models-IO/00-Large-Language-Models.ipynb).




## 3. Data Connections

## 4. Chains

## 5. Memory

## 6. Agents



