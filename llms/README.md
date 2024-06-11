# LLM Playground

In this folder I test different tools related to LLMs. See also my quick guide on [LangChain](https://github.com/mxagar/tool_guides/tree/master/langchain).

Table of contents:

- [LLM Playground](#llm-playground)
  - [0. Setup](#0-setup)
  - [1. Llamafile](#1-llamafile)
  - [2. Chatbot Evaluation](#2-chatbot-evaluation)
  - [3. Ollama](#3-ollama)
  - [Authorship](#authorship)


## 0. Setup

Install the conda environment and activate it:

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate llms

# If you extend the YAML list, update the environment
conda env update --name llms --file conda.yaml --prune
```

I will add the required dependencied to the YAML as the repository evolves.

## 1. Llamafile

The notebook [`01_llamafile.ipynb`](./01_llamafile.ipynb) shows how to run LLMs locally using [Llamafile](https://github.com/Mozilla-Ocho/llamafile); as the authors describe it in their [announcement blogpost](https://hacks.mozilla.org/2023/11/introducing-llamafile/):

> `llamafile` lets you turn large language model (LLM) weights into executables. Say you have a set of LLM weights in the form of a 4GB file (in the commonly-used GGUF format). With llamafile you can transform that 4GB file into a binary that runs on six OSes without needing to be installed.

Covered topics:

- Running llamafiles with embedded weights.
- **Running llamafile with external weights**: 
  - via a web UI,
  - via a provided REST API,
  - or using the OpenAI library: we connect the REST API to it and we can use the OpenAI interfaces!
- Running a any local LLM served with llamafile to OpenAI and connecting that OpenAI instance to [`instructor`](https://pypi.org/project/instructor/), which parses natural language into JSONs.

## 2. Chatbot Evaluation

I created a simple chatbot evaluation package which can be used to score chatbots using a dataset of predefined and rated chat sessions.

I moved the package to its own repository: [chatbot_evaluation](https://github.com/mxagar/chatbot_evaluation).

Also, consider libraries like [RAGAS](https://docs.ragas.io/en/latest/getstarted/index.html).

## 3. Ollama

The folder [`03_ollama/`](./03_ollama/README.md) contains an introduction tutorial to [Ollama](https://ollama.com/), a similar tool to `llamafile`.

According to [github.com/ollama](https://github.com/ollama/ollama), `Ollama` is to

> Get up and running with large language models.

Covered topics:

- ...
- ...

## Authorship

Mikel Sagardia, 2024.  
No guarantees.  
