# LLM Playground

In this folder I test different tools related to LLMs. See also my quick guide on [LangChain](https://github.com/mxagar/tool_guides/tree/master/langchain).

Table of contents:

- [LLM Playground](#llm-playground)
  - [0. Setup](#0-setup)
  - [1. Llamafile](#1-llamafile)
  - [Authorship](#authorship)


## 0. Setup

Install the conda environment and activate it:

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate llms
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

## Authorship

Mikel Sagardia, 2024.  
No guarantees.  
