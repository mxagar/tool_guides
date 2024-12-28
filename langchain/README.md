# LangChain Guide

> LangChain is a framework for developing applications powered by language models. It enables applications that are:
>
> - Data-aware: connect a language model to other sources of data, i.e., we integrate external data
> - Agentic: allow a language model to interact with its environment via decision making.

The idea is that we have several modules with different functionalities which are used in chains to accomplish tasks. Check the official

- [use cases](https://python.langchain.com/docs/use_cases),
- list of [modules](https://python.langchain.com/docs/modules/),
- list of [integrations](https://python.langchain.com/docs/integrations).

Note that LanChain uses models from other frameworks/vendors, such as OpenAI, Cohere, HuggingFace, etc; these are called **integrations**. As such, we need to have the specific keys or access tokens of those vendors. This mini-tutorial uses `dotenv` to load the keys defined in the `.env` file (not committed):

```
OPENAI_API_KEY=xxx
HUGGINGFACEHUB_API_TOKEN=xxx
```

This guide is organized in two parts:

- Part 1 in [`01_introduction/`](./01_introduction/): Quick start featuring most of the capabilities of LangChain.
- Part 2 in [`02_udemy_langchain/`](./02_udemy_langchain/): More detailed guide composed after following the [LanChain Bootcamp by JM Portilla, Udemy](https://www.udemy.com/course/langchain-with-python-bootcamp).

So choose one part and dive into it!

Other related repositories of mine:

- [LLM Tools](https://github.com/mxagar/tool_guides/tree/master/llms): Llamafile, Ollama, etc.
- [Generative AI Guide](https://github.com/mxagar/generative_ai_udacity): Notes of the [Udacity GenAI Nanodegree](https://www.udacity.com/course/generative-ai--nd608).
  - **This repository contains important guides on other LangChain-based tools**:
    - RAGs and Agents
    - LangSmith
    - LangGraph
    - etc.

Mikel Sagardia, 2024.  
No guarantees.
