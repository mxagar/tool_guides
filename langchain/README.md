# LangChain Introduction

> LangChain is a framework for developing applications powered by language models. It enables applications that are:
>
> - Data-aware: connect a language model to other sources of data
> - Agentic: allow a language model to interact with its environment

The idea is that we have several modules with different functionalities which are used in chains to accomplish tasks. Check the official

- [use cases](https://python.langchain.com/docs/use_cases)
- and list of [modules](https://python.langchain.com/docs/modules/).

Note that LanChain uses models from other frameworks/vendors, such as OpenAI, Cohere, HuggingFace, etc; these are called **integrations**. As such, we need to have the specific keys or access tokens of those vendors. This mini-tutorial uses `dotenv` to load the keys defined in the `.env` file (not committed):

```
OPENAI_API_KEY=xxx
HUGGINGFACEHUB_API_TOKEN=xxx
```

Sources followed to prepare this guide:

- [LangChain Crash Course - Build apps with language models](https://www.youtube.com/watch?v=LbT1yp6quS8)
- [The LangChain Cookbook - Beginner Guide To 7 Essential Concepts](https://www.youtube.com/watch?v=2xxziIWmaSA)

## 0. Setup

Official websites:

- [documentation](https://python.langchain.com/)
- [github](https://github.com/langchain-ai/langchain)

To install langchain and the required integrations:

```bash
conda activate ds

# Install langchain without any integrations
pip install langchain
# Then, integrations are installed later as needed
pip install openai
pip install huggingface_hub

# Install all integrations
pip install 'langchain[all]'
```

## 1. LLMs

