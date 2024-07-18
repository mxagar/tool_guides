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
    - [Documentation](#documentation)
  - [2. Models](#2-models)
    - [Large Language Models (LLMs)](#large-language-models-llms)
    - [Prompt Templates](#prompt-templates)
  - [3. Data Connections](#3-data-connections)
  - [4. Chains](#4-chains)
  - [5. Memory](#5-memory)
  - [6. Agents](#6-agents)

## 1. Introduction

LangChain is not only a Python library, but also a package for other languages, e.g., Javascript. If we're going to use Python, we should stick to its documentation only:

[https://python.langchain.com/](https://python.langchain.com/)

The functionalities of the library are organized in **Modules** (Python) or **Components** (general). However, the Python documentation seems to use the word **Components**, which are:

> - Chat models
> - LLMs
> - Messages
> - Prompt templates
> - Example selectors
> - Output parsers
> - Chat history
> - Documents
> - Document loaders
> - Text splitters
> - Embedding models
> - Vector stores
> - Retrievers
> - Tools
> - Toolkits
> - Agents
> - Callbacks

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
- Set a payment method in the OpenAI account and fill in the balance if we choose the pay-as-you-go option; and make sure what the pricing of our queries is!

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

### Documentation

Important links:

- [Conceptual Guide](https://python.langchain.com/v0.2/docs/concepts/): where the packages, components and techniques are explained.
- [Integrations](https://python.langchain.com/v0.2/docs/integrations/platforms/): where bindings are listed.
- [API Reference](https://api.python.langchain.com/en/latest/langchain_api_reference.html).

:warning: **IMPORTANT NOTES:

- There seem to be some deprecations / API updates in the OpenAI library.
- This guide is very focused on OpenAI; however, we can instead use Google models by checking the examples in the [Integrations](https://python.langchain.com/v0.2/docs/integrations/platforms/) page.

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

Notebook: [`00-Models-IO/00-Large-Language-Models.ipynb`](./00-Models-IO/00-Large-Language-Models.ipynb):

- LLM and Chat models: common calls, parameters
- Caching

```python
### -- LLMs
# !pip install langchain-openai
from langchain_openai import OpenAI, ChatOpenAI
# The OpenAI refers to the LLM, which is often an instruct model and returns raw text
# while ChatOpenAI refers to the Chatbot, which is often a conversation model and uses 3 distinct objects
# - SystemMessage: General system tone, personality
# - HumanMessage: Human request/reply
# - AIMessage: AI's response

llm = OpenAI(openai_api_key=openai_token) # default model: gpt-3.5-turbo-instruct
chat = ChatOpenAI(openai_api_key=openai_token) # default model: gpt-3.5-turbo

# We can use both the LLM and the Chat, but the reponse structure is different
# The tendency is to move towards Chats
print(llm.invoke('Here is a fun fact about Pluto:')) # Raw text: Pluto was discovered on February 18...
print(chat.invoke('Here is a fun fact about Pluto:')) # AIMessage object: {content: 'response', response_metadata: '', ...}

# Generate needs to be always a list
result = llm.generate(
    ['Here is a fun fact about Pluto:',
     'Here is a fun fact about Mars:']
)

# We can also use generate with a Chat model
result_chat = chat.generate(
    ['Here is a fun fact about Pluto:',
     'Here is a fun fact about Mars:']
)

# We get back a LLMResult object, both for llm and chat
type(result) # langchain_core.outputs.llm_result.LLMResult

# Schema of LLMResult
result.schema()

# The results are in the property generations
for g in result.generations:
    print(g[0].text)
    # Pluto was...
    # Mars has...

# Here we can see the metadata related to the operation
result.llm_output
# {'token_usage': {'total_tokens': 63, 'completion_tokens': 47, 'prompt_tokens': 16},
#  'model_name': 'gpt-3.5-turbo-instruct'}

### -- Chats and Parameters

# ChatOpenAI refers to the Chatbot, which is often a conversation model and uses 3 distinct objects
# - SystemMessage: General system tone, personality
# - HumanMessage: Human request/reply
# - AIMessage: AI's response
# The trend is towards using Chat models, not LLMs
from langchain_openai import ChatOpenAI

chat = ChatOpenAI(openai_api_key=openai_token)

from langchain.schema import (
    AIMessage,
    HumanMessage,
    SystemMessage
)

# The correst way of interacting with chat in OpenAI
# is to use the correct Message object: HumanMessage, SystemMessage
result = chat.invoke([HumanMessage(content="Can you tell me a fact about Earth?")])

type(result) # langchain_core.messages.ai.AIMessage
print(result.content) # Sure! One interesting...

# We can alter the personality or role of the chat with SystemMessage
result = chat.invoke(
    [
        SystemMessage(content='You are a very rude teenager who only wants to party and not answer questions'),
        HumanMessage(content='Can you tell me a fact about Earth?')
    ]
)

print(result.content) # Ugh, I don't know, like, why do you care? ...

# Generate needs to receive a list
# We can pass different message objects, though: SystemMessage, HumanMessage
# And each item can be a list, i.e., a chat history!
result = chat.generate(
    [[SystemMessage(content='You are a University Professor'),
      HumanMessage(content='Can you tell me a fact about Earth?')
      ]]
)

result.generations[0][0].text # Certainly! A fascinating fact...

# Extra params and arguments
# - temperature: creativity
# - presence_penalty: penalize token repetition
# - max_tokens: maximum number of tokens
result = chat.invoke(
    [HumanMessage(content='Can you tell me a fact about Earth?')],
    temperature=2, # default: 0.7, 2 is very high, so it will hallucinate rubbish
    presence_penalty=1,
    max_tokens=100
)

### -- Caching

# Caching is helpful when we're doing the same query several times:
# we incur in new costs, but the answer should be the same!
# The solution is to cache them, i.e., save the results.
# We can cache in memory, or we could also use a SQLite DB for that
# https://python.langchain.com/v0.1/docs/modules/model_io/chat/chat_model_caching/#sqlite-cache
import langchain
from langchain_openai.chat_models import ChatOpenAI

llm = ChatOpenAI(openai_api_key=openai_token)

# !pip install langchain-community
from langchain.cache import InMemoryCache, SQLiteCache
langchain.llm_cache = InMemoryCache()

# The first time, it is not yet in cache, so it should take longer
llm.invoke("Tell me a fact about Mars")
# You will notice this reply is instant!
llm.invoke('Tell me a fact about Mars')

```

### Prompt Templates

Notebook: [`00-Models-IO/01-Prompt-Templates.ipynb`](./00-Models-IO/01-Prompt-Templates.ipynb):

We can use prompt templates to define parametrized messages/instructions for the chatbot. These are an alternative to using f-string literals.

```python
### -- LLM Models

from langchain import PromptTemplate

# Instead of using f-string literals (add formating them),
# we can create prompt temaplates
# An example prompt with multiple input variables (but we can have no or one input)
multiple_input_prompt = PromptTemplate(
    input_variables=["topic", "level"], 
    template="Tell me a fact about {topic} for a student {level} level."
)
multiple_input_prompt.format(topic='Mars',level='8th Grade')
# 'Tell me a fact about Mars for a student 8th Grade level.'

### -- Chat Models

from langchain_openai import ChatOpenAI

chat = ChatOpenAI(openai_api_key=api_key)

# For chat type models, we need specific prmpt template classes
from langchain.prompts import (
    ChatPromptTemplate,
    PromptTemplate,
    SystemMessagePromptTemplate,
    AIMessagePromptTemplate,
    HumanMessagePromptTemplate,
)
from langchain.schema import (
    AIMessage,
    HumanMessage,
    SystemMessage
)

# We can automatically create a prompt from a string
system_template = "You are an AI recipe assistant that specializes in {dietary_preference} dishes that can be prepared in {cooking_time}."
system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
# input_variables is automatically extracted from the template
system_message_prompt.input_variables # ['cooking_time', 'dietary_preference']

human_template = "{recipe_request}"
human_message_prompt = HumanMessagePromptTemplate.from_template(human_template)
human_message_prompt.input_variables # human_message_prompt.input_variables

# Now, we can create a chat prompt template from the system and human message prompts
chat_prompt = ChatPromptTemplate.from_messages([system_message_prompt, human_message_prompt])
chat_prompt.input_variables

# Get a chat completion from the formatted messages
prompt = chat_prompt.format_prompt(cooking_time="15 min", dietary_preference="Vegan", recipe_request="Quick Snack").to_messages()
# [SystemMessage(...), HumanMessage(...)]

result = chat.invoke(prompt)
print(result.content)
# How about making a simple and delicious Avocado Toast? Here's a quick recipe for you: ...

### -- Exercise / Example

from langchain_openai.chat_models import ChatOpenAI
from langchain.prompts import (
    ChatPromptTemplate,
    PromptTemplate,
    SystemMessagePromptTemplate,
    AIMessagePromptTemplate,
    HumanMessagePromptTemplate,
)

def travel_idea(interest,budget):
    '''
    INPUTS:
        interest: A str interest or hobby (e.g. fishing)
        budget: A str budget (e.g. $10,000)
    '''
    # PART ONE: SYSTEM
    system_template="You are an AI Travel Agent that helps people plan trips about {interest} on a budget of {budget}"
    system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
    # PART TWO: HUMAN REQUEST
    human_template="{travel_help_request}"
    human_message_prompt = HumanMessagePromptTemplate.from_template(human_template)
    # PART THREE: COMPILE TO CHAT
    chat_prompt = ChatPromptTemplate.from_messages([system_message_prompt, human_message_prompt])
    # PART FOUR: INSERT VARIABLES
    request = chat_prompt.format_prompt(interest=interest, budget=budget, travel_help_request="Please give me an example travel itinerary").to_messages()
    # PART FIVE: CHAT REQUEST
    chat = ChatOpenAI(openai_api_key=api_key)
    result = chat.invoke(request)
    return result.content

```



## 3. Data Connections

## 4. Chains

## 5. Memory

## 6. Agents



