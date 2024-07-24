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
  - [2. Model IO](#2-model-io)
    - [Large Language Models (LLMs) and Chatbots](#large-language-models-llms-and-chatbots)
    - [Prompt Templates](#prompt-templates)
    - [Few Shot Prompt Templates](#few-shot-prompt-templates)
    - [Parsing Outputs](#parsing-outputs)
    - [Serialization of Prompts: Saving and Loading](#serialization-of-prompts-saving-and-loading)
  - [3. Data Connections](#3-data-connections)
    - [Document Loading](#document-loading)
    - [Document Transformers](#document-transformers)
    - [Text Embedding](#text-embedding)
    - [Vector Stores](#vector-stores)
    - [Retrievers and Multi-Query Retrievers](#retrievers-and-multi-query-retrievers)
    - [Context Compression](#context-compression)
    - [Example: US Constitution Helper](#example-us-constitution-helper)
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

First, create a Python environment, e.g, with Conda:

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

## 2. Model IO

With the Model IO module, we can easily interchange the underlying models; we can even benchmark them before choosing one.

Contents:

- LLMs and Chatbots
- Prompt Templates
- Few Shot Prompt Templates
- Parsing Outputs
- Serialization: Saving and Loading Prompts

### Large Language Models (LLMs) and Chatbots

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

### Few Shot Prompt Templates

Notebook: [`00-Models-IO/04-Few-Shot-Prompt-Templates.ipynb`](./00-Models-IO/04-Few-Shot-Prompt-Templates.ipynb).

Few-show prompting means giving some examples to the chatbot in the prompt. There is a way of achieveing that with the presented tools.

```python
from langchain_openai.chat_models import ChatOpenAI
from langchain import PromptTemplate, LLMChain
from langchain.prompts.chat import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    AIMessagePromptTemplate,
    HumanMessagePromptTemplate,
)
from langchain.schema import AIMessage, HumanMessage, SystemMessage

# System prompt: we define the role and overall task
template = "You are a helpful assistant that translates complex legal terms into plain and understandable terms."
system_message_prompt = SystemMessagePromptTemplate.from_template(template)

# Noe we need example input and output to train the model, i.e., Human-AI pairs
legal_text = "The provisions herein shall be severable, and if any provision or portion thereof is deemed invalid, illegal, or unenforceable by a court of competent jurisdiction, the remaining provisions or portions thereof shall remain in full force and effect to the maximum extent permitted by law."
example_input_one = HumanMessagePromptTemplate.from_template(legal_text)
plain_text = "The rules in this agreement can be separated. If a court decides that one rule or part of it is not valid, illegal, or cannot be enforced, the other rules will still apply and be enforced as much as they can under the law."
example_output_one = AIMessagePromptTemplate.from_template(plain_text)

# Now, we define the real human prompt we're going to use, which is basically the legal text!
human_template = "{legal_text}"
human_message_prompt = HumanMessagePromptTemplate.from_template(human_template)

# Finally, the complete prompt is the list
# system_message_prompt, example_input_one, example_output_one, human_message_prompt
chat_prompt = ChatPromptTemplate.from_messages(
    [system_message_prompt, example_input_one, example_output_one, human_message_prompt]
)

chat_prompt.input_variables # ['legal_text']

# Now, we format the prompt with the variable
some_example_text = "The grantor, being the fee simple owner of the real property herein described, conveys and warrants to the grantee, his heirs and assigns, all of the grantor's right, title, and interest in and to the said property, subject to all existing encumbrances, liens, and easements, as recorded in the official records of the county, and any applicable covenants, conditions, and restrictions affecting the property, in consideration of the sum of [purchase price] paid by the grantee."
request = chat_prompt.format_prompt(legal_text=some_example_text).to_messages()

result = chat.invoke(request)
print(result.content) # The person giving the property, who owns it completely, is transferring and...
```

### Parsing Outputs

Notebook: [`00-Models-IO/05-Parsing-Output.ipynb`](./00-Models-IO/05-Parsing-Output.ipynb).

Sometimes we want to structure the output from the chat in a certain way, e.g., convert it into a JSON, a datetime object, etc. We have tools to achieve that. Some format parsers:

- JSON
- XML
- CSV: CommaSeparatedListOutputParser
- YAML
- PandasDataFrame
- Enum
- Pydantic
- Datetime
- ...

Sometimes the output parsers don't work because the model outputs some more text. We can fix that with several methods:

- System prompt: we explicitly highlight the format.
- We can use `OutputFixingParser`, which re-requests to the model to reformat the answer.

Also, regarding the `Pydantic` parser/format, check the [OpenAI Function Calling](https://platform.openai.com/docs/guides/function-calling):

> In an API call, you can describe functions and have the model intelligently choose to output a JSON object containing arguments to call one or many functions. The Chat Completions API does not call the function; instead, the model generates JSON that you can use to call the function in your code. Example: `extract_data(name: string, birthday: string)`.

```python
from langchain.prompts import (
    PromptTemplate, 
    SystemMessagePromptTemplate,
    ChatPromptTemplate, 
    HumanMessagePromptTemplate
)
from langchain_openai import ChatOpenAI

model = ChatOpenAI(openai_api_key=api_key)

### -- List Parsing

# List Parsing: we want to have a list as an output
# We can browse all the parsers with TAB 
# https://python.langchain.com/v0.1/docs/modules/model_io/output_parsers/
# - JSON
# - XML
# - CSV: CommaSeparatedListOutputParser
# - YAML
# - PandasDataFrame
# - Enum
# - Pydantic
# - Datetime
# - ...
from langchain.output_parsers import CommaSeparatedListOutputParser
output_parser = CommaSeparatedListOutputParser()

# The instructions are really a string 
format_instructions = output_parser.get_format_instructions()
print(format_instructions) # Your response should be a list of comma separated values, eg: `foo, bar, baz` or `foo,bar,baz`

# However, the parser has also the method parse() which parses a correctly formatted string
reply = "one, two, three"
output_parser.parse("one, two, three") # ['one', 'two', 'three']

# The prompt is a string with two placeholders: {request} and {format_instructions}
human_template = '{request}\n{format_instructions}' # \n new line is a good idea
human_prompt = HumanMessagePromptTemplate.from_template(human_template)

# Now, we can create a chat prompt
chat_prompt = ChatPromptTemplate.from_messages([human_prompt])
chat_prompt.format_prompt(request="give me 5 characteristics of dogs",
                   format_instructions = output_parser.get_format_instructions())

# Note: the request needs to line up with the instructions!
request = chat_prompt.format_prompt(
    request="give me 5 characteristics of dogs",
    format_instructions = output_parser.get_format_instructions()).to_messages()

result = model.invoke(request)

# We get back a string but it should follow the instructions
result.content # 'Loyal, playful, protective, social, obedient'

# Convert to desired output
output_parser.parse(result.content) # ['Loyal', 'friendly', 'playful', 'protective', 'intelligent']

### -- Pydantic Objects

from langchain.output_parsers import PydanticOutputParser
from pydantic import BaseModel, Field

# First, we define the Pydantic class
# We want to get objects that line up with these fields
class Scientist(BaseModel):
    name: str = Field(description="Name of a Scientist")
    discoveries: list = Field(description="Python list of discoveries")

# This is our NL query
query = 'Name a famous scientist and a list of their discoveries' 

# Pydantic parser
parser = PydanticOutputParser(pydantic_object=Scientist)

print(parser.get_format_instructions()) # The output should be formatted as a JSON instance...

# Prompt template
prompt = PromptTemplate(
    template="Answer the user query.\n{format_instructions}\n{query}\n",
    input_variables=["query"],
    partial_variables={"format_instructions": parser.get_format_instructions()},
)

# We modify the template with the query
input_prompt = prompt.format_prompt(query=query)
# Run the model
output = model.invoke(input_prompt.to_string()) # Sometimes lowering the temperature helps
# Parse
parser.parse(output.content) # A Scientist object is returned!
```

### Serialization of Prompts: Saving and Loading

Notebook: [`00-Models-IO/06-Serialization-Saving-Prompts.ipynb`](./00-Models-IO/06-Serialization-Saving-Prompts.ipynb).

We save prompt templates as JSON files and load them later.

```python
import os
from langchain.prompts import (
    PromptTemplate,
    ChatPromptTemplate,
    HumanMessagePromptTemplate
)
from langchain import PromptTemplate

from langchain_openai import OpenAI
model = OpenAI(openai_api_key=api_key)


# We can save a prompt template to a JSON with .save()
template = "Question: {question}\n\nAnswer: Let's think step by step."
prompt = PromptTemplate(template=template, input_variables=["question"])
prompt.save("prompt.json")

# All prompts are loaded through the `load_prompt` function.
from langchain.prompts import load_prompt
loaded_prompt = load_prompt('prompt.json')
loaded_prompt # PromptTemplate(input_variables=['question'],

```

The JSON file [`prompt.json`](./00-Models-IO/prompt.json):

```json
{
    "name": null,
    "input_variables": [
        "question"
    ],
    "optional_variables": [],
    "input_types": {},
    "output_parser": null,
    "partial_variables": {},
    "metadata": null,
    "tags": null,
    "template": "Question: {question}\n\nAnswer: Let's think step by step.",
    "template_format": "f-string",
    "validate_template": false,
    "_type": "prompt"
}
```

## 3. Data Connections

Contents:

- Document Loading and Integrations
- Document Transformers
- Text Embedding
- Vector Stores
- Queries and Retrievers

### Document Loading

There are many [Document Loaders](https://python.langchain.com/v0.2/docs/integrations/document_loaders/) available in LangChain: CSV, PDF, Word, TXT, HTML, etc. We need to manually install the dependencies of each of them.

Similarly, we have many [Integrations](https://python.langchain.com/v0.2/docs/integrations/platforms/): AWS S3, ElasticSearch, MongoDB, IBM, Pinecone, Wikipedia,s etc. Each integration is different and we should look at the documentation. In some cases, the integrations might be broken, because the providers have changed something and LangChain has not updated the interfaces yet.

Usually, any document format (CSV, HTML, PDF, etc.) is loaded as a `Document` object.

Notebooks: 

- [`01-Data-Connections/00-Document-Loaders.ipynb`](./01-Data-Connections/00-Document-Loaders.ipynb): CSV, HTML, PDF
- [`01-Data-Connections/01-Document-Integration-Loader.ipynb`](./01-Data-Connections/01-Document-Integration-Loader.ipynb): Hacker News integration and example
- [`01-Data-Connections/03-Document-Loading-Exercise-Solution.ipynb`](./01-Data-Connections/03-Document-Loading-Exercise-Solution.ipynb): Wikipedia integration and example

```python
### -- CSV
from langchain.document_loaders import CSVLoader

# Lazy loader: not loaded until .load() is called
loader = CSVLoader('some_data/penguins.csv')

# Now occurs the loading
data = loader.load()

type(data) # List of Document objects
# We work with Document objects, which are essentially dictionaries with some extra methods
# No matter the format, the data is always returned as a list of Document objects

# Document 0
data[0]

# Print metadata and the content of the first document
print(data[0].metadata) # {'source': 'some_data/penguins.csv', 'row': 0}
print(data[0].page_content)
# species: Adelie
# island: Torgersen
# ...

### -- HTML

from langchain.document_loaders import BSHTMLLoader

loader = BSHTMLLoader('some_data/some_website.html')
data = loader.load()

### -- PDF

# PDFs in particular, might lead to reading errors, e.g., 
# wrong formatting, images replaed by placeholders, etc.
# We can always use the PyPDF2 library to extract the text from a PDF file
# and the load the text as a Document
from langchain.document_loaders import PyPDFLoader

loader = PyPDFLoader('some_data/SomeReport.pdf')
pages = loader.load_and_split()

pages[0]
print(pages[0].page_content)

### -- Integrations - Example: Hacker News

from langchain.prompts import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
)
from langchain_openai.chat_models import ChatOpenAI

model = ChatOpenAI(openai_api_key=api_key)

human_prompt = HumanMessagePromptTemplate.from_template('Please give me a single sentence summary of the following:\n{document}')

chat_prompt = ChatPromptTemplate.from_messages([human_prompt])

result = model.invoke(chat_prompt.format_prompt(document=data[0].page_content).to_messages())

result.content
# 'Nicholast was a multi-talented individual with interests in jazz music, juggling, unicycling, and inventing various contraptions, whose papers are still relevant in fields such as information theory and artificial intelligence.'

### -- Integrations - Example: Wikipedia

# https://python.langchain.com/v0.2/docs/integrations/document_loaders/wikipedia/

from langchain.prompts import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
)
from langchain_openai.chat_models import ChatOpenAI
from langchain.document_loaders import WikipediaLoader


def answer_question_about(person_name: str, question: str, model=None) -> str:
    '''
    Use the Wikipedia Document Loader to help answer questions about someone,
    insert it as additional helpful context.
    '''
    # Get Wikipedia Article
    docs = WikipediaLoader(query=person_name,load_max_docs=1)
    context_text = docs.load()[0].page_content
    
    # Connect to OpenAI Model
    if model is None:
        #load_dotenv()
        api_key = os.getenv("OPENAI_API_KEY")
        model = ChatOpenAI(openai_api_key=api_key)
    
    # Ask Model Question
    human_prompt = HumanMessagePromptTemplate.from_template('Answer this question\n{question}, here is some extra context:\n{document}')
    
    # Assemble chat prompt
    chat_prompt = ChatPromptTemplate.from_messages([human_prompt])
    
    #result
    result = model.invoke(chat_prompt.format_prompt(question=question,document=context_text).to_messages())
    
    print(result.content)

model = ChatOpenAI(openai_api_key=api_key)
answer_question_about("Claude Shannon", "When was he born?", model)

```

### Document Transformers

Document transformers split document strings into chunks so that they can be processed later on. Keep in mind these chunks and usually encoded into vector embeddings and the length of the sequences fed to the encoder models has often a limit.

Notebook: [`01-Data-Connections/04-Document-Transformers.ipynb`](./01-Data-Connections/04-Document-Transformers.ipynb).

```python
# https://api.python.langchain.com/en/latest/character/langchain_text_splitters.character.CharacterTextSplitter.html
from langchain.text_splitter import CharacterTextSplitter
# We can use the CharacterTextSplitter to split the text into smaller chunks
# - based on separator characters (e.g., new line) or the number of characters in each chunk 
# - based on the number of tokens in each chunk

with open('some_data/FDR_State_of_Union_1944.txt') as file:
    speech_text = file.read()

# Characters: 21995
len(speech_text)

# Words: 3750
len(speech_text.split())

### -- Split by character

# We can separate upon a particular character: \n\n, ##, etc.
text_splitter = CharacterTextSplitter(
    separator="\n\n", # default
    chunk_size=1000 # 1000 is default value
)
# The text is split into chunks when separator=\n\n is found or every chunk_size=1000 characters
# However, if the characters between the separators are much less than chunk_size, the separators are ignored.
# In fact, I think chunk_size preceeds?

texts = text_splitter.create_documents([speech_text])
print(type(texts)) # list
print(type(texts[0])) # Document
print(len(texts[0].page_content.split())) # 152 words
print('\n')
print(texts[0].page_content) # complete chunk text

### -- Split by Token

# Now chunk size is a hard length based on tokens
text_splitter = CharacterTextSplitter.from_tiktoken_encoder(chunk_size=500)

texts = text_splitter.split_text(speech_text)

# In contrast to the previous CharacterTextSplitter,
# if we use tiktoken the output is a text string!
type(texts[0]) # str

print(len(texts[0].split())) # 411 words ~ 500 tokens
```

### Text Embedding

Text embeddings are vector representations of texts. Usually semantic embeddings are used, i.e., similar texts yield similar vectors, in such a way, that algebra operations are possible with the vectors.

This guide uses [OpenAI Text Embeddings](https://python.langchain.com/v0.1/docs/integrations/text_embedding/openai/), but many more are possible.

Embedding models are not compatible between each other.

Notebook: [`01-Data-Connections/05-Text-Embedding.ipynb`](./01-Data-Connections/05-Text-Embedding.ipynb).

```python
import numpy as np
from langchain_openai.embeddings import OpenAIEmbeddings
from langchain.document_loaders import CSVLoader

# Embedding model
# To use the OpenAI model, we need to have defined OPENAI_API_KEY as environment variable
# https://platform.openai.com/docs/guides/embeddings
# Default model: text-embedding-ada-002
# Dimension: 1536
embeddings = OpenAIEmbeddings()

text = "Some normal text to send to OpenAI to be embedded into a N dimensional vector"
embedded_text = embeddings.embed_query(text)

# The returned vector is a list
type(embedded_text) # list
len(embedded_text) # 1536

# We can convert the list into an array
embedding_array = np.array(embedded_text)
embedding_array.shape # (1536,)

## -- Embedding a Document

loader = CSVLoader('some_data/penguins.csv')
# This loads a list of Document objects
# Each Document is a row in the CSV
data = loader.load()
type(data) # list
type(data[0]) # langchain_core.documents.base.Document

# Unfortunately, we cannot pass a list of Documents
# but we need to pass a list of strings, i.e., page_content
embedded_docs = embeddings.embed_documents([text.page_content for text in data])

len(embedded_docs) # 344 (rows)
len(embedded_docs[0]) # Embedding dimension
```

### Vector Stores

We want to persist/store embedding vectors and be able to access them easily, even with similarity queries. We can do that with a **vector store**:

- We can store large N-dimensional vectors.
- We can index a vector and its text string.
- Given a new query vector not in the DB, we can run similarity searches in the DB.
- We can easily add/update/delete vectors.

Popular vector stores:

- [ChromaDB](https://www.trychroma.com/)
- [FAISS (Meta): Facebook AI Similarity Search](https://github.com/facebookresearch/faiss)
- [Elasticsearch with k-NN Plugin](https://www.elastic.co/guide/en/elasticsearch/reference/current/knn-search.html)
- [Pinecone](https://www.pinecone.io/)
- [Weaviate](https://weaviate.io/)
- [ScaNN (Google)](https://github.com/google-research/google-research/tree/master/scann)
- [Annoy (Spotify): Approximate Nearest Neighbors Oh Yeah](https://github.com/spotify/annoy)
- [PGVector: Vector Plugin for Postgres](https://python.langchain.com/v0.2/docs/integrations/vectorstores/pgvector/)

Here, we use **ChromaDB**.

Notebook: [`01-Data-Connections/06-Vector-Store.ipynb`](./01-Data-Connections/06-Vector-Store.ipynb).

```python
from langchain_openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma
from langchain.document_loaders import TextLoader

# Load the document and split it into chunks
loader = TextLoader("some_data/FDR_State_of_Union_1944.txt")
documents = loader.load()

# Split it into chunks
text_splitter = CharacterTextSplitter.from_tiktoken_encoder(chunk_size=500)
docs = text_splitter.split_documents(documents)

# The variable OPENAI_API_KEY needs to bet set in the environment
embedding_function = OpenAIEmbeddings()

# Load it into Chroma; we pass:
# - the documents
# - the embedding function or Embeddings class/object
# - the directory where the DB is persisted
db = Chroma.from_documents(docs,
                           embedding_function,
                           persist_directory='./speech_embedding_db')
# The persisted folder contains
# - a SQLite database
# - two parquet files with the embeddings and the texts
# - index files

# Here, we load a persisted DB, i.e.,
# we CONNECT to persisted DB
db_connection = Chroma(persist_directory='./speech_embedding_db',
                       embedding_function=embedding_function)

# Query the DB
# WARNING: Do not use questions, but rather statements
# because this is not a chat, but a similarity search
# based on the cosine similarit of the embeddings
new_doc = "What did FDR say about the cost of food law?"

# We can do it with the string or the vector of the string
# Default top k = 4
# We get as result Document objects of the chunks
# that were passed to the vector store
docs = db_connection.similarity_search(new_doc)

len(docs) # 4
type(docs[0]) # langchain_core.documents.base.Document

# Show the content of the first document
print(docs[0].page_content) # A continuation ...

# Important: for each chunk, we can get the original document
print(docs[0].metadata) # {'source': 'some_data/FDR_State_of_Union_1944.txt'}
```


### Retrievers and Multi-Query Retrievers

Vector stores are passed as retrievers to different modules when we use them.

When we convert a vector store into a retriever, we get access to standardized retriever methods that can be used for querying and searching.

Additionally, in some cases, the phrasing in our query is not optimal; we can use an LLM to generate multiple variations of it. That way, we rather focus on ideas and not in the exact phrasing. That is achieved with **MultiQueryRetrievers**. Note, however, that in a Multi-Query Retriever an LLM generates query variations, so we should decrease its temeprature if we want to have reproducible results.

Notebooks: 

- [`01-Data-Connections/07-Vector-Store-Retriever.ipynb`](./01-Data-Connections/07-Vector-Store-Retriever.ipynb).
- [`01-Data-Connections/08-Multi-Query-Retriever.ipynb`](./01-Data-Connections/08-Multi-Query-Retriever.ipynb).

```python
from langchain.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings

# Load vector store
# We need to have OPENAI_API_KEY in the environment
# The DB contains the Wikipedia page on the MKUltra project from the CIA
embedding_function = OpenAIEmbeddings()
db_connection = Chroma(
    persist_directory='./mk_ultra',
    embedding_function=embedding_function
)

# When we convert our vector store to a retriever,
# we get access to standardized retriever methods
# that can be used for querying and searching
retriever = db_connection.as_retriever()

search_kwargs = {
    "score_threshold":0.8,
    "k":4
}

# This gets the top 4 documents
# that are most similar to the query "President"
docs = retriever.invoke(
    "President",
    search_kwargs=search_kwargs
)

print(len(docs)) # 4
docs[0].page_content # The United States President...
```

**This is the continuation of the previous notebook:**

```python
# We have db_connection already loaded in previous code block
# db_connection = Chroma(...)
# The DB contains the Wikipedia page on the MKUltra project from the CIA

# For the MultiQueryRetriever we need to associated module
# and a chat model
from langchain_openai import ChatOpenAI
from langchain.retrievers.multi_query import MultiQueryRetriever

# Instance of a chat model
# It is advisable to use temperature=0
# in order to be able to replicate the results,
# i.e., otherwise the query variations might differ
# and lead to different results...
llm = ChatOpenAI(temperature=0)

# MultiQueryRetriever: we need a retriever and a chat model
retriever_from_llm = MultiQueryRetriever.from_llm(
    retriever=db_connection.as_retriever(),
    llm=llm
)

# Set logging for the queries
# to see which queries are formed
import logging

logging.basicConfig()
logging.getLogger('langchain.retrievers.multi_query').setLevel(logging.INFO)

# Initial query: This is the seed query which will be used to generate the queries
question = "When was this declassified?"

# This will return the most relevant documents
# related to the query, NOT the multiple queries
# However, since we activated the logging with level INFO
# we see the different queries that are formed from our initial
# question
unique_docs = retriever_from_llm.invoke(input=question)
# INFO:langchain.retrievers.multi_query:Generated queries: 
# ['1. What is the date of the declassification of this information?', 
# '2. Can you provide the specific time when this was declassified?', 
# '3. Do you know the exact moment when this information became declassified?']

len(unique_docs) # 5 (top 5 unique documents)

print(unique_docs[0].page_content) # The Church Committee (formally...
```

### Context Compression

It is possible to compress/distill/summarize the retrieved results using an LLM. This often used more than the multi-query retrieval.

Notebook: [`01-Data-Connections/09-Context-Compression.ipynb`](./01-Data-Connections/09-Context-Compression.ipynb).

```python
# Build a sample vectorDB
from langchain.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings

# We need to have OPENAI_API_KEY in the environment
embedding_function = OpenAIEmbeddings()

# Load/connect DB
db_connection = Chroma(
    persist_directory='./mk_ultra',
    embedding_function=embedding_function
)

from langchain_openai import ChatOpenAI
from langchain.retrievers import ContextualCompressionRetriever
from langchain.retrievers.document_compressors import LLMChainExtractor

# We need an LLM chat
llm = ChatOpenAI(temperature=0)
# We create the compressor with the chat
compressor = LLMChainExtractor.from_llm(llm)

# The compression retriever consists of
# - a base compressor (from an LLM/chat)
# - a retriever (from a DB)
compression_retriever = ContextualCompressionRetriever(
    base_compressor=compressor, 
    base_retriever=db_connection.as_retriever()
)

# Similarity search in the DB
docs = db_connection.similarity_search('When was this declassified?')

docs[0] # Document(page_content='The United States President

# Here, we use the compression retriever to summarize the document:
# - Documents are retrieved according to the query
# - Results are sent to the LLM to summarize
# - The summarized results are presented
compressed_docs = compression_retriever.invoke("When was this declassified?")compressed_docs[0].page_contents
```


### Example: US Constitution Helper

Notebook: [`01-Data-Connections/11-Data-Connections-Exercise-Solution.ipynb`](./01-Data-Connections/11-Data-Connections-Exercise-Solution.ipynb).

The US Constitution is ingested/vectorized and we can ask questions to it.

I refactored the proposed solution.

```python
from langchain.vectorstores import Chroma
from langchain_openai import ChatOpenAI
from langchain.document_loaders import TextLoader
from langchain_openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.retrievers import ContextualCompressionRetriever
from langchain.retrievers.document_compressors import LLMChainExtractor 
from typing import List
from langchain.schema import Document

import os
from dotenv import load_dotenv
load_dotenv()

openai_token = os.getenv("OPENAI_API_KEY")

class QABot:
    def __init__(self, persist_directory: str):
        self.persist_directory = persist_directory
        self.docs: List[Document] = []
        self.db = None
        self.llm = None
        self.retriever = None

    def load_text_file(self, file_path: str, chunk_size: int = 500) -> None:
        # Load the text file into a Document object
        loader = TextLoader(file_path)
        documents = loader.load()

        # Split the document into chunks
        text_splitter = CharacterTextSplitter.from_tiktoken_encoder(chunk_size=chunk_size)
        self.docs = text_splitter.split_documents(documents)
        
        # Embed the documents and create a persisted ChromaDB
        embedding_function = OpenAIEmbeddings()
        self.db = Chroma.from_documents(self.docs,
                                        embedding_function,
                                        persist_directory=self.persist_directory)
        #self.db.persist()
    
    def _connect_llm(self) -> ChatOpenAI:
        # Initialize and return the LLM chat model
        self.llm = ChatOpenAI(temperature=0.0)
        return self.llm

    def _get_retriever(self) -> ContextualCompressionRetriever:
        # Initialize the LLM compressor
        if self.llm is None:
            self._connect_llm()
        compressor = LLMChainExtractor.from_llm(self.llm)

        # Create and return the contextual compression retriever
        self.retriever = ContextualCompressionRetriever(base_compressor=compressor, base_retriever=self.db.as_retriever())
        return self.retriever

    def ask(self, question: str) -> str:
        # Ensure the LLM and retriever are loaded
        if self.retriever is None:
            self._get_retriever()

        # Use the retriever to get the most relevant part of the documents
        compressed_docs = self.retriever.invoke(question)
        return compressed_docs[0].page_content if compressed_docs else "No relevant content found."

# First, load the documents and create a specific chatbot
qa_bot = QABot(persist_directory='./US_Constitution')
qa_bot.load_text_file("some_data/US_Constitution.txt")

# Then, ask questions to the chatbot
answer = qa_bot.ask("What is the 1st Amendment?")
print(answer) # First Amendment: Congress shall make no law respecting an establishment of religion...
```



## 4. Chains

## 5. Memory

## 6. Agents



