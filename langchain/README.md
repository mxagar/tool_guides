# LangChain Introduction

> LangChain is a framework for developing applications powered by language models. It enables applications that are:
>
> - Data-aware: connect a language model to other sources of data, i.e., we integrate external data
> - Agentic: allow a language model to interact with its environment via decision making.

The idea is that we have several modules with different functionalities which are used in chains to accomplish tasks. Check the official

- [use cases](https://python.langchain.com/docs/use_cases),
- list of [modules](https://python.langchain.com/docs/modules/),
- list oof [integrations](https://python.langchain.com/docs/integrations).

Note that LanChain uses models from other frameworks/vendors, such as OpenAI, Cohere, HuggingFace, etc; these are called **integrations**. As such, we need to have the specific keys or access tokens of those vendors. This mini-tutorial uses `dotenv` to load the keys defined in the `.env` file (not committed):

```
OPENAI_API_KEY=xxx
HUGGINGFACEHUB_API_TOKEN=xxx
```

Sources followed to prepare this guide:

- [LangChain Crash Course - Build apps with language models](https://www.youtube.com/watch?v=LbT1yp6quS8)
    - [Colab Notebook](https://colab.research.google.com/drive/1VOwJpcZqOXag-ZXi-52ibOx6L5Pw-YJi?usp=sharing#scrollTo=NkGGSdmtta6s)
- [The LangChain Cookbook - Beginner Guide To 7 Essential Concepts](https://www.youtube.com/watch?v=2xxziIWmaSA)
    - [Github: langchain-tutorials](https://github.com/gkamradt/langchain-tutorials)
- [The LangChain Cookbook Part 2 - Beginner Guide To 9 Use Cases](https://www.youtube.com/watch?v=vGP4pQdCocw)

Table of contents:

0. [Setup](#0-setup)
1. [LLMs](#1-llms)
2. [Prompt Templates](#2-prompt-templates)
3. [Chains](#3-chains)
4. [Agents and Tools](#4-agents-and-tools)
5. [Memory](#5-memory)
6. [Document Loaders and Vector Stores](#6-document-loaders-and-vector-stores)
7. [Examples, Use-Cases](#7-examples)
    - [Summarization Chain](#71-summarization-chain)
    - [Question & Answering Using Documents As Contex](#72-question--answering-using-documents-as-context)
    - [Extraction: Parse Information from a Document and Structure It](#73-extraction-parse-information-from-a-document-and-structure-it)
    - [Querying Tabular and SQL Data](#74-querying-tabular-and-sql-data)
    - [Code Understanding](#75-code-understanding)
    - [Interacting with APIs](#76-interacting-with-apis)
    - [Chatbots](#77-chatbots)

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

With LangChain we can easily access all integrated models from 3rd party vendors using a simple interface. We need to set the API keys or access tokens in the environment variables, e.g., in the `.env` file, which is loaded with `dotenv`.
**IMPORTANT NOTE**: Using those 3rd party APIs might cause costs! Additionally, if we have a free plan, some LLMs or functionalities might not be available.

```python
# OpenAI: environment variable OPENAI_API_KEY set
from langchain.llms import OpenAI

# default model_name="text-davinci-003"
# temperature: the larger 1, the more exaggerated/spicy
llm = OpenAI(temperature=0.9)
text = "Tell me a good company name for a company which sells potatoes."
print(llm(text))

# HuggingFace: environment variable HUGGINGFACEHUB_API_TOKEN set
from langchain import HuggingFaceHub

# https://huggingface.co/google/flan-t5-small
llm = HuggingFaceHub(repo_id="google/flan-t5-small",
                     model_kwargs={"temperature":0,
                                   "max_length":64})
text = "Translate English to Spanish: What time is it?"
print(llm(text))
```

### Schemas

Note that we usually interact with LLMs using text, i.e., text or natural language has become like a programming language; however, text is part of a larger set of interaction **schemas**:
 
- Text
- Chat messages, composed of
    - `SystemMessage` (context/instruction setting for AI), 
    - `HumanMessage` (what the human writes), 
    - and `AIMessages` (what the AI writes/responds).
- Documents: text and metadata.

### Models

Note that LLMs are in essence **models** where *text* goes in and *text* comes out. There are different kinds of models:

- Chat models: with the system, human and AI messages; e.g., `ChatOpenAI`.
- Text embeddings: text is converted to vectors that captured meaning e.g., `OpenAIEmbeddings`.

## 2. Prompt Templates

Usually we send prompts to the LLMs, not direct questions. Prompts are structured queries with instructions. We have tools to create them.

```python
# First, we need to instantiate an LLM, e.g., one from OpenAI
# OpenAI: environment variable OPENAI_API_KEY set
from langchain.llms import OpenAI

llm = OpenAI(temperature=0.9) # model_name="text-davinci-003"

print(llm("Can Barack Obama have a conversation with George Washington?"))

prompt = """Question: Can Barack Obama have a conversation with George Washington?

Let's think step by step.

Answer: """
print(llm(prompt))

from langchain import PromptTemplate

template = """Question: {question}

Let's think step by step.

Answer: """

prompt = PromptTemplate(template=template, input_variables=["question"])

prompt.format(question="Can Barack Obama have a conversation with George Washington?")

# We cannot pass directly a prompt to an llm
# We need to create Chain for that
print(llm(prompt))
```

### Other Types of Prompts

There are other types of prompts, too:

- Example selectors: we pass analogy pairs ("bird": "tree", "car": "stree") and the prompt template selects our pair given a new query object ("student") and creates a new prompt accordingly.
- Output parsers: a way to format the output from a model, which can sometimes be a JSON.

## 3. Chains

Chains are necessary to use prompts. Additionally, Chains make possible multi-step workflows.

```python
from langchain import LLMChain

llm_chain = LLMChain(prompt=prompt, llm=llm)

question = "Can Barack Obama have a conversation with George Washington?"

print(llm_chain.run(question))
```
## 4. Agents and Tools

LLMs alone cannot accomplish many tasks, such as web browsing (e.g., retrieving information from the Wikipedia) or math operations. With tools and agents we are able to carry out those tasks: we load different tools and ask an agent to use them with the help of an underlying LLM. So, in summary we have:

- Tools: functionalities to perform specific tasks: Python interpreter, math, Google search, Wikipedia API, Wolfram Alpha, etc.
- Agent: entity which performs the tasks using the tools.
- LLM: the LLM which powers the agent.

This feature/functionality is very similar to the OpenAI ChatGPT Plugins, only much more powerful because there are a lot of integrations.

```python
# The tool Wikipedia needs to be installed separately
!pip install wikipedia

from langchain.agents import load_tools
from langchain.agents import initialize_agent

# First, we instantiate an LLM, e.g., one from OpenAI,
# and load the required tools. See a list of integrated tools and agent toolkits:
#   https://python.langchain.com/docs/integrations/tools/
#   https://python.langchain.com/docs/integrations/toolkits/
# Note: Some require API keys
# Also, note the agent types:
#   https://python.langchain.com/docs/modules/agents/agent_types/
# Examples:
# - Math
# - Google search
# - Wikipedia
# - Python interpreter: https://python.langchain.com/docs/integrations/toolkits/python
# - Wolfram Alpha
# - ...
from langchain.llms import OpenAI

llm = OpenAI(temperature=0)
# We should list the tools we consider necessary, not more, because the agent needs to decide
# We sometimes can/need to pass to API keys in the parameters
tools = load_tools(["wikipedia", "llm-math"], llm=llm)

# Then, we instantiate an agent with the tools and the LLM
# Zero-Shot React agents use the description and the list of tools
# to decide on their own which tools to use
agent = initialize_agent(tools, llm, agent="zero-shot-react-description", verbose=True)

# The reasoning process/workflow and tool usage to obtain the answer is shown
agent.run("In what year was the film Departed with Leonardo Dicaprio released? What is this year raised to the 0.43 power?")
```

## 5. Memory

> Most LLM applications have a conversational interface. An essential component of a conversation is being able to refer to information introduced earlier in the conversation. At bare minimum, a conversational system should be able to access some window of past messages directly. A more complex system will need to have a world model that it is constantly updating, which allows it to do things like maintain information about entities and their relationships.
> 
> We call this ability to store information about past interactions "memory". LangChain provides a lot of utilities for adding memory to a system. These utilities can be used by themselves or incorporated seamlessly into a chain.

So basically, we add a state to Chains and Agents so that information persists between calls.

```python
from langchain import OpenAI, ConversationChain

llm = OpenAI(temperature=0)
# verbose=True: the conversation is remembered and displayed
conversation = ConversationChain(llm=llm, verbose=True)

conversation.predict(input="Hi there!")

conversation.predict(input="Can we talk about AI?")

conversation.predict(input="I'm interested in Reinforcement Learning.")
```
## 6. Document Loaders and Vector Stores

Document loaders enable loading external documents to be queried or used by the LLMs or the agents. After loading the documents, we usually need to index them, i.e., structure them so that LLMs can best interact with them. Utilities necessary for that:

- Embeddings: numerical/vectorized representation of a piece of information, for example, text, documents, images, audio, etc. Those vectors capture the meaning in number format.
- Text Splitters: long texts need to be split into chunks.
- Vector stores: vector databases store and index vector embeddings from NLP models; similarity searches cn be performed by performing dot products between query and database vectors.

### 6.1 Document Loaders

```python
# lxml and html2text are required to parse EverNote notes
!pip install lxml
!pip install html2text

# Important links with the document loaders:
#   https://python.langchain.com/docs/modules/data_connection/document_loaders/
#   https://python.langchain.com/docs/integrations/document_loaders/evernote
#       Evernote, CSV, HTML, URL, PDF, Google Drive, Github, Git, Arxiv, AWS, ...
from langchain.document_loaders import EverNoteLoader

# By default all notes are combined into a single Document
# I exported an Evernote note; we can also export an evernote notebook.
loader = EverNoteLoader("./data/test.enex")
loader.load()

# Markdown loading requires the following package
!pip install unstructured > /dev/null

from langchain.document_loaders import UnstructuredMarkdownLoader

markdown_path = "./README.md"
loader = UnstructuredMarkdownLoader(markdown_path)

data = loader.load()

data
```

### 6.2 Indexing and Vector Stores

In order to convert the text into embedding vectors we need a package

```python
# The HaggingFace embeddings use SBERT
!pip install sentence_transformers

# We need to have a vector database.
# Here, FAISS is used, but there are many other options:
#   https://python.langchain.com/docs/modules/data_connection/vectorstores.html
#   https://python.langchain.com/docs/integrations/vectorstores/
!pip install faiss-cpu

import requests

url = "https://github.com/hwchase17/chroma-langchain/blob/master/state_of_the_union.txt"
res = requests.get(url)
with open("data/state_of_the_union.txt", "w") as f:
    f.write(res.text)

# Document Loader
from langchain.document_loaders import TextLoader
loader = TextLoader('./data/state_of_the_union.txt')
documents = loader.load()

# Text Splitter: better to split big documents;
# try different parameters and select which best works for each case
from langchain.text_splitter import CharacterTextSplitter
text_splitter = CharacterTextSplitter(chunk_size=1000, # 1000 characters
                                      chunk_overlap=0) # no overlap between chunks
docs = text_splitter.split_documents(documents)

len(docs)

# Embeddings with HuggingFace and SBERT
# We instantiate the embeddings object, which is passed to the vector database
# to create embeddings
from langchain.embeddings import HuggingFaceEmbeddings
embeddings = HuggingFaceEmbeddings()

from langchain.vectorstores import FAISS
# Vector database is create with the text (docs) and the and an embeddings generator (embeddings)
db = FAISS.from_documents(docs, embeddings)

query = "What did the president say about Ketanji Brown Jackson"
docs = db.similarity_search(query)

print(docs[0].page_content)

# We can save locally teh database
db.save_local("data/faiss_index")
new_db = FAISS.load_local("data/faiss_index", embeddings)
docs = new_db.similarity_search(query)
print(docs[0].page_content)
```

## 7. Examples, Use-Cases

This section provides additional examples and use-cases.

### 7.1 Summarization Chain

This example shows to summarize arbitrarily long texts. Note that the usual promt/input + response text length is limited (e.g., to 4k tokens in OpenAI); however, we can split the text we'd like to summarize into chunks below that limit, compute the summaries of those, and the aggregate them to create an overall summary.

```python
!pip install tiktoken

# %%
from langchain.chains.summarize import load_summarize_chain
from langchain.document_loaders import TextLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.llms import OpenAI

llm = OpenAI(temperature=0)

loader = TextLoader('./data/work_pgraham.txt')
documents = loader.load()

# Show number of tokens
# Recall that many models have a limit of 4k tokens,
# so we should split the text
num_tokens = llm.get_num_tokens(documents[0].page_content)
print(f"Number of tokens: {num_tokens}")

# Get your splitter ready
text_splitter = RecursiveCharacterTextSplitter(chunk_size=700, chunk_overlap=50)

# Split your docs into texts
texts = text_splitter.split_documents(documents)

# There is a lot of complexity hidden in this one line.
# Check out this video for m ore details:
# https://www.youtube.com/watch?v=f9_BWhCI4Zo
chain = load_summarize_chain(llm, chain_type="map_reduce", verbose=True)
chain.run(texts)
```
### 7.2 Question & Answering Using Documents As Context

We give a context to our LLM and ask questions about it.

We can:

- chat with our documents,
- ask questions to academic papers,
- etc.

Example product: [ChatPDF](https://www.chatpdf.com/).

Another example: [How to query a book](https://www.youtube.com/watch?v=h0DHDp1FbmQ).

In order to be able to work with complex or long texts, we use **embeddings**.

```python
from langchain.llms import OpenAI

llm = OpenAI(temperature=0)

# The vectorstore we'll be using
from langchain.vectorstores import FAISS

# The LangChain component we'll use to get the documents
from langchain.chains import RetrievalQA

# The easy document loader for text
from langchain.document_loaders import TextLoader

# The embedding engine that will convert our text to vectors
from langchain.embeddings.openai import OpenAIEmbeddings

# Load the text/document(s)
loader = TextLoader('./data/worked_pgraham.txt')
doc = loader.load()
print (f"You have {len(doc)} document")
print (f"You have {len(doc[0].page_content)} characters in that document")

# Split the document(s)
text_splitter = RecursiveCharacterTextSplitter(chunk_size=3000, chunk_overlap=400)
docs = text_splitter.split_documents(doc)

# Get the total number of characters so we can see the average later
num_total_characters = sum([len(x.page_content) for x in docs])
print (f"Now you have {len(docs)} documents that have an average of {num_total_characters / len(docs):,.0f} characters.")

# Get your embeddings engine ready
embeddings = OpenAIEmbeddings()

# Embed your documents and combine with the raw text in a pseudo db. Note: This will make an API call to OpenAI
docsearch = FAISS.from_documents(docs, embeddings)

# Create retrieval engine
qa = RetrievalQA.from_chain_type(llm=llm, chain_type="stuff", retriever=docsearch.as_retriever())

# Query: Ask
query = "What does the author describe as good work?"
qa.run(query)
```
### 7.3 Extraction: Parse Information from a Document and Structure It

One common application would be to perform API or SQL calls from natural language. The next section is related to this use-case.

Example of popular extraction library: [Kor](https://eyurtsev.github.io/kor/).

```python
# To help construct our Chat Messages
from langchain.schema import HumanMessage
from langchain.prompts import PromptTemplate, ChatPromptTemplate, HumanMessagePromptTemplate

# We will be using a chat model, defaults to gpt-3.5-turbo
from langchain.chat_models import ChatOpenAI

# To parse outputs and get structured data back
from langchain.output_parsers import StructuredOutputParser, ResponseSchema

chat_model = ChatOpenAI(temperature=0, model_name='gpt-3.5-turbo')
```
#### Vanilla Extraction

```python
instructions = """
You will be given a sentence with fruit names, extract those fruit names and assign an emoji to them
Return the fruit name and emojis in a python dictionary
"""

fruit_names = """
Apple, Pear, this is an kiwi
"""

# Make your prompt which combines the instructions w/ the fruit names
prompt = (instructions + fruit_names)

# Call the LLM
output = chat_model([HumanMessage(content=prompt)])

# Note that an LLM can only output strings,
# thus we need to evaluate them in order to convert them into
# python objects, as in this case a dictionary
print (output.content)
print (type(output.content))

output_dict = eval(output.content)

print (output_dict)
print (type(output_dict))
```
#### Extraction with ResponseSchema

The LangChain `ResponseSchema`:

1. creates the prompt
2. and evaluates the LLM response to the propper Python object.

```python
# The schema I want out
response_schemas = [
    ResponseSchema(name="artist", description="The name of the musical artist"),
    ResponseSchema(name="song", description="The name of the song that the artist plays")
]

# The parser that will look for the LLM output in my schema and return it back to me
output_parser = StructuredOutputParser.from_response_schemas(response_schemas)

# The format instructions that LangChain makes. Let's look at them
format_instructions = output_parser.get_format_instructions()
print(format_instructions)

# The prompt template that brings it all together
# Note: This is a different prompt template than before because we are using a Chat Model
prompt = ChatPromptTemplate(
    messages=[
        HumanMessagePromptTemplate.from_template("Given a command from the user, extract the artist and song names \n \
                                                    {format_instructions}\n{user_prompt}")  
    ],
    input_variables=["user_prompt"],
    partial_variables={"format_instructions": format_instructions}
)

query = prompt.format_prompt(user_prompt="I really like So Young by Portugal. The Man")
print(query.messages[0].content)

output_llm = chat_model(query.to_messages())
output = output_parser.parse(output_llm.content)

print (output)
print (type(output))
```
### 7.4 Querying Tabular and SQL Data

Given a SQL database of a CSV/Pandas table, we can ask with natural language analysis queries which are transformed to the required query language, e.g., SQL.

More information: [Analyzing structured data](https://python.langchain.com/docs/use_cases/tabular).

```python
!pip install langchain-experimental

from langchain.llms import OpenAI
from langchain.utilities import SQLDatabase
from langchain_experimental.sql import SQLDatabaseChain

llm = OpenAI(temperature=0)

sqlite_db_path = './data/San_Francisco_Trees.db'
db = SQLDatabase.from_uri(f"sqlite:///{sqlite_db_path}")

db_chain = SQLDatabaseChain(llm=llm, database=db, verbose=True)

# Given the following prompt, these steps/tasks are carried ou automatically:
# 1. Find which table to use
# 2. Find which column to use
# 3. Construct the correct sql query
# 4. Execute that query
# 5. Get the result
# 6. Return a natural language reponse back
db_chain.run("How many Species of trees are there in San Francisco?")
```

### 7.5 Code Understanding

We can upload a complete codebase to the LLM and query it for function calls to specific tasks using natural language.

As an example, the code of the repository [The Fuzz](https://github.com/seatgeek/thefuzz) is used; this open source package can be used to detect text similarities using Levenshtein distances (so no LLMs).

```python
# Helper to read local files
import os

# Vector Support
from langchain.vectorstores import FAISS
from langchain.embeddings.openai import OpenAIEmbeddings

# Model and chain
from langchain.chat_models import ChatOpenAI

# Text splitters
from langchain.text_splitter import CharacterTextSplitter
from langchain.document_loaders import TextLoader

llm = ChatOpenAI(model_name='gpt-3.5-turbo')

embeddings = OpenAIEmbeddings(disallowed_special=())

root_dir = './data/thefuzz-master'
docs = []

# Go through each folder
for dirpath, dirnames, filenames in os.walk(root_dir):
    
    # Go through each file
    for file in filenames:
        try: 
            # Load up the file as a doc and split
            loader = TextLoader(os.path.join(dirpath, file), encoding='utf-8')
            docs.extend(loader.load_and_split())
        except Exception as e: 
            pass

print (f"You have {len(docs)} documents\n")
print ("------ Start Document ------")
print (docs[0].page_content[:300])

# Create document embeddings and persist in vector store
docsearch = FAISS.from_documents(docs, embeddings)

# Get our retriever ready
qa = RetrievalQA.from_chain_type(llm=llm, chain_type="stuff", retriever=docsearch.as_retriever())

query = "What function do I use if I want to find the most similar item in a list of items?"
output = qa.run(query)
print(output)

query = "Can you write the code to use the process.extractOne() function? Only respond with code. No other text or explanation"
output = qa.run(query)
print(output)
```
### 7.6 Interacting with APIs

We can upload the documentation of an API and query with natural language to the API; the endpoint calls (including parameters) are automatically performed and the results parsed.

```python
from langchain.chains import APIChain
from langchain.llms import OpenAI

llm = OpenAI(temperature=0)

api_docs = """

BASE URL: https://restcountries.com/

API Documentation:

The API endpoint /v3.1/name/{name} Used to find informatin about a country. All URL parameters are listed below:
    - name: Name of country - Ex: italy, france
    
The API endpoint /v3.1/currency/{currency} Uesd to find information about a region. All URL parameters are listed below:
    - currency: 3 letter currency. Example: USD, COP
    
Woo! This is my documentation
"""

chain_new = APIChain.from_llm_and_api_docs(llm, api_docs, verbose=True)

chain_new.run('Can you tell me information about france?')

chain_new.run('Can you tell me about the currency COP?')
```
### 7.7 Chatbots

We can create chatbots by leveraging the memory feature provided by LangChain.

For more information, check [Use-Cases: Chatbots](https://python.langchain.com/docs/use_cases/chatbots.html).

```python
from langchain.llms import OpenAI
from langchain import LLMChain
from langchain.prompts.prompt import PromptTemplate

# Chat specific components
from langchain.memory import ConversationBufferMemory

template = """
You are a chatbot that is unhelpful.
Your goal is to not help the user but only make jokes.
Take what the user is saying and make a joke out of it

{chat_history}
Human: {human_input}
Chatbot:"""

prompt = PromptTemplate(
    input_variables=["chat_history", "human_input"], 
    template=template
)
memory = ConversationBufferMemory(memory_key="chat_history")

llm_chain = LLMChain(
    llm=OpenAI(), 
    prompt=prompt, 
    verbose=True, 
    memory=memory
)

llm_chain.predict(human_input="Is an pear a fruit or vegetable?")

llm_chain.predict(human_input="What was one of the fruits I first asked you about?")
```
