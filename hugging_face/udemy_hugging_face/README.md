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
    - [Account Setup and Model Repository](#account-setup-and-model-repository)
    - [Login from Notebooks](#login-from-notebooks)
    - [Understanding Models and Spaces](#understanding-models-and-spaces)
    - [Datasets](#datasets)
    - [Compute Services with GPUs](#compute-services-with-gpus)
    - [Cache Directories](#cache-directories)
  - [2. NLP with Transformers](#2-nlp-with-transformers)
    - [Pipelines](#pipelines)
    - [Large Language Models (LLMs)](#large-language-models-llms)
    - [Tokenization, Probablities and Text Generation](#tokenization-probablities-and-text-generation)
  - [3. Image Models: Diffusers](#3-image-models-diffusers)
    - [Basic Image Processing](#basic-image-processing)
    - [Image Generation: Text-to-Image and Diffusion Models](#image-generation-text-to-image-and-diffusion-models)
    - [Diffusion Models with Python](#diffusion-models-with-python)
    - [Auto-Pipelines](#auto-pipelines)
  - [4. Video Models](#4-video-models)
    - [Stable Video Diffusion: Image-to-Video](#stable-video-diffusion-image-to-video)
    - [I2VGenXL: Image-and-Text-to-Video](#i2vgenxl-image-and-text-to-video)
  - [5. Audio Models](#5-audio-models)
    - [Understanding Audio Data](#understanding-audio-data)
  - [6. Gradio for User Interfaces](#6-gradio-for-user-interfaces)

## 1. Introduction to Hugging Face

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

# Install the transformers libraries & Co.
pip install transformers diffusers datasets evaluate accelerate

# For CPU support only:
pip install 'transformers[torch]' diffusers datasets accelerate evaluate

# If you want to install the HuggingFace Hub conection
# This is only necessary if we need programmatic access,
# for which there is also a CLI tool
pip install huggingface_hub

# Audio analysis
pip install librosa
```

However, most of the examples in this guide were carried out in online services with GPUs, such as

- Google Colab
- AWS SageMaker Studio Lab

Note: since the libraries are benig updated so frequently, we often need to upgrade or even downgrade (because compatility is broken) the library versions. AWS SageMaker Studio Lab allows setting our own environments, but Google Colab requires us to do that manually.

```bash
# Install desired version
!pip uninstall transformers -y
!pip install transformers==4.41.0

# Install any version if missing
!pip install transformers diffusers torch torchvision
```

In order to open a notebook from the present repository in Google Colab:

    https://colab.research.google.com/github/<usename>/<path>

Finally, bear in mind that sometimes models are very large and we occupy the complete GPU memory (specially with video models). We can clear the memory by removig unused models with the following lines:

```python
import torch
torch.cuda.empty_cache()
```

### Account Setup and Model Repository

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

### Login from Notebooks

If we are using remote hosted notebook (e.g., Google Colab or AWS SageMaker Studio Lab), we can use `notebook_login()` to log in to Hugging Face:

```python
from huggingface_hub import notebook_login
notebook_login()
# A box is shown where we can paste our HF token
```

### Understanding Models and Spaces

If we click on **Models**, we're going to see popular available models; we can also sort them by different criteria.
Most popular (treding):

- [meta-llama/Meta-Llama-3-8B](https://huggingface.co/meta-llama/Meta-Llama-3-8B/tree/main)
- [meta-llama/Meta-Llama-3-8B-Instruct](https://huggingface.co/meta-llama/Meta-Llama-3-8B-Instruct)

Most downloaded:

- [MIT/ast-finetuned-audioset-10-10-0.4593](https://huggingface.co/MIT/ast-finetuned-audioset-10-10-0.4593/tree/main): Audio classification
- [sentence-transformers/all-MiniLM-L12-v2](https://huggingface.co/sentence-transformers/all-MiniLM-L12-v2/tree/main)
- [openai/clip-vit-large-patch14](https://huggingface.co/openai/clip-vit-large-patch14/tree/main): Zero-shot image classification; image-text similary scores, etc.
- [facebook/fasttext-language-identification](https://huggingface.co/facebook/fasttext-language-identification/tree/main): Text classification; language can be identified and texts classified.
- [google-bert/bert-base-uncased](https://huggingface.co/google-bert/bert-base-uncased/tree/main)
- [distilbert/distilbert-base-uncased](https://huggingface.co/distilbert/distilbert-base-uncased/tree/main): distllied version of BERT.
- [openai/whisper-small](https://huggingface.co/openai/whisper-small/tree/main): Automatic Speech Recognition (ASR).
- [openai-community/gpt2](https://huggingface.co/openai-community/gpt2/tree/main)
- [microsoft/trocr-base-handwritten](https://huggingface.co/microsoft/trocr-base-handwritten/tree/main)
- ...

We can also select the type of method/use-case we'd like and explore available models (e.g., *text-to-3d*).

![Models](./assets/models.png)

Further info available:

- Model card
- Weights
- Code
- Paper links
- Discussions
- etc.

Also we can

- train a model on AWS SageMaker
- deploy it to different providers: AWS, Azure, Google, etc.
- or use it the HF packages `transformers` (text data) or `diffusers` (image data).

![Deploy](./assets/deploy.png)

If we publish our model to HF Models, we can attach it to a **Space**, which is a compute service that runs the model! These spaces can be used to try the models; however, not all models have spaces attached and spaces of popular models might be overloaded, so they take a very long time to produce an output. Use it extensively!

### Datasets

HF hosts also datasets; we need to distinguish two things regarding datasets:

- The datasets hosted at HF
- The Python packaged `datasets` from HF, used to access and process those datasets

We should explore the hosted datasets, filtered by most downloaded for each task/use-case.
We can:

- use the viewer to explare tha dataset,
- see the Python code snippet to get the dataset in the `Use this dataset` button of a dataset card, e.g.:

  ```python
  from datasets import load_dataset

  # Usually, we pass as argument the repo name
  ds = load_dataset("ylecun/mnist")
  ```

  ![MNIST Dataset](./assets/mnist.png)

### Compute Services with GPUs

Free-tier services:

- Google Colab: [https://colab.research.google.com/](https://colab.research.google.com/); for a quick guide, see [`Google_Colab_Notes.md`](https://github.com/mxagar/computer_vision_udacity/blob/main/02_Cloud_Computing/Google_Colab_Notes.md).
- Amazon SageMaker Studio Lab: [https://studiolab.sagemaker.aws/](https://studiolab.sagemaker.aws/); for a quick guide, see [`AWS_SageMaker_StudioLab_Notes.md`](https://github.com/mxagar/computer_vision_udacity/blob/main/02_Cloud_Computing/AWS_SageMaker_StudioLab_Notes.md).

If we are using remote hosted notebook (e.g., Google Colab or AWS SageMaker Studio Lab), we can use `notebook_login()` to log in to Hugging Face:

```python
from huggingface_hub import notebook_login
notebook_login()
# A box is shown where we can paste our HF token
```

If we want to access to a notebook in a 

### Cache Directories

Anything downloaded from HF is stored by default here:

```bash
# Unix
~/.cache/huggingface/ 
# Windows
C:\Users\<YourUsername>\.cache\huggingface\ 
```

But we can modify the directory by custommizing the option `cache_dir`.

To get information of the cache direcory:

```python
from huggingface_hub import scan_cache_dir

hf_cache_info = scan_cache_dir()
print(hf_cache_info)
```

## 2. NLP with Transformers

Folder: [`01-Transformers/`](./01-Transformers/).  
Notebooks:
- [`00-HF-Basics.ipynb`](./01-Transformers/00-HF-Basics.ipynb)
- [`01-Pipelines-for-NLP-Tasks.ipynb`](./01-Transformers/01-Pipelines-for-NLP-Tasks.ipynb)
- [`02-LLMs.ipynb`](./01-Transformers/02-LLMs.ipynb)

### Pipelines

The `transformers` library is used for NLP tasks. Usually, we input natural language text and thanks to pipelines it is used for several tasks:

- Sentiment analysis
- Text classification
- etc.

The module `pipeline` is the basic usage interface for the `transformers` library.
We can go to to Hugging Face, select models for a given task (e.g., text classification) and specify them to `pipeline`. Then, the input text will be automatically processed generating the expected task output.

We get the code snippet required to use a model when we click on `Use this model` on the web UI; example for text classification usng `distillbert`:

![Models Usage](./assets/models_usage.png)

```python
# Use a pipeline as a high-level helper
from transformers import pipeline

pipe = pipeline("text-classification", model="distilbert/distilbert-base-uncased-finetuned-sst-2-english")

# Load model directly
from transformers import AutoTokenizer, AutoModelForSequenceClassification

tokenizer = AutoTokenizer.from_pretrained("distilbert/distilbert-base-uncased-finetuned-sst-2-english")
model = AutoModelForSequenceClassification.from_pretrained("distilbert/distilbert-base-uncased-finetuned-sst-2-english")
```

There are many `pipelines`, and the list is continuously being extended; in fact, that's one of the main reasons why there are so many updates in the `transformers` library. The following page contains a list of all pipelines as well as their details:

[Hugging Face Pipelines](https://huggingface.co/docs/transformers/en/main_classes/pipelines).

Additionally, if we run `help(pipeline)` we ge a list of all tasks or pipelines:

- `"audio-classification"`: will return a [`AudioClassificationPipeline`].
- `"automatic-speech-recognition"`: will return a [`AutomaticSpeechRecognitionPipeline`].
- `"conversational"`: will return a [`ConversationalPipeline`].
- `"depth-estimation"`: will return a [`DepthEstimationPipeline`].
- `"document-question-answering"`: will return a [`DocumentQuestionAnsweringPipeline`].
- `"feature-extraction"`: will return a [`FeatureExtractionPipeline`].
- `"fill-mask"`: will return a [`FillMaskPipeline`]:.
- `"image-classification"`: will return a [`ImageClassificationPipeline`].
- `"image-feature-extraction"`: will return an [`ImageFeatureExtractionPipeline`].
- `"image-segmentation"`: will return a [`ImageSegmentationPipeline`].
- `"image-to-image"`: will return a [`ImageToImagePipeline`].
- `"image-to-text"`: will return a [`ImageToTextPipeline`].
- `"mask-generation"`: will return a [`MaskGenerationPipeline`].
- `"object-detection"`: will return a [`ObjectDetectionPipeline`].
- `"question-answering"`: will return a [`QuestionAnsweringPipeline`].
- `"summarization"`: will return a [`SummarizationPipeline`].
- `"table-question-answering"`: will return a [`TableQuestionAnsweringPipeline`].
- `"text2text-generation"`: will return a [`Text2TextGenerationPipeline`].
- `"text-classification"` (alias `"sentiment-analysis"` available): will return a
  [`TextClassificationPipeline`].
- `"text-generation"`: will return a [`TextGenerationPipeline`]:.
- `"text-to-audio"` (alias `"text-to-speech"` available): will return a [`TextToAudioPipeline`]:.
- `"token-classification"` (alias `"ner"` available): will return a [`TokenClassificationPipeline`].
- `"translation"`: will return a [`TranslationPipeline`].
- `"translation_xx_to_yy"`: will return a [`TranslationPipeline`].
- `"video-classification"`: will return a [`VideoClassificationPipeline`].
- `"visual-question-answering"`: will return a [`VisualQuestionAnsweringPipeline`].
- `"zero-shot-classification"`: will return a [`ZeroShotClassificationPipeline`].
- `"zero-shot-image-classification"`: will return a [`ZeroShotImageClassificationPipeline`].
- `"zero-shot-audio-classification"`: will return a [`ZeroShotAudioClassificationPipeline`].
- `"zero-shot-object-detection"`: will return a [`ZeroShotObjectDetectionPipeline`].

Code summary of the following notebooks:

- [`00-HF-Basics.ipynb`](./01-Transformers/00-HF-Basics.ipynb)
- [`01-Pipelines-for-NLP-Tasks.ipynb`](./01-Transformers/01-Pipelines-for-NLP-Tasks.ipynb)


```python
from datasets import load_dataset
import transformers
from transformers import pipeline

# Get a list of all pipelines: task
help(pipeline)

# We can find and view the datasets in HF:
# https://huggingface.co/datasets/cornell-movie-review-data/rotten_tomatoes/viewer
# We can specify the cache_dir, otherwise it defaults to ~/.cache/huggingface
reviews = load_dataset('rotten_tomatoes',cache_dir='rotten_tomatoes_data')

# datasets.dataset_dict.DatasetDict
# It is a dictionary which contains the splits,
# which are of type Dataset
# reviews['train'], reviews['test'], reviews['validation'] -> Dataset
type(reviews)

# Export a Dataset split to pandas
# Then, we can grab the columns 'text' and 'label' from the pandas df
reviews['train'].to_pandas()

# If no model is passed, the default model for the task is used
# sentiment-analysis = text-classification
classifier = pipeline('sentiment-analysis')

result = classifier("This movie was great!")
result
# [{'label': 'POSITIVE', 'score': 0.9998677968978882}]

def label(review):
    label = classifier(review)[0]['label']
    if label == 'POSITIVE':
        return 1
    else:
        return 0

label("This movie was so bad, I would have walked out if I wasn't on a plane! lol")
# 0

test_df = reviews['test'].to_pandas()
test_df['predicted_label'] = test_df['text'].apply(label)

# This would be our accuracy
sum(test_df['label']==test_df['predicted_label'])/1066

### -- Saving and Loading

# We can sace the model to file/folder:
# my_local_text_classification/
#   config.json
#   special_tokens_map.json
#   tokenizer.json
#   model.safetensors
#   tokenizer_config.json
#   vocab.txt
classifier.save_pretrained('my_local_text_classification/')

# Load the pipeline from the saved directory
classifier = pipeline(task="text-classification", model='my_local_text_classification/', tokenizer='my_local_text_classification/')

# Now we can use the pipeline for inference
result = classifier("I love this movie!")
print(result)
# [{'label': 'POSITIVE', 'score': 0.9971315860748291}]

### -- Specific Models

# Instead of using the default model, we can take specific ones
# Example: a model for financial news classification
# https://huggingface.co/ProsusAI/finbert
# Each model has also a default task associated
# Usually sentiment-analysis = text-classification
pipe = pipeline(model="ProsusAI/finbert")
tweets = ['Gonna buy AAPL, its about to surge up!',
          'Gotta sell AAPL, its gonna plummet!']
pipe(tweets)
#[{'label': 'positive', 'score': 0.5234110355377197},
# {'label': 'neutral', 'score': 0.5528594255447388}]

### -- NER: Name Entity Recognition

# Name Entity Recognition (WARNING: large model)
# We can tag parts of speech: 
ner_tag_pipe = pipeline('ner')

result = ner_tag_pipe("After working at Tesla I started to study Nikola Tesla a lot more, especially at university in the USA.")

# Tesla: Organization
# Nicola Tesla: Person
# USA: Location
result

### -- Question Answering

# Another task is QA: this is similar to a chatbot,
# but we pass a context + question and get one answer
# Default QA model:
# https://huggingface.co/distilbert/distilbert-base-cased-distilled-squad
qa_bot = pipeline('question-answering')

text = """
D-Day, marked on June 6, 1944, stands as one of the most significant military operations in history, 
initiating the Allied invasion of Nazi-occupied Europe during World War II. Known as Operation Overlord, 
this massive amphibious assault involved nearly 160,000 Allied troops landing on the beaches of Normandy, 
France, across five sectors: Utah, Omaha, Gold, Juno, and Sword. Supported by over 5,000 ships and 13,000 
aircraft, the operation was preceded by extensive aerial and naval bombardment and an airborne assault. 
The invasion set the stage for the liberation of Western Europe from Nazi control, despite the heavy 
casualties and formidable German defenses. This day not only demonstrated the logistical prowess 
and courage of the Allied forces but also marked a turning point in the war, leading to the eventual 
defeat of Nazi Germany.
"""

question = "What were the five beach sectors on D-Day?"

result = qa_bot(question=question,context=text)
# {'score': 0.9430820345878601,
#  'start': 345,
#  'end': 379,
#  'answer': 'Utah, Omaha, Gold, Juno, and Sword'}

### -- Transalation

# Language translation: we can get a generic translation pipeline 'translation'
# or a specific one 'translation_xx_to_yy'
# Default model:
# https://huggingface.co/google-t5/t5-base
translate = pipeline('translation_en_to_fr')
result = translate("Hello, my name is Mikel. What is your name?")
# [{'translation_text': 'Hola, me llamo Mikel. ¿Cómo te llamas?'}]

# For other language pairs, we need to specify other specific models
translate = pipeline('translation_en_to_es', model='Helsinki-NLP/opus-mt-en-es')
result = translate("Hello, my name is Mikel. What is your name?")
# [{'translation_text': 'Hola, me llamo Mikel. ¿Cómo te llamas?'}]

# We can also load the generic translation pipeline
# and specify the source and target languages when translating
translator = pipeline('translation', model='Helsinki-NLP/opus-mt-en-es')
result = translator("Hello, how are you?", src_lang='en', tgt_lang='es')
# [{'translation_text': 'Hola, ¿cómo estás?'}]
```

### Large Language Models (LLMs)

LLMs are the underlying models that run an NLP `pipeline`. A LLM has the following components or steps, from the users' perspective:

- **Tokenizer**: words are divided into tokens. Modern tokenization splits words into parts, aka *sub-word tokenization*. Different models use different tokenizers (GPT2 uses Byte-level BPE). Tokens are expressed as ids `[0,N]`, being `N+1` the vocabulary size. For instance, GPT2 has a vocabulary size of 50k tokens.
- **Encoder**: tokens are converted to embedding vectors.
- **Transformer model, decoder**: we pass the sequence of embedding vectors and the transformer outputs the most likely next vector.
- **Selection**: the most likely vector is converted to a token; the token is output as the next word. Note that we sample from a distribution of most likely tokens, we don't have to take always the most likely one! Thus, the output is not deterministic! We can control that with several variables:
  - **Temperature**: it affects the sampling
    - High temperature: `~ 1.0`: more creative results.
    - Low temperature: `~ 0.0`: less creative results.
  - **Tok K**: the size of the sampling as cut off
    - If `tok_k = 3` we only consider the 3 most likely tokens to perform the sampling.
  - **Top P**: cummulative probability as cut off
    - If `top_p = 0.97` the tokens that sum up to `0.97` are considered, teh rest ignored

### Tokenization, Probablities and Text Generation

Notebook: [`02-LLMs.ipynb`](./01-Transformers/02-LLMs.ipynb).

```python
import transformers
import torch

# The tokenizers are abstracted by AutoTokenizer
from transformers import AutoTokenizer
# Similarly as with the AutoTokenizer 
# we can use the AutoModelForCausalLM (AutoModelFor* + TAB)
# to load the generative LLM we want.
# Note that the model and the tokenized must match,
# which is achieved with the model string "gpt2" we pass.
# The class AutoModelForCausalLM is a generic model class that 
# will be instantiated as one of the model classes of the library 
# (with a causal language modeling head).
# Using the Auto* classes for GPT2
# we really load GPT2Tokenizer and GPT2LMHeadModel
from transformers import AutoModelForCausalLM

# Load the GPT-2 tokenizer: 
# we pass the model name and its associated toknizer is loaded
tokenizer = AutoTokenizer.from_pretrained("gpt2")

# return_tensors="pt": PyTorch tensors
# ids: we go from words to uids; then, we'll convert the uids into vectors
input_ids = tokenizer("Preposterous, I'm flabbergasted!", return_tensors="pt").input_ids
print(input_ids)
# Output: tensor([[1026,  373,  257, 3223,  290, 6388,   88]])

# Decode the tokens back into text
for t in input_ids[0]:
    print(t, "\t:", tokenizer.decode(t))
# tensor(37534) 	: Prep
# tensor(6197) 	: oster
# tensor(516) 	: ous
# tensor(11) 	: ,
# ...

# Generative LLM: GPT2
gpt2 = AutoModelForCausalLM.from_pretrained("gpt2")
# We can get all the config and interfaces of the model
help(gpt2)

outputs = gpt2(input_ids)
outputs.logits.shape
# torch.Size([1, 4, 50257])
# 1: number of batches
# 4: sequence length, i.e., number of tokens in "I skip across the"
# 50257: vocabulary sized
# logits: raw outputs from the model, we can convert them to ps
# IMPORTANT: the length of the output sequence is the same as the input length, but:
# - the first output tensor/token is the 2nd in the input sequence
# - the last output tensor/token is the NEW token!

final_logits = gpt2(input_ids).logits[0, -1] # The last set of logits
final_logits.argmax() # tensor(1627)
tokenizer.decode(final_logits.argmax()) # We decode the most probable NEW token
# 'line'

# We can also check the top 10 NEW tokens
top10_logits = torch.topk(final_logits, 10)
for index in top10_logits.indices:
    print(tokenizer.decode(index))
    # line
    # street
    # river
    # ...

top10 = torch.topk(final_logits.softmax(dim=0), 10)
# Here, we see the associated probabilities - very low
for value, index in zip(top10.values, top10.indices):
    print(f"{tokenizer.decode(index):<10} {value.item():.1%}")
    # line      3.6%
    # street    2.7%
    # river     2.2%
    # ...

# We have a generate() interface which wraps the LLM,
# abstracts away the details of making multiple forward passes (to get successive next words)
# and adds additional functonalities, such as:
# - max_new_tokens
# - repetition_penalty: decrease re-use of words
# - do_sample: sample, don't select likeliest
# - temperature: randomness of sampling
# - top_k: number of top k in sampling
# - top_p: cumulative p in sampling
# - bad_words_ids: avoid offensive words
# - num_beams: don't pick likeliest, but consider several branches (beam search)
# ...
# We can use different strategies for generation:
# - Greedy decoding: pick likeliest word; default
# - Beam search: keeps track of multiple hypotheses during generation, choosing the most likely overall sequence
output_ids = gpt2.generate(input_ids, max_new_tokens=20, repetition_penalty=1.5)
# Then, we decode the output ids to obtain tokens
decoded_text = tokenizer.decode(output_ids[0])

print("Input IDs:", input_ids[0])
print("Output IDs:", output_ids)
print(f"Generated text: {decoded_text}") # # We have a generate() interface which wraps the LLM,
# abstracts away the details of making multiple forward passes (to get successive next words)
# and adds additional functonalities, such as:
# - max_new_tokens
# - repetition_penalty: decrease re-use of words
# - do_sample: sample, don't select likeliest
# - temperature: randomness of sampling
# - top_k: number of top k in sampling
# - top_p: cumulative p in sampling
# - bad_words_ids: avoid offensive words
# - num_beams: don't pick likeliest, but consider several branches (beam search)
# ...
# We can use different strategies for generation:
# - Greedy decoding: pick likeliest word; default
# - Beam search: keeps track of multiple hypotheses during generation, choosing the most likely overall sequence
output_ids = gpt2.generate(input_ids, max_new_tokens=20, repetition_penalty=1.5)
# Then, we decode the output ids to obtain tokens
decoded_text = tokenizer.decode(output_ids[0])

print("Input IDs:", input_ids[0])
print("Output IDs:", output_ids)
print(f"Generated text: {decoded_text}")
# I skip across the line to a place where I can see it.
# 
# The next time you're in town,

from transformers import set_seed
set_seed(70)
sampling_output = gpt2.generate(
    input_ids,
    num_beams=5,
    do_sample=True,
    temperature=0.4,
    repetition_penalty=1.2,
    max_length=40,
    top_k=10,
)
print(tokenizer.decode(sampling_output[0], skip_special_tokens=True))
```

## 3. Image Models: Diffusers

Folder: [`02-Diffusers/`](./02-Diffusers/).  
Notebooks:
- [`00-Understanding-Image-Data.ipynb`](./02-Diffusers/00-Understanding-Image-Data.ipynb)
- [`01-Understanding-Diffusion-Models.ipynb`](./02-Diffusers/01-Understanding-Diffusion-Models.ipynb)
- [`02-Diffusers/02-AutoPipelines-Diffusers.ipynb`](./02-Diffusers/02-AutoPipelines-Diffusers.ipynb)

Images are regresented as hypermatrices of pixels (`W x H x C`), where each pixel contains a value in `[0,255]` and `C` often `{R,G,B}`.

![Image Representation](./assets/image_representation.png)

### Basic Image Processing

Notebook: [`00-Understanding-Image-Data.ipynb`](./02-Diffusers/00-Understanding-Image-Data.ipynb).

The `diffusers` library uses the default image processing library Pillow from Python. Another handy library is OpenCV.

```bash
pip install pillow
pip install opencv-python
# Often, cloud platforms require the headless library
pip install opencv-python-headless
```

Some basic commands for managing images:

```python
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2

### -- PIL

# Load the image
pic = Image.open('puppy.png')
# Check the type of the image
type(pic) # PIL.PngImagePlugin.PngImageFile

# Convert image to NumPy array
pic_arr = np.asarray(pic)
pic_arr.shape # (1024, 1024, 4)
plt.imshow(pic_arr) # Display

# Copy the array
pic_red = pic_arr.copy()

# Zero out the green and blue channels
pic_red[:, :, 1] = 0  # Green
pic_red[:, :, 2] = 0  # Blue
# Display the modified image: Red
plt.imshow(pic_red)

### -- OpenCV

# Load the image
img = cv2.imread('puppy.png')
# Convert BGR to RGB: OpenCV opens images as BGR
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# Display
plt.imshow(img_rgb)

# Load image in grayscale
img_gray = cv2.imread('puppy.png', cv2.IMREAD_GRAYSCALE)
plt.imshow(img_gray, cmap='gray')

# Resize image
img_resized = cv2.resize(img_rgb, (1300, 275))
plt.imshow(img_resized)

# Resize by ratio
new_img = cv2.resize(img_rgb, (0, 0), fx=0.5, fy=0.5)
plt.imshow(new_img)
```

See also:

- [mxagar/computer_vision_udacity](https://github.com/mxagar/computer_vision_udacity)
- [`CVND_Introduction.md`](https://github.com/mxagar/computer_vision_udacity/blob/main/01_Intro_Computer_Vision/CVND_Introduction.md)
- [`Catalog_CV_Functions.md`](https://github.com/mxagar/computer_vision_udacity/blob/main/Catalog_CV_Functions.md)

### Image Generation: Text-to-Image and Diffusion Models

These are some important papers/works:

| Attribute             | CLIP                                                                 | DALL-E                                                               | Stable Diffusion                                                     |
|-----------------------|----------------------------------------------------------------------|----------------------------------------------------------------------|----------------------------------------------------------------------|
| Task                  | Encoder model that creates embedding vectors for both images and text in the same embedding space | Generates images from textual descriptions; it uses CLIP in its pipeline: the model generates images that match the semantic meaning of the input text                           | Generates images through a process of iterative refinement starting from random noise; the model is equivalent to DALLE, because it generates images from text, so a similar pipeline is used behing the scenes |
| Year                  | 2021                                                                 | 2021                                                                 | 2022                                                                 |
| Paper                 | [https://arxiv.org/abs/2103.00020](https://arxiv.org/abs/2103.00020) | [https://arxiv.org/abs/2102.12092](https://arxiv.org/abs/2102.12092) | [https://arxiv.org/abs/2205.11487](https://arxiv.org/abs/2205.11487) |
| Organization + Author | OpenAI, Alec Radford                                                 | OpenAI, Aditya Ramesh                                                 | CompVis, Robin Rombach                                               |
| Type of model         | Transformer                                                          | Transformer                                                          | Diffusion                                                            |
| Num. parameters       | 400 million                                                          | 12 billion                                                            | Unknown                                                              |
| Weights + License     | Yes, MIT License                                                     | No                                                                   | Yes, CreativeML Open RAIL-M                                          |
| API available         | Yes                                                                  | Yes                                                                  | Yes                                                                  |

However, note that new versions are constantly appearing; e.g., Stable Diffusion 3 appeared in 2024, anf it is a Diffusion Transformer instead of a UNet, as previous versions.

Image-to-Text generation models work in different stages:

- Texts are embedded using an encoder model simila to CLIP, which produces and embedding space valid for both images and text.
- Text embeddings are processed by a prior model which maps them to an image-only space; this works better than directly taking the CLIP embedding. That mapping occurs with some linear, normalization and attention layers.
- Then, diffusion models are applied, as decoders, which expand the image embedding. Diffusion models reverse gradually hypothetical noise in an image.

![Text-to-Image Process](./assets/text_to_image.png)

Diffusion models: 

- Diffusion models take an image embedding and a random noise vector and generate an image out of them.
- The Diffusion model is trained to undo the steps of a fixed corruption process (adding Gaussian noise).
- After the final step, the image is clear and visible.
- The model starts with random noise and uses the image embedding to guide the denoising process, ensuring the generated image matches the desired features and structure represented by the embedding.
- Noise removal (denoising) is applied iteratively during the image generation process, i.e., as we expand the image.
- The image embedding remains the same throughout the entire denoising process.
- Unlike previous techniques like Variational Autoencoders (VAEs) or Generative Adversarial Networks (GANs), which generate images in a single pass, diffusion models create images through many steps. This iterative process allows the model to refine and correct its output gradually, leading to high-quality image generation.
- Initial models used `512 x 512` resolution, but current models use `1024 x 1024`.

![Denoising](./assets/denoising.gif)

### Diffusion Models with Python

Notebook: [`01-Understanding-Diffusion-Models.ipynb`](./02-Diffusers/01-Understanding-Diffusion-Models.ipynb). The following contents are shown:

- Difussion Pipeline: generate random celebrity images
- Difussion Model & Scheduler: Manually generating random celebrity images

We need a GPU for the notebook; I used a T4 in Google Colab.

```python
from PIL import Image
import numpy as np
import torch
from tqdm.auto import tqdm

### -- Difussion Pipeline: generate random celebrity images
from diffusers import DDPMPipeline

# DDPMPipeline = Denoising Diffusion Probabilistic Models
# https://huggingface.co/docs/diffusers/v0.3.0/en/api/pipelines/ddpm
# This is the base class which can contain diffusion models
# and we can load them simply passing the model string to from_pretrained()
# Usually, they have two components:
# - model, unet: the UNet model which performs the denoising
# - scheduler: which runs the unet in several steps
# Here, both components are used behing the hood
# but we can use them separately, as shown below.
# However, usually we don't use them separately.
ddpm = DDPMPipeline.from_pretrained("google/ddpm-celebahq-256").to("cuda")

# We run the model using the schedulerin 30 steps
image = ddpm(num_inference_steps=30).images[0]
# Display
image

### -- Difussion Model & Scheduler: Manually generating random celebrity images
from diffusers import DDPMScheduler, UNet2DModel

# Instead of using DDPMPipeline,
# we now will use th emodel and the scheduler separately
scheduler = DDPMScheduler.from_pretrained("google/ddpm-celebahq-256")
model = UNet2DModel.from_pretrained("google/ddpm-celebahq-256").to("cuda")
 
# Create 50 evenly spaced timesteps
scheduler.set_timesteps(50)


# Get the sample size from the model configuration
# This is the image size
sample_size = model.config.sample_size

# Create random noise with the same shape as the desired output
noise = torch.randn((1, 3, sample_size, sample_size), device="cuda")

# Denoising loop, written manually
# Initialize the input to be the random noise we created
input = noise

# Loop over each timestep
for t in scheduler.timesteps:
    with torch.no_grad():  # No gradient calculation is needed
        # Get the noisy residual from the model
        # The Diffusion model predicts the noise overlaping the image
        # UNet2DModel.forward() is run here
        noisy_residual = model(input, t).sample
    # Predict the image at the previous timestep
    # The scheuler removed the predicted noise
    previous_noisy_sample = scheduler.step(noisy_residual, t, input).prev_sample
    # Update the input for the next iteration
    input = previous_noisy_sample

# Convert the denoised output to an image
# Normalize the image data: values in [0,1], not [-1,1]
image = (input / 2 + 0.5).clamp(0, 1).squeeze()
# Change the shape and type for image conversion
image = (image.permute(1, 2, 0) * 255).round().to(torch.uint8).cpu().numpy()
# Create a PIL image
image = Image.fromarray(image)
# Display
image

### -- Text-to-Image: Prompt a caption and obtain an image, all steps manual
from transformers import CLIPTextModel, CLIPTokenizer
from diffusers import AutoencoderKL, UNet2DConditionModel, PNDMScheduler
from diffusers import UniPCMultistepScheduler

# Stable Diffusion v1.4 is used here
# It consists of several models:
# - A text encoder
# - A UNet for denoising
# - A VAE for expanding the denoised image
vae = AutoencoderKL.from_pretrained("CompVis/stable-diffusion-v1-4", subfolder="vae", use_safetensors=True)
tokenizer = CLIPTokenizer.from_pretrained("CompVis/stable-diffusion-v1-4", subfolder="tokenizer")
text_encoder = CLIPTextModel.from_pretrained("CompVis/stable-diffusion-v1-4", subfolder="text_encoder", use_safetensors=True)
unet = UNet2DConditionModel.from_pretrained("CompVis/stable-diffusion-v1-4", subfolder="unet", use_safetensors=True)

# Instead of the default PNDMScheduler, let's use the UniPCMultistepScheduler
scheduler = UniPCMultistepScheduler.from_pretrained("CompVis/stable-diffusion-v1-4", subfolder="scheduler")

# Move Models to GPU
torch_device = "cuda"
vae.to(torch_device)
text_encoder.to(torch_device)
unet.to(torch_device)
 
# Parameters: prompt, sizes, etc. 
prompt = ["a photo of a red planet"]
height = 512  # default height of Stable Diffusion
width = 512  # default width of Stable Diffusion
num_inference_steps = 25  # Number of denoising steps
guidance_scale = 7.5  # Scale for classifier-free guidance

# Seed generator to create the initial latent noise
generator = torch.manual_seed(0)

batch_size = len(prompt)

text_input = tokenizer(
    prompt,
    padding="max_length",
    max_length=tokenizer.model_max_length,
    truncation=True,
    return_tensors="pt"
)

with torch.no_grad():
    text_embeddings = text_encoder(text_input.input_ids.to(torch_device))[0]

# Tokenization of Unconditional Guidance
# Tokenizing empty prompts for unconditional guidance
# is a technique used in diffusion models,
# particularly in text-to-image generation models like Stable Diffusion.
# It involves providing the model with a set of inputs 
# that do not contain any specific information or prompts.
# This helps the model learn the underlying structure or distribution
# of the data without being influenced by specific input text.
max_length = text_input.input_ids.shape[-1]
uncond_input = tokenizer(
    [""] * batch_size,
    padding="max_length",
    max_length=max_length,
    return_tensors="pt"
)
uncond_embeddings = text_encoder(uncond_input.input_ids.to(torch_device))[0]

text_embeddings = torch.cat([uncond_embeddings, text_embeddings])

# Generate some initial random noise
latents = torch.randn((batch_size, unet.config.in_channels, height // 8, width // 8), device=torch_device)
latents = latents * scheduler.init_noise_sigma

scheduler.set_timesteps(num_inference_steps)
# Create the denoising loop to progressively transform 
# the pure noise in latents to an image described by your prompt
for t in tqdm(scheduler.timesteps):
    latent_model_input = torch.cat([latents] * 2)
    latent_model_input = scheduler.scale_model_input(latent_model_input, timestep=t)

    with torch.no_grad():
        noise_pred = unet(latent_model_input, t, encoder_hidden_states=text_embeddings).sample

    noise_pred_uncond, noise_pred_text = noise_pred.chunk(2)
    noise_pred = noise_pred_uncond + guidance_scale * (noise_pred_text - noise_pred_uncond)

    latents = scheduler.step(noise_pred, t, latents).prev_sample

# Use the VAE to decode the latent representation into an image
latents = 1/ 0.18215 * latents
with torch.no_grad():
    image = vae.decode(latents).sample

image = (image / 2 + 0.5).clamp(0, 1).squeeze()
image = (image.permute(1, 2, 0) * 255).to(torch.uint8).cpu().numpy()
image = Image.fromarray(image)
# Display
image

```

![Celibrity](./assets/celebrity.png)

![Red Planet](./assets/red_planet.png)

### Auto-Pipelines

Notebook: [`02-AutoPipelines-Diffusers.ipynb`](./02-Diffusers/02-AutoPipelines-Diffusers.ipynb).  

```python
# We can browse HuggingFace looking for the right model
# For instance:
# https://huggingface.co/dreamlike-art/dreamlike-photoreal-2.0
# The option "use this model" shows how to load that model and use it
# but we can also try to load it with the generic AutoPipelineFor + TAB
from diffusers import AutoPipelineForText2Image
import torch

# Load the pipeline
# With AutoPipelineFor* we pass the model string
pipe_txt2img = AutoPipelineForText2Image.from_pretrained(
    "dreamlike-art/dreamlike-photoreal-2.0",
    torch_dtype=torch.float16,
    use_safetensors=True
).to("cuda")

# Define the prompt and generator
prompt = "cinematic photo of Elmo as the president of the United States in front of the white house, 35mm photograph, film, professional, 4k, highly detailed"
generator = torch.Generator(device="cuda").manual_seed(37)

# Generate the image
image = pipe_txt2img(prompt, generator=generator).images[0]
# Display
image
```

![Elmo President](./assets/elmo_president.png)

## 4. Video Models

Folder: [`03-Video-Models/`](./03-Video-Models/).  
Notebooks:
- [`00-Stable-Video-Diffusion.ipynb`](./03-Video-Models/00-Stable-Video-Diffusion.ipynb)
- [`01-Image2VideoGen-XL.ipynb`](./03-Video-Models/01-Image2VideoGen-XL.ipynb)

We can create videos, for instance, from

- a prompt
- a single image
- or both a text and an image

**Stable Video Diffusion** is one of the most important works to date in the field:

- [Blog Post - Introducing Stable Video Diffusion](https://stability.ai/news/stable-video-diffusion-open-ai-video-model)
- [Paper - Stable Video Diffusion: Scaling Latent Video Diffusion Models to Large Datasets](https://arxiv.org/abs/2311.15127)

Stable Video Diffusion is/was trained as follows:

- First, we have the **text-to-image pretraining**: a text-to-image diffusion model is trained, which will be able to generate images from text prompts. Stable Diffusion 2.0 is used here, which is trained with high resolution images.
  - The Stable Video Diffusion models operates in the latent space, which reduces computational complexity.
- Then, we perform the **video pretraining**, i.e., the text-to-image model is adapted by inserting temporal layers that handle sequences of frames. A video is at the end of the day a sequence of coherent image frames. 
  - The Large Video Dataset (LVD) is used, which consists of 580 million annotated low resolution video clips.
  - The model learns video dynamics, motion, temporal coherence.
  - Temporal convolutions are used, as well as attention layers, which enable changing only relevant pixels at a time.
- Finally, we have the **high-quality video fine-tuning**: since the video pretraning is/was with low-quality videos, now we train it with a smaller set of high-quality videos. The final model is able to output high-resolution videos.
  - Human curation is important here; optical flow analysis and OCR are used, among others, to filter videos before curation (we want videos with movement and fewer text).

These models are very memory-intensive, so we need GPUs and they need to be configured properly.

### Stable Video Diffusion: Image-to-Video

Notebook: [`00-Stable-Video-Diffusion.ipynb`](./03-Video-Models/00-Stable-Video-Diffusion.ipynb). Stable Video Diffusion is used; we pass an image to it and obtain a video of around 4 seconds.

```python
import torch
import diffusers
import transformers
import cv2
from diffusers import StableVideoDiffusionPipeline
from diffusers.utils import load_image, export_to_video

# Model in complete Pipeline
# https://huggingface.co/stabilityai/stable-video-diffusion-img2vid-xt
# We feed an image and get a short video (4s, approx.)
pipe = StableVideoDiffusionPipeline.from_pretrained(
    'stabilityai/stable-video-diffusion-img2vid-xt',
    torch_dtype=torch.float16, # This reduces from 32 to 16B, less memory
    variant='fp16' # This goes with the previous line, it's a variante of the model
)
pipe.enable_model_cpu_offload()

# We load the image; this is the conditioning image
image = load_image("nasa-U2uKrI4lci8-unsplash.png")

# If the size is not 1024 x 576
# We would need to:
# 1) crop to the same ratio, manually, 
# 2) resize to (1024, 576): image.resize((1024,576))
image.size # (1024, 576)

# Reproducible seed
generator = torch.manual_seed(42)

# This takes around 5 minutes on a T4
# Sometimes we get Out of Memory errors, depending on the GPUs.
# One solution is to downgrade the model from 32 to 16B
# That's done in the Pipeline instantiation
# frames is a list of PIL.Images!
frames = pipe(
    image,
    decode_chunk_size=8,
    generator=generator
).frames[0] # The frames are in the first elemet of the pipeline.frames

# Frames are exported to a video
export_to_video(frames,'generated.mp4',fps=7)

type(frames[0]) # PIL.Image.Image

# Display frame
frames[1]
```

### I2VGenXL: Image-and-Text-to-Video

Notebook: [`01-Image2VideoGen-XL.ipynb`](./03-Video-Models/01-Image2VideoGen-XL.ipynb). This models is similar to the previous one, because we pass an image to it and obtain a short video, but we can guide the video witha text prompt.

Paper: [I2VGen-XL: High-Quality Image-to-Video Synthesis via Cascaded Diffusion Models (2023)](https://arxiv.org/abs/2311.04145).

```python
import torch
from diffusers import I2VGenXLPipeline
from diffusers.utils import load_image, export_to_gif, export_to_video  

# Empty GPU RAM before anything
torch.cuda.empty_cache()

# Load the model pipeline
# https://huggingface.co/docs/diffusers/en/api/pipelines/i2vgenxl
# https://arxiv.org/abs/2311.04145
repo_id = "ali-vilab/i2vgen-xl" 
pipeline = I2VGenXLPipeline.from_pretrained(repo_id, torch_dtype=torch.float16, variant="fp16")
pipeline.enable_model_cpu_offload()

image_url = "astro_moon.png"
image = load_image(image_url).convert("RGB")
prompt = "astronaut jumping straight up"

generator = torch.manual_seed(8888)
# This takes about 12 minutes in a T4
frames = pipeline(
    prompt=prompt,
    image=image,
    generator=generator
).frames[0] # The frames are in the first elemet of the pipeline.frames

len(frames) # Only 16 frames

# Convert list of PIL.Images to GIF
print(export_to_gif(frames))

# Convert list of PIL.Images to mp4
export_to_video(frames,'astro_jump.mp4',fps=8)
```

## 5. Audio Models

Folder: [`04-Audio-Models/`](./04-Audio-Models/).  
Notebooks:
- [`00-Audio-Data.ipynb`](./04-Audio-Models/00-Audio-Data.ipynb)
- [`01-Audio-Classification.ipynb`](./04-Audio-Models/01-Audio-Classification.ipynb)
- [`02-Audio-Transcription.ipynb`](./04-Audio-Models/02-Audio-Transcription.ipynb)
- [`03-Audio-Generation.ipynb`](./04-Audio-Models/03-Audio-Generation.ipynb)

### Understanding Audio Data

Sound waves are matter vibration which cause increased/decreased pressure in an area over time:

![Audio Waves](./assets/audio_waves.png)

We need to select a **sampling rate** to measure (`Hz = 1 /s`); in audio, this is done typically in `{16 kHz, 44.1 kHz, 192 kHz}`, depending on the quality `{normal (voice), high (music), super-high}`. The sampling rate is important because the input data needs to be sampled at the same frequency at which the model was trained in.

The pressure level caused by a wave is the **amplitude** of the wave, measure in `dB`; this is the perceived **loudness**. Some details about the amplitude and `dB`:

- Human hearing is logarithmic, as `dB`; thus the interpretation of the amplitude in `dB` makes sense.
- In the physical world: `0 dB` = quietest sound; larger values = larger sounds.
- For digital audio: `0 dB` = loudest possible amplitude!
  - every `-6 dB` is around a halving of amplitude,
  - below `-60 dB` generally inaudible.

A digital audio sample records the amplitude of the audio wave at a specific point in time. That amplitude can be recorded with different precission, determined by the **bit depth**, similarly as in the images (i.e., pixels usually have 3 channels of 8 bits = 256 levels). Higher bit depths lead to more accurate sound representations.

- 16-bit audio: 65,536 possible amplitude values
- 24-bit audio: 16 million.
- Higher bit-depths reduce quantization noise.

Waves have **frequency** components, i.e., they can be decomposed to simple sinusoidal base waves each with an amplitude and an oscillation frequency.

- Low frequencies are associated to grave pitches.
- High frequencies are associated to high pitches.

Note that according to the Nyquist-Shannon equation, the sampling rate must be twice as much as the highest frequency we would like to measure.

We can visualize audio with

- Waveforms: Amplitude over Time (2D)
  ![Waveform](./assets/waveform.png)
- Frequency spectrums: Amplitude over Frequency (2D)
  ![Frequuency spectrum](./assets/frequency_spectrum.png)
- Spectrograms: Amplitude values for each Time and Frequency point (3D)
  ![Spectrogram](./assets/spectrogram.png)

Notebook: [`00-Audio-Data.ipynb`](./04-Audio-Models/00-Audio-Data.ipynb). Audio data is loaded and the three aforementioned plots are carried out using [librosa](https://librosa.org/doc/latest/index.html).

```python

```


## 6. Gradio for User Interfaces

Folder: []().  
Notebooks:
- A
- B

