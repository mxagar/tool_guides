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
    - [Tokenization and Probablities](#tokenization-and-probablities)
    - [Text Generation](#text-generation)
  - [3. Image Models: Diffusers](#3-image-models-diffusers)
  - [4. Video Models](#4-video-models)
  - [5. Audio Models](#5-audio-models)
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
```

However, most of the examples in this guide were carried out in online services with GPUs, such as

- Google Colab
- AWS SageMaker Studio Lab

Note: since the libraries are benig updated so frequently, we often need to upgrade or even downgrade (because compatility is broken) the library versions. AWS SageMaker Studio Lab allows setting our own environments, but Google Colab requires us to do that manually.

```bash
!pip uninstall transformers -y
pip install transformers==4.41.0
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

### Tokenization and Probablities

### Text Generation

## 3. Image Models: Diffusers

Folder: []().  
Notebooks:
- A
- B

## 4. Video Models

Folder: []().  
Notebooks:
- A
- B

## 5. Audio Models

Folder: []().  
Notebooks:
- A
- B

## 6. Gradio for User Interfaces

Folder: []().  
Notebooks:
- A
- B

