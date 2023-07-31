# Hugging Face: A Guide

This mini-project compiles information on the usage of Hugging Face, as well as examples and material from the listed tutorials.

Source material:

- [Getting Started With Hugging Face in 15 Minutes | Transformers, Pipeline, Tokenizer, Models](https://www.youtube.com/watch?v=QEaBAZQCtwE)
- [HuggingFace Crash Course - Sentiment Analysis, Model Hub, Fine Tuning](https://www.youtube.com/watch?v=GSt00_-0ncQ)
- [An Introduction to Using Transformers and Hugging Face](https://www.datacamp.com/tutorial/an-introduction-to-using-transformers-and-hugging-face)
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/index)
- [Hugging Face NLP Course](https://huggingface.co/learn/nlp-course/chapter1/1)
- [Fine-tune a pretrained model](https://huggingface.co/docs/transformers/training)
- [Fine-tuning with custom datasets](https://huggingface.co/transformers/v3.2.0/custom_datasets.html)

Table of contents:

- [Hugging Face: A Guide](#hugging-face-a-guide)
  - [1. Introduction and Quick Start](#1-introduction-and-quick-start)
    - [Setup](#setup)
    - [Pipeline](#pipeline)
    - [Tokenizer and Models](#tokenizer-and-models)
    - [Combining the Code with PyTorch and Tensorflow](#combining-the-code-with-pytorch-and-tensorflow)
    - [Save and Load](#save-and-load)
    - [Model Hub](#model-hub)
    - [Finetuning Pre-Trained Models with Our Datasets](#finetuning-pre-trained-models-with-our-datasets)


## 1. Introduction and Quick Start

See the notebook [`hugging_face_intro.ipynb`](hugging_face_intro.ipynb).

### Setup

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

# Install the transformers library
pip install transformers datasets accelerate evaluate

# For CPU support only:
pip install 'transformers[torch]' datasets accelerate evaluate

# If you want to install the HuggingFace Hub conection
# This is only necessary if we need programmatic access,
# for which there is also a CLI tool
pip install huggingface_hub
```

### Pipeline

The [pipeline](https://huggingface.co/docs/transformers/main/main_classes/pipelines) does 3 things:

- Preprocessing the text: tokenization
- Feed the preprocessed text to the model
- Postprocessing, e.g., labels applied and output packed

To the pipeline, we need to pass the **task** we want to carry out, and optionally the **model** (plus the **revision**) we would like to use. Then, the pipeline returns the complete model pipeline which performs the 3 steps above. There are many [tasks](https://huggingface.co/docs/transformers/main/main_classes/pipelines#transformers.pipeline.task):

- `sentiment-analysis`
- `text-generation`
- `question-answering`
- `translation`
- `zero-shot-classification`
- `audio-classification`
- `image-to-text`
- `object-detection`
- `image-segmentation`
- `summarization`
- ...

Notes:

- When a model is used for the first time, it needs to be downloaded.
- Always use the `model` and `revision` for reproducibility!

```python
from transformers import pipeline

# %%
# Vanilla sentiment analysis with a pipeline object:
# https://huggingface.co/docs/transformers/main/main_classes/pipelines
# We pass the task (see link above for a complete list of tasks)
classifier = pipeline(task="sentiment-analysis") # Task; no model selected - default used
res = classifier("I've been waiting for a HuggingFace course my whole life.")
print(res)
# [{'label': 'POSITIVE', 'score': 0.9598049521446228}]

# %%
# Now, we pass the model explicitly
classifier = pipeline(task="sentiment-analysis",
                      model="distilbert-base-uncased-finetuned-sst-2-english")
res = classifier("I've been waiting for a HuggingFace course my whole life.")
print(res)
# [{'label': 'POSITIVE', 'score': 0.9598049521446228}]

# %%
# Vanilla text generation
generator = pipeline(task="text-generation",
                     model="distilgpt2")
res = generator("In this course, I will teach you how to",
                max_length=30,
                num_return_sequences=2)
print(res)
# [{'generated_text': 'In this course, I will teach you how to solve the problems in various languages.\n\n\nPlease do not miss this course:\nIf you'}, {'generated_text': 'In this course, I will teach you how to make a good job for your children:\n\n\n\n1. Make a decision about the future'}]

# %%
# Vanilla zero-shot-classification
classifier = pipeline(task="zero-shot-classification",
                      model="facebook/bart-large-mnli")
res = classifier("This course is about Python list comprehensions",
                 candidate_labels=["education", "politics", "business"])
print(res)
# {'sequence': 'This course is about Python list comprehensions', 'labels': ['education', 'business', 'politics'], 'scores': [0.8270175457000732, 0.12526951730251312, 0.04771287366747856]}
```
### Tokenizer and Models

The **tokenizer** is the first step in the **pipeline**. We can access and use it, and we can even pass our own tokenizer to the pipeline. The same happens with the **models**.

- [Hugging Face: Tokenizer](https://huggingface.co/docs/transformers/main/main_classes/tokenizer)
- [Hugging Face: Models](https://huggingface.co/docs/transformers/main/main_classes/model)

```python
from transformers import pipeline
# Generic classes for tokenization & sequence classification
from transformers import AutoTokenizer, AutoModelForSequenceClassification
# Specific classes for tokenization & sequence classificaion
from transformers import BertTokenizer, BertForSequenceClassification, BertModel

# %%
# Now, we can create instances of the tokenizer/model
# It is important to take the objects using from_pretrained()
model_name = "distilbert-base-uncased-finetuned-sst-2-english"
model = AutoModelForSequenceClassification.from_pretrained(model_name)
tokenizer = AutoTokenizer.from_pretrained(model_name)

classifier = pipeline(task="sentiment-analysis",
                      model=model,
                      tokenizer=tokenizer)

res = classifier("I've been waiting for a HuggingFace course my whole life.")
print(res)
# [{'label': 'POSITIVE', 'score': 0.9598049521446228}]

# %%
# We can also use the tokenizer separately
sequence = "Using transformers is simple with HuggingFace."

# We get the token ids and the attention_mask: 0 ignored by attention layer
res = tokenizer(sequence)
print(res)
# {'input_ids': [101, 2478, 19081, 2003, 3722, 2007, 17662, 12172, 1012, 102], 'attention_mask': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}

# We get the token strings
tokens = tokenizer.tokenize(sequence)
print(tokens)
# ['using', 'transformers', 'is', 'simple', 'with', 'hugging', '##face', '.']

# We get the ids of the token strings
ids = tokenizer.convert_tokens_to_ids(tokens)
print(ids)
# [2478, 19081, 2003, 3722, 2007, 17662, 12172, 1012]

# We decode the ids back to words
decoded_string = tokenizer.decode(ids)
print(decoded_string)
# using transformers is simple with huggingface.
```

### Combining the Code with PyTorch and Tensorflow

This section shows how to combine the HuggingFace `transformers` library with PyTorch and Tensorflow. The following examples deal with PyTorch, but the code for Tensorflow is very similar, often just adding the `TF` prefix to the classes.

In this example, instead of using the `pipeline`, we just use the `tokenizer` and the `model` as if the would be PyTorch objects.

```python
from transformers import pipeline
from transformers import AutoTokenizer, AutoModelForSequenceClassification
import torch
import torch.nn.functional as F

# %%
# We create instances of the tokenizer/model
model_name = "distilbert-base-uncased-finetuned-sst-2-english"
model = AutoModelForSequenceClassification.from_pretrained(model_name)
tokenizer = AutoTokenizer.from_pretrained(model_name)

# Our input
X = ["I've been waiting for a HuggingFace course my whole life.",
     "Python is great!",
     "But I don't like at all the fact that there is no notebook available."]

batch = tokenizer(X,
                  padding=True,
                  truncation=True,
                  max_length=512,
                  return_tensors="pt") # PyTorch tensor format
print(batch) # {'input_dis': ..., 'attention_mask': ...}

# Feed the model and process the output
with torch.no_grad():
    outputs = model(**batch) # unpack the dictionary to a list
    print(outputs)
    predictions = F.softmax(outputs.logits, dim=1)
    print(predictions)
    labels = torch.argmax(predictions, dim=1)
    print(labels) # [1, 1, 0]: positive, positive, negative
```

### Save and Load

It is possible to load HuggingFace objects (tokenizer, model, etc.), to fine-tune them (see section below) and save them. Then, we can load them when needed and use them.

```python
from transformers import pipeline
from transformers import AutoTokenizer, AutoModelForSequenceClassification

# %%
model_name = "distilbert-base-uncased-finetuned-sst-2-english"
model = AutoModelForSequenceClassification.from_pretrained(model_name)
tokenizer = AutoTokenizer.from_pretrained(model_name)
# Now, we can use these objects
# and even fine-tune the  model!
# After that, we would save the objects

# %%
# Save
save_directory = "./output/"
tokenizer.save_pretrained(save_directory)
model.save_pretrained(save_directory)
# In the save_directory, several files are created
# - related to the tokenizer: vocab.txt, tokenizer.json, ...
# - model: pytorch_model.bin
# - general: config.json

# Load
tok = AutoTokenizer.from_pretrained(save_directory)
mod = AutoModelForSequenceClassification.from_pretrained(save_directory)
```
### Model Hub

There are more that 200k available [HuggingFace Models](https://huggingface.co/models), built by the community, FAANG companies, etc. Notes to take into account:

- We can filter the models by
  - Task: Image Classification, Text Classification, etc.
  - Libraries: PyTorch, Tensorflow, JAX, Keras, etc.
  - Datasets
  - Languages: English, German, Spanisch, etc.
  - Licenses
  - Other

Some of them have code examples; in any case, we just need to click on a desired one and copy the model name to use it in the `pipeline` (use the icon/button).

Some example models:

- [Text clasification (default): distilbert-base-uncased-finetuned-sst-2-english](https://huggingface.co/distilbert-base-uncased-finetuned-sst-2-english)
- [Summarization: facebook/bart-large-cnn](https://huggingface.co/facebook/bart-large-cnn)

As for PyTorch, the pre-trained models are saved in `~/.cache/`, i.e. in `~/.cache/huggingface`. We can change that as follows:

- Setting the environment variable `HF_HOME`.
- Specifying it: `pipeline(cache_dir="/path/to/custom/cache/dir")`.

```python
from transformers import pipeline

# Example of text summarization
summarizer = pipeline("summarization", model="facebook/bart-large-cnn")

ARTICLE = """ New York (CNN)When Liana Barrientos was 23 years old, she got married in Westchester County, New York.
A year later, she got married again in Westchester County, but to a different man and without divorcing her first husband.
Only 18 days after that marriage, she got hitched yet again. Then, Barrientos declared "I do" five more times, sometimes only within two weeks of each other.
In 2010, she married once more, this time in the Bronx. In an application for a marriage license, she stated it was her "first and only" marriage.
Barrientos, now 39, is facing two criminal counts of "offering a false instrument for filing in the first degree," referring to her false statements on the
2010 marriage license application, according to court documents.
Prosecutors said the marriages were part of an immigration scam.
On Friday, she pleaded not guilty at State Supreme Court in the Bronx, according to her attorney, Christopher Wright, who declined to comment further.
After leaving court, Barrientos was arrested and charged with theft of service and criminal trespass for allegedly sneaking into the New York subway through an emergency exit, said Detective
Annette Markowski, a police spokeswoman. In total, Barrientos has been married 10 times, with nine of her marriages occurring between 1999 and 2002.
All occurred either in Westchester County, Long Island, New Jersey or the Bronx. She is believed to still be married to four men, and at one time, she was married to eight men at once, prosecutors say.
Prosecutors said the immigration scam involved some of her husbands, who filed for permanent residence status shortly after the marriages.
Any divorces happened only after such filings were approved. It was unclear whether any of the men will be prosecuted.
The case was referred to the Bronx District Attorney\'s Office by Immigration and Customs Enforcement and the Department of Homeland Security\'s
Investigation Division. Seven of the men are from so-called "red-flagged" countries, including Egypt, Turkey, Georgia, Pakistan and Mali.
Her eighth husband, Rashid Rajput, was deported in 2006 to his native Pakistan after an investigation by the Joint Terrorism Task Force.
If convicted, Barrientos faces up to four years in prison.  Her next court appearance is scheduled for May 18.
"""
print(summarizer(ARTICLE, max_length=130, min_length=30, do_sample=False))
# [{'summary_text': 'Liana Barrientos, 39, is charged with two counts of "offering a false instrument for filing in the first degree" In total, she has been married 10 times, with nine of her marriages occurring between 1999 and 2002. She is believed to still be married to four men.'}]
```

### Finetuning Pre-Trained Models with Our Datasets

Once we have prepared our dataset, fine-tuning a HuggingFace model is as simple as using the `Trainer` class.

Notes after the official tutorial [Fine-tune a pretrained model](https://huggingface.co/docs/transformers/main/training), in which a review rating text model is fine-tuned using the [Yelp Reviews dataset](https://huggingface.co/datasets/yelp_review_full).

Summary of steps for fine-tuning:
1. Prepare dataset
2. Load pre-trained Tokenizer, call it with dataset -> encoding
3. Build PyTorch Dataset with the encodings
4. Load pre-trained model
5. Training / Fine-tuning
    - Load Trainer and train it
    - Use native Pytorch training loop

```python
from datasets import load_dataset
from transformers import AutoTokenizer

# %%
# Load the Yelp Reviews Dataset
# https://huggingface.co/datasets/yelp_review_full
# The nice thing is that the dataset class from HuggingFace
# are stored to the chache folder, but only the requested instances
# are loaded to memory. Usually, you have dictionary-like
# samples.
dataset = load_dataset("yelp_review_full")
dataset["train"][100]

# %%
# Information on the features
dataset["train"].features

# %%
# We instantiate the tokenizer and pack it intoa function
# which is mapped to the items of the dataset
# We could apply several functions to the tokenized_datasets
# tokenized_datasets.remove_columns(...)
# tokenized_datasets.rename_columns(...)
# tokenized_datasets.with_format("torch")
tokenizer = AutoTokenizer.from_pretrained("bert-base-cased")

def tokenize_function(examples):
    return tokenizer(examples["text"], padding="max_length", truncation=True)

tokenized_datasets = dataset.map(tokenize_function, batched=True)

# %%
# We create a smaller sub-dataset in this example
small_train_dataset = tokenized_datasets["train"].shuffle(seed=42).select(range(1000))
small_eval_dataset = tokenized_datasets["test"].shuffle(seed=42).select(range(1000))

# %%
# Check all the features/elements in a sample
tokenized_datasets["train"].features

# %%
# Get sample 0 and see its values: label, text, input_ids, etc.
# We could apply several functions to the tokenized_datasets
# tokenized_datasets.remove_columns(...)
# tokenized_datasets.rename_columns("label", "labels")
# tokenized_datasets.with_format("torch")
tokenized_datasets["train"][0]
```
#### Training with the Pytorch Trainer

```python
import numpy as np
from transformers import AutoModelForSequenceClassification
from transformers import TrainingArguments, Trainer
import evaluate

model = AutoModelForSequenceClassification.from_pretrained("bert-base-cased", num_labels=5)

# %%
# We need to define a metric using the HuggingFace library evaluate
metric = evaluate.load("accuracy")

# %%
def compute_metrics(eval_pred):
    logits, labels = eval_pred
    predictions = np.argmax(logits, axis=-1)
    return metric.compute(predictions=predictions, references=labels)

# %%
training_args = TrainingArguments(output_dir="test_trainer", evaluation_strategy="epoch")

# %%
# Create a Trainer instance with all defined components
trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=small_train_dataset,
    eval_dataset=small_eval_dataset,
    compute_metrics=compute_metrics,
)

# %%
# TRAIN!
# WARNING: mlflow and Wandb are used
# We should use the Wandb API key
trainer.train()
```
#### Training in Native Pytorch

```python
tokenized_datasets["train"].features

# %%
# In order to be able to run the native Pytorch training loop
# we need to modify the dataset as follows
tokenized_datasets = tokenized_datasets.remove_columns(["text"])
tokenized_datasets = tokenized_datasets.rename_column("label", "labels")
tokenized_datasets.set_format("torch")

# %%
# Now, we have a modified dataset
tokenized_datasets["train"].features

# %%
# We take a smaller subset
small_train_dataset = tokenized_datasets["train"].shuffle(seed=42).select(range(1000))
small_eval_dataset = tokenized_datasets["test"].shuffle(seed=42).select(range(1000))

# %%
# Data Loader
from torch.utils.data import DataLoader

train_dataloader = DataLoader(small_train_dataset, shuffle=True, batch_size=8)
eval_dataloader = DataLoader(small_eval_dataset, batch_size=8)

# %%
# Model
from transformers import AutoModelForSequenceClassification

model = AutoModelForSequenceClassification.from_pretrained("bert-base-cased", num_labels=5)

# %%
# Optimizer
from torch.optim import AdamW

optimizer = AdamW(model.parameters(), lr=5e-5)

# %%
# Scheduler
from transformers import get_scheduler

num_epochs = 3
num_training_steps = num_epochs * len(train_dataloader)
lr_scheduler = get_scheduler(
    name="linear", optimizer=optimizer, num_warmup_steps=0, num_training_steps=num_training_steps
)

# %%
# Device
import torch

#device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
device = "cpu"
print(device)
model.to(device)

# %%
# Training loop
from tqdm.auto import tqdm

progress_bar = tqdm(range(num_training_steps))

model.train()
for epoch in range(num_epochs):
    for batch in train_dataloader:
        batch = {k: v.to(device) for k, v in batch.items()}
        outputs = model(**batch)
        loss = outputs.loss
        loss.backward()

        optimizer.step()
        lr_scheduler.step()
        optimizer.zero_grad()
        progress_bar.update(1)

# %%
# Evaluate
import evaluate

metric = evaluate.load("accuracy")
model.eval()
for batch in eval_dataloader:
    batch = {k: v.to(device) for k, v in batch.items()}
    with torch.no_grad():
        outputs = model(**batch)

    logits = outputs.logits
    predictions = torch.argmax(logits, dim=-1)
    metric.add_batch(predictions=predictions, references=batch["labels"])

metric.compute()
```

### Finetuning Pre-Trained Models with Custom Datasets - Example with IMDB

Source: [Fine-tuning with custom datasets](https://huggingface.co/transformers/v3.2.0/custom_datasets.html).

```python
# %%
!wget http://ai.stanford.edu/~amaas/data/sentiment/aclImdb_v1.tar.gz

# %%
!tar -xf aclImdb_v1.tar.gz

# %%
# This data is organized into pos and neg folders with one text file per example
from pathlib import Path

def read_imdb_split(split_dir):
    split_dir = Path(split_dir)
    texts = []
    labels = []
    for label_dir in ["pos", "neg"]:
        for text_file in (split_dir/label_dir).iterdir():
            texts.append(text_file.read_text())
            labels.append(0 if label_dir is "neg" else 1)

    return texts, labels

train_texts, train_labels = read_imdb_split('aclImdb/train')
test_texts, test_labels = read_imdb_split('aclImdb/test')

# %%
from sklearn.model_selection import train_test_split
train_texts, val_texts, train_labels, val_labels = train_test_split(train_texts, train_labels, test_size=.2)

# %%
# We’ll eventually train a classifier using pre-trained DistilBert,
# so let’s use the DistilBert tokenizer.
from transformers import DistilBertTokenizerFast
tokenizer = DistilBertTokenizerFast.from_pretrained('distilbert-base-uncased')

# %%
# Now we can simply pass our texts to the tokenizer.
# We’ll pass truncation=True and padding=True,
# which will ensure that all of our sequences are padded to the same length
# and are truncated to be no longer model’s maximum input length.
# This will allow us to feed batches of sequences into the model at the same time.
train_encodings = tokenizer(train_texts, truncation=True, padding=True)
val_encodings = tokenizer(val_texts, truncation=True, padding=True)
test_encodings = tokenizer(test_texts, truncation=True, padding=True)

# %%
# Let’s turn our labels and encodings into a Dataset object
import torch

class IMDbDataset(torch.utils.data.Dataset):
    def __init__(self, encodings, labels):
        self.encodings = encodings
        self.labels = labels

    def __getitem__(self, idx):
        item = {key: torch.tensor(val[idx]) for key, val in self.encodings.items()}
        item['labels'] = torch.tensor(self.labels[idx])
        return item

    def __len__(self):
        return len(self.labels)

train_dataset = IMDbDataset(train_encodings, train_labels)
val_dataset = IMDbDataset(val_encodings, val_labels)
test_dataset = IMDbDataset(test_encodings, test_labels)

# %%
# Fine-tuning with Trainer
from transformers import DistilBertForSequenceClassification, Trainer, TrainingArguments

training_args = TrainingArguments(
    output_dir='./results',          # output directory
    num_train_epochs=3,              # total number of training epochs
    per_device_train_batch_size=16,  # batch size per device during training
    per_device_eval_batch_size=64,   # batch size for evaluation
    warmup_steps=500,                # number of warmup steps for learning rate scheduler
    weight_decay=0.01,               # strength of weight decay
    logging_dir='./logs',            # directory for storing logs
    logging_steps=10,
)

model = DistilBertForSequenceClassification.from_pretrained("distilbert-base-uncased")

trainer = Trainer(
    model=model,                         # the instantiated 🤗 Transformers model to be trained
    args=training_args,                  # training arguments, defined above
    train_dataset=train_dataset,         # training dataset
    eval_dataset=val_dataset             # evaluation dataset
)

trainer.train()

# %%
# Fine-tuning with native PyTorch
from torch.utils.data import DataLoader
from transformers import DistilBertForSequenceClassification, AdamW

#device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
device = "cpu"

model = DistilBertForSequenceClassification.from_pretrained('distilbert-base-uncased')
model.to(device)
model.train()

train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True)

optim = AdamW(model.parameters(), lr=5e-5)

for epoch in range(3):
    for batch in train_loader:
        optim.zero_grad()
        input_ids = batch['input_ids'].to(device)
        attention_mask = batch['attention_mask'].to(device)
        labels = batch['labels'].to(device)
        outputs = model(input_ids, attention_mask=attention_mask, labels=labels)
        loss = outputs[0]
        loss.backward()
        optim.step()

model.eval()
```
