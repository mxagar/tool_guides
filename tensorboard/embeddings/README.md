# Tensorboard Embedding Projector

This tutorial shows how to use the Tensorboard embedding projector independently from the framework used to generate the embedding vectors.

Basically, we need to follow these steps:

- First, we need a CSV/dataframe with all the image paths and embedding vectors.
- We create a TSV file `image_embeddings.tsv` with all the embedding vectors.
- We create a TSV file with the metadata of the images, such as `label` and `cluster`: `image_metadata.tsv`.
- We create an image sprite/mosaic with all the resized images; the order of the images must be the same as in the TSV files.
- We create a configuration file `projector_config.pbtxt` which points to the TSV files and the image sprite.
- We launch `tensorboard` on the setting as the `logdir` the directory where all the TSV and PBTXT files are.

Before running the application, image/text embeddings need to be generated. In this example, I use the image embeddings from my other repository [simclr_pytorch_flowers](https://github.com/mxagar/simclr_pytorch_flowers), where the embedding vectors of the [Flowers dataset](https://www.kaggle.com/datasets/imsparsh/flowers-dataset) from Kaggle are generated in [`datasets/vectors_dataset.csv`](./datasets/vectors_dataset.csv).

```python
import os
import sys

import numpy as np
from tqdm import tqdm
from PIL import Image
import pandas as pd
import ast

import warnings
warnings.filterwarnings("ignore", message=".*The 'nopython' keyword.*")

## Variables
class Config:
    def __init__(self):
        self.vector_dataset_path = "../datasets/vectors_dataset.csv"
        self.output_path = "./output"
        os.makedirs(self.output_path, exist_ok=True)  # Create the output_path directory if it doesn't exist

config = Config()

## Load Dataset
df = pd.read_csv(config.vector_dataset_path)
df['embedding'] = df['embedding'].apply(ast.literal_eval)
df.head()
# 	filename	filepath	label	linear_pred	embedding	cluster

## Create Image Sprite
# Fix small image size
size = 60
image_width, image_height = size, size
# Resize all images
images = [Image.open(filename).resize((image_width,image_height)) for filename in tqdm(df['filepath'])]

one_square_size = int(np.ceil(np.sqrt(len(images))))
master_width = (image_width * one_square_size) 
master_height = image_height * one_square_size

spriteimage = Image.new(
    mode='RGBA',
    size=(master_width, master_height),
    color=(0,0,0,0))  # fully transparent

for count, image in enumerate(images):
    div, mod = divmod(count,one_square_size)
    h_loc = image_width*div
    w_loc = image_width*mod    
    spriteimage.paste(image,(w_loc,h_loc))

image_sprinte_filepath = os.path.join(config.output_path, "sprite.jpg")
spriteimage.convert("RGB").save(image_sprinte_filepath, transparency=0)

## Save Embedding Vectors and Metadata as TSV
embeddings_list = df["embedding"].tolist()
embeddings_df = pd.DataFrame(embeddings_list)
embedding_tsv_filepath = os.path.join(config.output_path, "image_embeddings.tsv")
embeddings_df.to_csv(embedding_tsv_filepath, sep='\t', header=False, index=False)

filenames_df = df[["filename", "filepath", "label", "cluster"]]
filename_tsv_filepath = os.path.join(config.output_path, "image_metadata.tsv")
filenames_df.to_csv(filename_tsv_filepath, sep='\t', header=True, index=False)

## Save Config File
config_filepath = os.path.join(config.output_path, "projector_config.pbtxt")

content = '''embeddings {
  tensor_path: "image_embeddings.tsv"
  metadata_path: "image_metadata.tsv"
  sprite {
    image_path: "sprite.jpg"
    single_image_dim: 60
    single_image_dim: 60
  }
}'''

with open(config_filepath, 'w') as file:
    file.write(content)
```

To run Tensorboard:

```bash
# Anaconda Powershell
conda activate ds
pip install tensorflow
pip install tensorboard

# On Windows:
# Add path to tensorboard executable in the environment variables:
# Path += C:\Users\<UserName>\AppData\Roaming\Python\Python39\Scripts

cd /path/to/all/tsv/sprite/and/config/files
# Don't use blank spaces
tensorboard --logdir=./
# Open borwser at http://localhost:6006/
# Refresh it several times until it works
```
