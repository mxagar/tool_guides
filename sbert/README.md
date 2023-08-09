# SBERT: Senstence Transformers

[SBERT](https://www.sbert.net/) is a powerful library which is able to convert any text into text embedding vectors which capture semantic meaning. Then, those vectors can be used for downstream applications, such as:

- Clustering
- Look for similar documents
- Classification
- etc.

Here is the original [SBERT Paper](https://arxiv.org/abs/1908.10084).

And here are some quick features of SBERT, highlighted by me:

- It is lightweight.
- It can be used within the [HuggingFace framework](https://huggingface.co/sentence-transformers) or without it, simply by installing it with `pip install -U sentence-transformers`.
- The embedding vector size is 384.
- There are many models available, among which we can find multi-lingual models (i.e., English & German & Spanish, etc.)
- We can pass to it regular natural text or even short one-two word clauses.
- The official documentation seems very good: [https://www.sbert.net/index.html](https://www.sbert.net/index.html).

The notebook [`sbert_tests.ipynb`](./sbert_tests.ipynb) performs several tests and implements downstream applications:

- Tests with the SBERT or `sentence_transformers` library
- Tests with the HuggingFace library
- Evaluation with the [AG News Classification Dataset](https://www.kaggle.com/datasets/amananandrai/ag-news-classification-dataset)
  - Comparison of the simmilarity matrices of text embeddings and `CountVectorizer` one-hot vectors
  - Basic K-Means clustering and T-SNE visualization
  - PCA compression analysis
  - Classification with Random Forest Models

Note that no thorough NLP processing is performed in the notebook, instead, the focus lies on the evaluation of SBERT.

In order to **setup an environment**, you can follow this recipe:

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate sbert

# Alternatively, if you have a DS env (with matplotlib, numpy, pandas, pytorch & Co.),
# you can extend it with the following packages
pip install -U sentence-transformers
pip install transformers

# OPTIONAL: If we have a CUDA device,
# we can speed up the encoding:
# Example: Pytorch on Windows + CUDA 11.7
# Update your NVIDIA drivers: https://www.nvidia.com/Download/index.aspx
# I have version 12.1, but it works with older versions, e.g. 11.7
# Check your CUDA version with: nvidia-smi.exe
# In case of any runtime errors, check vrsion compatibility tables:
# https://github.com/pytorch/vision#installation
python -m pip install -U torch==1.13+cu117 torchvision==0.14+cu117 torchaudio torchtext==0.14 --index-url https://download.pytorch.org/whl/cu117
```

For more related information, you can check these repositories of mine:

- [deep_learning_udacity](https://github.com/mxagar/deep_learning_udacity)
- [nlp_guide](https://github.com/mxagar/nlp_guide)

Mikel Sagardia, 2023.  
Enjoy and link it if you find it useful.  
No guarantees.  
