# Ollama

Ollama is a tool to run local Large Language Models (LLMs), among others.

- Website: [https://ollama.com/](https://ollama.com/)
- Github: [https://github.com/ollama/ollama](https://github.com/ollama/ollama)
- Python Library: [https://github.com/ollama/ollama-python](https://github.com/ollama/ollama-python)

Sources of this guide:

- [How to Use Ollama: Hands-On With Local LLMs and Building a Chatbot](https://hackernoon.com/how-to-use-ollama-hands-on-with-local-llms-and-building-a-chatbot)
- [Running models with Ollama step-by-step](https://medium.com/@gabrielrodewald/running-models-with-ollama-step-by-step-60b6f6125807)
- [Using Ollama in your IDE with Continue](https://medium.com/@omargohan/using-ollama-in-your-ide-with-continue-e8cefeeee033)

## Setup

- First, we need to install the binaries from their website (Mac/Windows/Linux): [https://ollama.com/download](https://ollama.com/download).
- Then, we need to install `ollama` in our Python environment of choise: `pip install ollama`.

## Basic Usage

```bash
# Run ollama: help menu is displayed
ollama

    Usage:
    ollama [flags]
    ollama [command]

    Available Commands:
    serve       Start ollama
    create      Create a model from a Modelfile
    show        Show information for a model
    run         Run a model
    pull        Pull a model from a registry
    push        Push a model to a registry
    list        List models
    ps          List running models
    cp          Copy a model
    rm          Remove a model
    help        Help about any command

    Flags:
    -h, --help      help for ollama
    -v, --version   Show version information

# List all downloaded LLMs
ollama list

# Download a model: See the Library of Models
ollama pull llama3

# Run a model
ollama run llama3

# Remove a model
ollama rm llama3
```

### Library and Models

The complete model library can be accessed here: [https://ollama.com/library](https://ollama.com/library).

Some selected models from the official Github repository:

| Model              | Parameters | Size  | Download and Run               |
| ------------------ | ---------- | ----- | ------------------------------ |
| Llama 3            | 8B         | 4.7GB | `ollama run llama3`            |
| Llama 3            | 70B        | 40GB  | `ollama run llama3:70b`        |
| Phi 3 Mini         | 3.8B       | 2.3GB | `ollama run phi3`              |
| Phi 3 Medium       | 14B        | 7.9GB | `ollama run phi3:medium`       |
| Gemma              | 2B         | 1.4GB | `ollama run gemma:2b`          |
| Gemma              | 7B         | 4.8GB | `ollama run gemma:7b`          |
| Mistral            | 7B         | 4.1GB | `ollama run mistral`           |
| Moondream 2        | 1.4B       | 829MB | `ollama run moondream`         |
| Neural Chat        | 7B         | 4.1GB | `ollama run neural-chat`       |
| Starling           | 7B         | 4.1GB | `ollama run starling-lm`       |
| Code Llama         | 7B         | 3.8GB | `ollama run codellama`         |
| Llama 2 Uncensored | 7B         | 3.8GB | `ollama run llama2-uncensored` |
| LLaVA              | 7B         | 4.5GB | `ollama run llava`             |
| Solar              | 10.7B      | 6.1GB | `ollama run solar`             |

> Note: You should have at least 8 GB of RAM available to run the 7B models, 16 GB to run the 13B models, and 32 GB to run the 33B models.

### Importing Models from Hugging Face

We can download models from [Hugging Face](https://huggingface.co/) and import them as ollama models using `ollama create`. Two formats are supported:

- `GGUF`
- `safetensors`

If we want to use [Hugging Face](https://huggingface.co/) programmatically, we need to have an account and also the CLI tools installed.
Hugging Face will create a local folder in `~/.cache/huggingface` where all models and datasets will be downloaded to if not specified otherwise.
Alternatively, we can also download them manually from the web, or we can also clone the HuggingFace model repo if we have installed the LFS extension for Git.

Here's a quick recipy for all those steps:

```bash
# Install in our environment
pip install "huggingface_hub[cli]"

# Login: web us prompted
huggingface-cli login

# Then, we can browse and select a model and
# download from the hub to ~/.chache/huggingface (default)
# or specify a local folder which will contain .huggingface/download with the model inside
# https://huggingface.co/TheBloke/MistralLite-7B-GGUF
huggingface-cli download TheBloke/MistralLite-7B-GGUF mistrallite.Q4_K_M.gguf --local-dir ..\models --local-dir-use-symlinks False

# Alternatively, we can also download them manually from the web
# or we can also clone the HuggingFace model repo
# if we have installed the LFS extension for Git
git lfs install
cd ../models
# https://huggingface.co/bartowski/Starling-LM-7B-beta-GGUF
git clone https://huggingface.co/bartowski/Starling-LM-7B-beta-GGUF
# https://huggingface.co/ibm-granite/granite-3b-code-instruct
git clone https://huggingface.co/ibm-granite/granite-3b-code-instruct
```

Once we have the model, we need to create a `Modelfile` which will contain at least the `FROM` command with the path of the model:

```Dockerfile
FROM /path/to/file.gguf
FROM /path/to/safetensors/directory
```



#### Modelfile

The `Modelfile` can contain also a prompt in `TEMPLATE` and parameters as `PARAMETER`.
We can show the `Modelfile` of an `ollama` model as follows:

```bash
ollama show --modelfile llama3
```

We can browse in the Ollama website examples and see the `Modelfile` definitions; for instance, [https://ollama.com/library/phi3](https://ollama.com/library/phi3):

```Dockerfile
TEMPLATE """{{ if .System }}<|system|>
{{ .System }}<|end|>
{{ end }}{{ if .Prompt }}<|user|>
{{ .Prompt }}<|end|>
{{ end }}<|assistant|>
{{ .Response }}<|end|>
"""
PARAMETER stop "<|end|>"
PARAMETER stop "<|user|>"
PARAMETER stop "<|assistant|>"
```

Other `Modelfile` examples I have found:

`Starling-LM-7B-beta-GGUF`: 

```Dockerfile
FROM "./Starling-LM-7B-beta-Q6_K.gguf"
PARAMETER stop "<|im_start|>"
PARAMETER stop "<|im_end|>"
TEMPLATE """
<|im_start|>system
<|im_end|>
<|im_start|>user
<|im_end|>
<|im_start|>assistant
"""
```

#### Quantiation Summary

TBD.

## Links

- [HuggingFace CLI Guide](https://huggingface.co/docs/huggingface_hub/main/en/guides/cli).
- [GGUF Format](https://huggingface.co/docs/hub/gguf)
- [My HuggingFace Basic Guide]()
- [Ollama Modelfile](https://github.com/ollama/ollama/blob/main/docs/modelfile.md)