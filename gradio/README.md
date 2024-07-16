# Gradio

[Gradio](https://www.gradio.app/)

> is the fastest way to demo your machine learning model with a friendly web interface so that anyone can use it, anywhere!

It is owned and extensively used by [HuggingFace](https://huggingface.co/) and it is quite similar to [Streamlit](https://streamlit.io/) &mdash; check my guide on Streamlit here: [](https://github.com/mxagar/streamlit_guide).

This current guide contains my notes on the basic functionalities of [Gradio](https://www.gradio.app/). I created the notes after following the [Hugging Face Bootcamp by JM Portilla, Udemy](https://www.udemy.com/course/complete-hugging-face-bootcamp). I have further notes on more ML/model-related topics referrng to Haggig Face at [mxagar/tool_guides/hugging_face](https://github.com/mxagar/tool_guides/tree/master/hugging_face).

Notes:

- I am adding modified versions of the notebooks used in the bootcamp. I tried to find the original repository to fork it, but unfortunately I didn't find it.
- The slides are public and accessible under [https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz](https://drive.google.com/drive/folders/1KPNewt6K68qdwYq5bGAauBVjjx0Mr-zz).

Table of contents:

- [Gradio](#gradio)
  - [Setup](#setup)
  - [Introduction](#introduction)
  - [Gradio Components](#gradio-components)
  - [Layouts and Multiple Components](#layouts-and-multiple-components)

## Setup

See [mxagar/tool_guides/hugging_face/udemy_hugging_face](https://github.com/mxagar/tool_guides/tree/master/hugging_face/udemy_hugging_face).

To set up a conda environment, you ca use the co-located [`conda.yaml`](./conda.yaml):

```bash
conda env create -f conda.yaml
conda activate hf
```

Notebooks:

- [`01-Gradio-Introduction.ipynb`](./notebooks/01-Gradio-Introduction.ipynb)
- [`02-Layout.ipynb`](./notebooks/02-Layout.ipynb)
- [`03-Interactions.ipynb`](./notebooks/03-Interactions.ipynb)
- [`04-Image-ML-Integration.ipynb`](./notebooks/04-Image-ML-Integration.ipynb)
- [`05-Text-ML-Integration.ipynb`](./notebooks/05-Text-ML-Integration.ipynb)
- [`06-Modals-Errors.ipynb`](./notebooks/06-Modals-Errors.ipynb)
- [`07-Styling-and-Themes.ipynb`](./notebooks/07-Styling-and-Themes.ipynb)

Usually, GUIs are created in scripts, not in notebooks.
However, for learning puposes, here are shown in notebooks.
Each time we run `gradio.Interface.launch()` a web snippet is embedded in the notebook with the GUI; additionally, we can open the same GUI in `http://127.0.0.1:7860` or a successive port.

## Introduction

Key ideas:

- Gradio is made up by components: text, image, etc.
- Components can be interchangeably input or output components
- We define an interface layout where the components are set.
- Then, we define our Python code which connects to those components.

![Gradio Components](./assets/gradio_components.png)

## Gradio Components

Notebook: [`01-Gradio-Introduction.ipynb`](./notebooks/01-Gradio-Introduction.ipynb).

> Gradio includes pre-built [components](https://www.gradio.app/docs/gradio/introduction) that can be used as inputs or outputs in your Interface or Blocks with a single line of code

Original components diagram:

![Components Architecture](./assets/gradio_components_architecture.png)

Some of the most common components are:

> - **Number**
> - **Textbox**
> - **Slider**
> - **Image**: Lets users upload an image or take a picture for image processing models.
> - **Dropdown**
> - **Label**: Displays text output. Commonly used to show the result from classification models.
> - **JSON**: Handle JSON data. Ideal for displaying raw model outputs or more complex data structures.

```python
import gradio as gr

### -- Number

# We need to define a function, which will take
# inputs to process them and generate outputs
def add_numbers(x, y):
    return x + y

import gradio as gr

# We create the GUI just with this line!
# https://www.gradio.app/docs/gradio/interface
iface = gr.Interface(
    fn=add_numbers,
    # Input components
    inputs=[
        gr.Number(10), # gr.Number() can take a default value, label, info, etc.
        gr.Number()
    ], 
    # Output components
    outputs=gr.Number()
)
# When we call the launch method, the GUI will appear (if we're in a notebook)
# and the function will be executed when the user interacts with it
# Otherwise, we'll get a URL where we can access the GUI
# http://127.0.0.1:7860
# If we launch more gr.Interface objects, they will be available in different ports
# The `Flag` button can be used to *flag* outputs for manual review.
# The flagged outputs are stored within the `flagged` directory.
# We have a Submit button: when we click it,
# the inputs will be processed by fn and outputs will be displayed
iface.launch()


### -- Text

def reverse_text(input_text):
    return input_text[::-1]

iface = gr.Interface(fn=reverse_text, inputs=gr.Text(), outputs=gr.Text())
iface.launch()


### -- Slider

def slider_example(value):
    return f"Slider value: {value}"

iface = gr.Interface(fn=slider_example, inputs=gr.Slider(minimum=0, maximum=100), outputs=gr.Text())
iface.launch()


### -- Image

from PIL import Image
import numpy as np

def to_grayscale(input_image):
    # Watch out: gr.Image() is by default a numpy array
    # so we need to expect a numpy array here
    grayscale_image = np.mean(input_image, axis=2, keepdims=True)
    grayscale_image = np.tile(grayscale_image, (1, 1, 3))
    return grayscale_image.astype(np.uint8)

iface = gr.Interface(
    fn=to_grayscale,
    # Watch out: gr.Image() is by default a numpy array
    inputs=gr.Image(),
    outputs=gr.Image()
)
iface.launch()

### -- Dropdown

def handle_dropdown(selection):
    return f"You selected: {selection}"

options = ["Option 1", "Option 2", "Option 3"]
iface = gr.Interface(fn=handle_dropdown, inputs=gr.Dropdown(choices=options), outputs=gr.Text())
iface.launch()


### -- JSON

def number_details(number):
    details = {
        "original": number,
        "squared": number ** 2,
        "sqrt": number ** 0.5,
        "is_even": number % 2 == 0
    }
    return details

iface = gr.Interface(fn=number_details, inputs=gr.Number(), outputs=gr.Json())
iface.launch()


### -- Label

def classify_number(number):
    if number > 0:
        return "Positive"
    elif number < 0:
        return "Negative"
    else:
        return "Zero"

iface = gr.Interface(fn=classify_number, inputs=gr.Number(), outputs=gr.Label())
iface.launch()
```

Some output snapshots:

![Gradio Number Component](./assets/gradio_number.png)

![Gradio Image Component](./assets/gradio_image.png)

## Layouts and Multiple Components

It is possible to display more than one component or widget. We can achieve that with layouts that consist of blocks and rows:

- We generate a layout.
- Then we add a block.
- Each block can have several rows.
- We can add components to rows. The components in a row are equally sized/spaced.
- The users don't see the blocks and rows, but only the components nicely outlayed!

![Layouts: Blocks, Rows, Components](./assets/layouts.png)



