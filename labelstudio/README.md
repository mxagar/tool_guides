# Label Studio

[Label Studio](https://labelstud.io/) is a tool for data labeling. There are two versions at the moment:

- Community edition (free), used here; it runs locally.
- Enterprise cloud service edition (price unknown); hosted by them.

The tool has many useful functionalities:

- Labeling many data formats and applications:
  - Computer Vision: Classification, Segmentation, Object Detection, Keypoints, OCR, Image Captioning, etc.
  - NLP: Classification, QA, Translation, Summarization, etc.
  - Audio: Speech Recognition, etc.
  - Videos
  - ...
- We can add a Machine Learning Backend which assists the labeling.
- There is an SDK with which we can label programmatically.
- etc.

This guide focuses mainly on image labeling.

Note that this repository comes with auxiliary files/scripts used across the guide:

- [`utils.ipynb`](./utils.ipynb)
- [`serve_local_files.py`](./serve_local_files.py)

Table of contents:

- [Label Studio](#label-studio)
  - [Installation and Basic Setup](#installation-and-basic-setup)
    - [Create a Project](#create-a-project)
    - [Serve the Images as URLs](#serve-the-images-as-urls)
    - [Basic Usage: Label and Export Annotations](#basic-usage-label-and-export-annotations)
  - [More Advanced Usage](#more-advanced-usage)
    - [Notes on the Basic Label Studio JSON Format](#notes-on-the-basic-label-studio-json-format)
    - [Import Pre-Annotated Datasets](#import-pre-annotated-datasets)
    - [Basic API Usage](#basic-api-usage)
    - [Machine Learning Backend](#machine-learning-backend)
    - [Basic SDK Usage](#basic-sdk-usage)
  - [Notes on Examples and Use-Cases](#notes-on-examples-and-use-cases)
    - [Object Detection](#object-detection)
    - [Semantic Segmentation](#semantic-segmentation)


## Installation and Basic Setup

There are several ways of seting Label Studio up:

- `pip install`
- Docker
- Compilation from source

I tried two options and found complications with both:

- The `pip` local installation has sometimes issues with the `sqlite3.dll` library; the only way I managed to solve it is to create a new fresh conda environment using the committed `conda.yaml`.
- The docker container works right away, but its access to the local machine is restricted. The docker container will create a volume in the local folder we call docker from: `/mydata`. That might be an issue if we want to access images outside from that volume. However, not that the usual way of accessing to data/images is via a web server.

More information: [Install and upgrade](https://labelstud.io/guide/install.html).

```bash
# Create or activate your preferred python environment
conda env create -f conda.yaml
conda activate label

# If not done yet, install Label Studio
pip install --user label-studio
# If on Windows, add to Path the correct URL to be able to locate the binary
# C:\Users\<User>\AppData\Roaming\Python\Python39\Scripts

# Start Label Studio
label-studio start
# Open Browser at http://localhost:8080

#### -- Alternative: Docker Container

cd path/to/project
# Mac, Linux, WSL2
docker run -it -p 8080:8080 -v $(pwd)/mydata:/label-studio/data heartexlabs/label-studio:latest
# Windows PowerShell
docker run -it -p 8080:8080 -v ${PWD}/mydata:/label-studio/data heartexlabs/label-studio:latest label-studio --log-level DEBUG
# Open Browser at http://localhost:8080

# Shut down
# Ctrl+C
# Stop
docker stop label-studio
docker rm label-studio
docker ps

```

### Create a Project

For testing puposes, I used the [Flowers dataset](https://www.kaggle.com/datasets/imsparsh/flowers-dataset) from Kaggle, placed in the local folder `data/flowers` (not committed).

Label Studio can work with local files, i.e., we drag and drop the images we would like to use, for instance. However, **the most common/advised way is to serve those images (e.g., with a web server) and provide the URLs to Label Studio.**

The official guide suggests these steps after we launch Label Studio:

> - (Open the Label Studio UI at http://localhost:8080.)
> - Sign up with an email address and password that you create.
> - Click `Create` to create a project and start labeling data.
> - Name the project, and if you want, type a description and select a color.
> - Click Data Import and upload the data files that you want to use. **If you want to use data from a local directory, cloud storage bucket, or database, skip this step for now.**
> - Click Labeling Setup and choose a template and customize the label names for your use case. There are many options:
    - Computer Vision: Classification, Segmentation, Object Detection, Keypoints, OCR, Image Captioning, etc.
    - NLP: Classification, QA, Translation, Summarization, etc.
    - Audio: Speech Recognition, etc.
    - Videos
    - ...
> - Click `Save` to save your project.

We can access to the **Project Settings** clicking on the `...` icon.
In the project settings, in the *Labeling Interface*, we edit the UI with an XML code:

```xml
<View>
  <Image name="image" value="$image_path" zoom="true" zoomControl="true" rotateControl="true"/>
  <Choices name="class" toName="image">
    <Choice value="daisy"/>
    <Choice value="dandelion"/>
    <Choice value="rose"/>
    <Choice value="sunflower"/>
    <Choice value="tulip"/>
  </Choices>
</View>
```

Note the field `$image_path`.

### Serve the Images as URLs

As mentioned, Label Studio can work with local image paths, but the usual/recommended way of working is with image URLs, being images served via a web server, for instance. These are the steps I followed to achieve that serving:

- Using [`utils.ipynb`](./utils.ipynb), I created a CSV with all file URLs based on a local path.
- Using [`serve_local_files.py`](./serve_local_files.py) I served the images from a local path with a web server using CORS.

The web server is built with Flask; the `SERVER_DIRECTORY` specified in both files must be the same.

Effectively, we serve the image in such a way that they can be opened with a web browser:

    SERVER_DIRECTORY
        C:/Users/Msagardi/git_repositories/tool_guides/labelstudio/data/
    Original path
        ${SERVER_DIRECTORY}/flowers/test/Image_10.jpg
    URL: We can open the image one the browser
        http://localhost:8000/flowers/test/Image_10.jpg

Once the CSV is generated, we open the project in Label Studio and hit `Import`. Then, the image entries are generated.

In the following, the contents of the mentioned files are attached.

[`utils.ipynb`](./utils.ipynb)

```python
import os
import csv

def list_image_files(directory, server_directory,  base_url="http://localhost:8000/"):
    """
    Recursively lists all image URLs from a local server for the images in the given directory and its subdirectories.

    :param directory: Path to the directory.
    :param server_directory: Path to the directory from which the server is started.
    :return: List of URLs to image files served from a local server.
    """
    
    # List of common image extensions
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff', '.webp']

    # Recursively walk through the directory
    image_urls = []
    for dirpath, _, filenames in os.walk(directory):
        for filename in filenames:
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                # Convert file path to a URL path
                relative_path = os.path.relpath(os.path.join(dirpath, filename), server_directory)
                web_path = relative_path.replace('\\', '/')
                full_url = base_url + web_path
                
                image_urls.append(full_url)
                
    return image_urls

def save_to_csv(image_paths, output_file):
    """
    Save list of image paths to a CSV file.

    :param image_paths: List of image paths.
    :param output_file: Path to the output CSV file.
    """
    with open(output_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["image_path"])  # Writing the header
        for path in image_paths:
            csv_writer.writerow([path])

directory_path = 'C:/Users/Msagardi/git_repositories/tool_guides/labelstudio/data/flowers/test/'
server_directory = 'C:/Users/Msagardi/git_repositories/tool_guides/labelstudio/data'
image_paths = list_image_files(directory_path, server_directory)
print(image_paths[:5]) # ['http://localhost:8000/flowers/test/Image_1.jpg', 'http://localhost:8000/flowers/test/Image_10.jpg', ...

# Save CSV with image URLs
output_csv_path = 'image_paths.csv'
save_to_csv(image_paths, output_csv_path)

# Now, in ./serve_local_files.py, we need to set
#   SERVER_DIRECTORY = server_directory
# and execute it:
#   python serve_local_files.py
# With that, we're going to get the images served in the URLs
```

[`serve_local_files.py`](./serve_local_files.py)

```python
from flask import Flask, send_from_directory
from flask_cors import CORS

SERVER_DIRECTORY = 'C:/Users/Msagardi/git_repositories/tool_guides/labelstudio/data/'

app = Flask(__name__)
CORS(app)  # This will enable CORS for all routes

@app.route('/<path:path>')
def serve_file(path):
    return send_from_directory(SERVER_DIRECTORY, path)

if __name__ == '__main__':
    app.run(port=8000)

```

### Basic Usage: Label and Export Annotations

When the images are served and the project is created, we can click on it in the project dashboard and perform several actions in the project list overview:

- `Import` the CSV with the image URLs; then, the images are listed automatically. Images or samples are **tasks**.
- `Label All Tasks`: label the images with the custom UI we have defined in the project creation (accessible in the `Settings`).
- `Filter` the samples/tasks, according to their properties; if we upload a CSV/JSON with custom fields in `data`, these can be used here, too.
- `Export` the labelled images as a CSV or JSON. If filters are active, only the filtered results are exported!

![Projects UI](./assets/ls_ui_projects.png)

![Project List UI](./assets/ls_ui_project_list_overview.png)

![Labeling UI](./assets/ls_ui_labeling_overview.png)

The exported CSV looks like this (for some reason, empty lines are added):

```csv
"annotation_id","annotator","class","created_at","id","image_path","lead_time","updated_at"
2,"1","dandelion","2023-10-02T09:12:27.480432Z",2,"http://localhost:8000/flowers/test/Image_10.jpg",16.135,"2023-10-02T09:12:27.480432Z"
3,"1","daisy","2023-10-02T09:12:32.337558Z",3,"http://localhost:8000/flowers/test/Image_100.jpg",4.559,"2023-10-02T09:12:32.337662Z"
...
```

Analogously, the JSON export of two entries looks like this (note, an import follows the same format, but we don't need to use all those fields):

```json
[
    {
        "annotations": [
            {
                "completed_by": 1,
                "created_at": "2023-10-02T09:12:27.480432Z",
                "draft_created_at": null,
                "ground_truth": false,
                "id": 2,
                "import_id": null,
                "last_action": null,
                "last_created_by": null,
                "lead_time": 16.135,
                "parent_annotation": null,
                "parent_prediction": null,
                "prediction": {},
                "project": 6,
                "result": [
                    {
                        "from_name": "class",
                        "id": "CJIqR5nmdF",
                        "origin": "manual",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": [
                                "dandelion"
                            ]
                        }
                    }
                ],
                "result_count": 0,
                "task": 2,
                "unique_id": "4be41e98-2176-4ba2-b23c-512eeb09e8b9",
                "updated_at": "2023-10-02T09:12:27.480432Z",
                "updated_by": 1,
                "was_cancelled": false
            }
        ],
        "cancelled_annotations": 0,
        "comment_authors": [],
        "comment_count": 0,
        "created_at": "2023-09-29T17:04:38.137386Z",
        "data": {
            "image_path": "http:\/\/localhost:8000\/flowers\/test\/Image_10.jpg"
        },
        "drafts": [],
        "file_upload": "fde6c3c6-image_paths.csv",
        "id": 2,
        "inner_id": 2,
        "last_comment_updated_at": null,
        "meta": {},
        "predictions": [],
        "project": 6,
        "total_annotations": 1,
        "total_predictions": 0,
        "unresolved_comment_count": 0,
        "updated_at": "2023-10-02T09:12:27.545691Z",
        "updated_by": 1
    },
    {
        "annotations": [
            {
                "completed_by": 1,
                "created_at": "2023-10-02T09:12:32.337558Z",
                "draft_created_at": null,
                "ground_truth": false,
                "id": 3,
                "import_id": null,
                "last_action": null,
                "last_created_by": null,
                "lead_time": 4.559,
                "parent_annotation": null,
                "parent_prediction": null,
                "prediction": {},
                "project": 6,
                "result": [
                    {
                        "from_name": "class",
                        "id": "VbbozMXuC9",
                        "origin": "manual",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": [
                                "daisy"
                            ]
                        }
                    }
                ],
                "result_count": 0,
                "task": 3,
                "unique_id": "1dcf3c6b-59d9-424f-8392-da0574d1101f",
                "updated_at": "2023-10-02T09:12:32.337662Z",
                "updated_by": 1,
                "was_cancelled": false
            }
        ],
        "cancelled_annotations": 0,
        "comment_authors": [],
        "comment_count": 0,
        "created_at": "2023-09-29T17:04:38.137386Z",
        "data": {
            "image_path": "http:\/\/localhost:8000\/flowers\/test\/Image_100.jpg"
        },
        "drafts": [],
        "file_upload": "fde6c3c6-image_paths.csv",
        "id": 3,
        "inner_id": 3,
        "last_comment_updated_at": null,
        "meta": {},
        "predictions": [],
        "project": 6,
        "total_annotations": 1,
        "total_predictions": 0,
        "unresolved_comment_count": 0,
        "updated_at": "2023-10-02T09:12:32.398908Z",
        "updated_by": 1
    }
]
```

## More Advanced Usage

### Notes on the Basic Label Studio JSON Format

The JSON import format is the same as the one we output; the most important fields are (note that each of them can have children fields)

- `id`: Identifier for the labeling task from the dataset.
- `data`: Data copied from the input data task format. See the documentation for Task Format.
- `project`: Identifier for a specific project in Label Studio.
- `annotations`: Array containing the labeling results for the task. It has several children fields.
- `predictions`: Array of machine learning predictions. Follows the same format as the annotations array, with one additional parameter: `predictions.score`. It can be used to import pre-annotations.

Check these links for detaled information:

- [Basic Label Studio JSON format](https://labelstud.io/guide/tasks#Basic-Label-Studio-JSON-format)
- [Relevant JSON property descriptions](https://labelstud.io/guide/export#Relevant-JSON-property-descriptions)

Also, see examples below.

### Import Pre-Annotated Datasets

> If you have predictions generated for your dataset from a model, either as pre-annotated tasks or pre-labeled tasks, you can import the predictions with your dataset into Label Studio for review and correction. Label Studio automatically displays the pre-annotations that you import on the Labeling page for each task. 

The field `predictions` needs to be filled in the basic Label Studio JSON format:

- [Basic Label Studio JSON format](https://labelstud.io/guide/tasks#Basic-Label-Studio-JSON-format)
- [Relevant JSON property descriptions](https://labelstud.io/guide/export#Relevant-JSON-property-descriptions)

Following the flower example, if we define the UI with

```xml
<View>
  <Image name="image" value="$image_path" zoom="true" zoomControl="true" rotateControl="true"/>
  <Choices name="class" toName="image">
    <Choice value="daisy"/>
    <Choice value="dandelion"/>
    <Choice value="rose"/>
    <Choice value="sunflower"/>
    <Choice value="tulip"/>
  </Choices>
</View>
```

Then, an exemplary JSON file with two samples/entries could be:

```json
[
    {
        "data": {
            "image_path": "http://localhost:8000/flowers/test/Image_1.jpg"
        },
        "predictions": [
            {
                "result": [
                    {
                        "from_name": "class",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": ["daisy"]
                        }
                    }
                ]
            }
        ]
        /*
        "annotations": [
            {
                "result": [
                    {
                        "from_name": "class",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": ["daisy"]
                        }
                    }
                ]
            }
        ]
        */
    },
    {
        "data": {
            "image_path": "http://localhost:8000/flowers/test/Image_2.jpg"
        },
        "predictions": [
            {
                "result": [
                    {
                        "from_name": "class",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": ["rose"]
                        }
                    }
                ]
            }
        ]
        /*
        "annotations": [
            {
                "result": [
                    {
                        "from_name": "class",
                        "to_name": "image",
                        "type": "choices",
                        "value": {
                            "choices": ["rose"]
                        }
                    }
                ]
            }
        ]
        */
    }
    // ...
]
```

The function in [`utils.ipynb`](./utils.ipynb) which converts a list of image URLs into a pre-annotated JSON is the following:

```python
def save_to_json_preannotated(df, output_filepath):
    """
    Save DataFrame to a JSON file suitable for importing into Label Studio.

    :param df: DataFrame with columns: image_paths, prediction, cluster.
    :param output_filepath: Path to the output JSON file.
    """
    
    output_data = []

    for _, row in df.iterrows():
        item = {
            "data": {
                "image_path": row['image_paths'],
                "cluster": row['cluster']  # can be accessed in the Filters!
            },
            "predictions": [
                {
                    "result": [
                        {
                            "from_name": "class",
                            "to_name": "image",
                            "type": "choices",
                            "value": {
                                "choices": [row['prediction']]
                            }
                        }
                    ]
                }
            ]
        }
        output_data.append(item)
    
    with open(output_filepath, 'w') as outfile:
        json.dump(output_data, outfile, indent=4)


output_filepath = "images_paths_preannotated.json"
save_to_json_preannotated(df, output_filepath)
```

Note that the `cluster` field is a custom field which is not necessary; however, we can include such fields in `data` so that we use them in the `Filters`.

Even though the tasks/examples are pre-annotated, we need to go manually/individually through all of them and `submit` them one by one; in other words, **we cannot submit in bulk using the UI**. In order to submit tasks in bulk, we can use the API &mdash; see next section below.

### Basic API Usage

:construction:

TBD.

### Machine Learning Backend

:construction:

TBD.

### Basic SDK Usage

:construction:

TBD.

## Notes on Examples and Use-Cases

### Object Detection

:construction:

TBD.

### Semantic Segmentation

:construction:

TBD.
