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
- [`model.py`](./model.py)

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
    - [Basic SDK Usage](#basic-sdk-usage)
      - [Create a Project](#create-a-project-1)
      - [Add/Import Tasks: Empty of Pre-Annotated](#addimport-tasks-empty-of-pre-annotated)
    - [Machine Learning Backend](#machine-learning-backend)
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

# If we want to use the Label Studio SDK
pip install label-studio-sdk

# If we want to use the Label Studio ML backend tool
pip install --user label-studio-ml

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
    - Further project templates can be found in [Project Templates](https://labelstud.io/templates).
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

Note the field `$image_path`, which is later used whenever we import/create/update the image path of a **task**.
A **task** is a sample (e.g., image) which needs to be labelled.

Further project templates (i.e., XML specifications) can be found in [Project Templates](https://labelstud.io/templates).

The project data (the annotations, etc.) are stored by default in a single SQLite file; if we want to have more than 100k tasks and 5+ users, we need to use another database framework, like PostgreSQL. For more information check [Database setup](https://labelstud.io/guide/storedata.html).

In my local installation, the SQLite database is 

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

Label Studio starts a REST API which we can interact with, which resides under the same Label Studio server URL:

    http://localhost:8000/api

For all calls, we need to set our `LABEL_STUDIO_API_TOKEN` and pass it in the header; the token is obtained in the Label Studio account settings.

All API calls are listed here: [Label Studio API](https://labelstud.io/api).
Examples shown here:

- 1. List projects and their information.
- 2. List the tasks (labeled / samples to be labeled) of a project.
- 3. Label tasks programmatically in bulk.
- But there are much much more options!

```python
import requests

# Access the LABEL_STUDIO_API_TOKEN
LABEL_STUDIO_API_TOKEN = os.getenv("LABEL_STUDIO_API_TOKEN")

# Base URL
base_url = "http://localhost:8080"

# Setup headers with the API token
headers = {
    "Authorization": f"Token {LABEL_STUDIO_API_TOKEN}"
}

## -- 1. List projects and their information

response = requests.get(f"{base_url}/api/projects", headers=headers)

if response.status_code == 200:
    projects = response.json()
    # projects = {count, next, previous, results}
    for project_dict in projects["results"]: # for all projects
        #print(project_dict)  # dict
        project_json = json.dumps(project, indent=4) # json, for nice print
        print(project_json)
else:
    print(f"Failed to retrieve projects. Status code: {response.status_code}")
    print(response.text)  # This might give additional info about the error.

## -- 2. List the tasks in a project

# NOTE: There is also an export API!

def get_all_tasks(project_id, limit=None):
    page = 1
    tasks = []
    num_fetched_tasks = 0

    def determine_page_size(project_id):
        response = requests.get(f"{base_url}/api/projects/{project_id}/tasks?page=1", headers=headers)
        
        if response.status_code != 200:
            print(f"Failed to retrieve tasks to determine page size. Status code: {response.status_code}")
            return None

        tasks = response.json()
        return len(tasks)

    page_size = determine_page_size(project_id)

    while True:
        #response = requests.get(f"{base_url}/api/projects/{project_id}/tasks", headers=headers)
        response = requests.get(f"{base_url}/api/projects/{project_id}/tasks?page={page}", headers=headers)
        if response.status_code != 200:
            print(f"Failed to retrieve tasks on page {page}. Status code: {response.status_code}")
            break

        page_data = response.json()
        tasks.extend(page_data)
        num_fetched_tasks += len(page_data)

        # Check if there are more pages
        if not page_data or len(page_data) < page_size:
            break
        if limit is not None:
            if num_fetched_tasks >= limit:
                tasks = tasks[:num_fetched_tasks]
                break

        page += 1

    return tasks

project_id = 6 # can be obtained with previous call
tasks = get_all_tasks(project_id)
print(f"Total tasks: {len(tasks)}")

for task_dict in tasks[:10]:
    #print(task_dict) # dict
    task_json = json.dumps(task, indent=4) # nice formatting
    print(formatted_task)

## -- 3. Programmatically annotate tasks in bulk

# Fetch (all) tasks for the specified project
# Note: limited to the first 90 tasks!
project_id = 6 # can be obtained with previous call
tasks = get_all_tasks(project_id, limit=90)

# Define the list of flower classes
flower_classes = ['daisy', 'dandelion', 'rose', 'sunflower', 'tulip']

# Iterate through each task and update its annotations
for task in tasks:
    task_id = task['id']
    cluster = task['data']['cluster']

    # Filter by cluster, if desired
    if cluster == 1:
        # Create a random annotation for the task
        annotation_class = random.choice(flower_classes)
        payload = {
            "result": [{
                "from_name": "class",
                "to_name": "image",
                "type": "choices",
                "value": {"choices": [annotation_class]}
            }],
            "last_action": "prediction",
            "task": task_id,
            "project": project_id
        }
        
        response = requests.post(f"{base_url}/api/tasks/{task_id}/annotations", headers=headers, json=payload)
        if response.status_code != 201:
            print(f"Failed to update task {task_id}. Status code: {response.status_code}")
            print(response.text)
        else:
            print(f"Successfully updated task {task_id} with annotation: {annotation_class}")
```

### Basic SDK Usage

Apart from the REST API, Label Studio provides a Python SDK which can be integrated into our projects.

We need to install the SDK as follows:

```bash
pip install label-studio-sdk
```

Important links:

- Main source for this section: [Backend SDK Python Tutorial](https://labelstud.io/guide/sdk)
- Full [SDK reference](https://labelstud.io/sdk/index.html)

We can do many things with the SDK; here I summarize two important topics:

- Create a project
- Add/Import tasks: empty of pre-annotated

Another interesting use-case are filters, not covered here; for more information, check [prepare and manage data with filters](https://labelstud.io/guide/sdk#Prepare-and-manage-data-with-filters).

Note that the SDK has 6 submodules in total:

    label_studio_sdk.client
    label_studio_sdk.data_manager
    label_studio_sdk.project
    label_studio_sdk.users
    label_studio_sdk.utils
    label_studio_sdk.workspaces

First of all, after the SDK is installed, we need to connect to the Label Studio server:

```python
# Import the SDK and the client module
from label_studio_sdk import Client

# Access the LABEL_STUDIO_API_TOKEN
LABEL_STUDIO_API_TOKEN = os.getenv("LABEL_STUDIO_API_TOKEN")
LABEL_STUDIO_URL = 'http://localhost:8080'

# Connect to the Label Studio API and check the connection
ls = Client(url=LABEL_STUDIO_URL, api_key=LABEL_STUDIO_API_TOKEN)
ls.check_connection() # {'status': 'UP'}
```

#### Create a Project

```python
# Create a project with a template
# More templates: https://labelstud.io/templates
# After running the code, check the new project in the web UI
project = ls.start_project(
    title='Flowers 2',
    label_config='''
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
    '''
)
```

#### Add/Import Tasks: Empty of Pre-Annotated

In Label Studio, tasks are *imported*.
We follow the [Label Studio JSON format](https://labelstud.io/guide/tasks#Basic-Label-Studio-JSON-format), but as Python objects.

```python
project.import_tasks(
    [
        {'image_path': 'http://localhost:8000/flowers/test/Image_1.jpg'},
        {'image_path': 'http://localhost:8000/flowers/test/Image_10.jpg'}
    ]
)
```

The field names should match the ones in the XML definition, I think:

- `image_path`
- `class`
- etc.

Usage examples:

```python
# Recall we already have a web server serving all images with URLs
print(image_paths[:5]) # ['http://localhost:8000/flowers/test/Image_1.jpg', 'http://localhost:8000/flowers/test/Image_10.jpg', ...


# -- If we have a list of image URLs, we can programmatically add/import tasks
# Check the web UI to see the updates in there
project.import_tasks(
    [{'image_path': image_paths[i]} for i in range(10)]
)

# -- We can create predictions for tasks == pre-annotations
task_ids = project.get_tasks_ids()
project.create_prediction(task_ids[0], result='tulip', score=0.9)

# -- We can also create/import tasks with pre-annotations/predictions

flower_classes = ['daisy', 'dandelion', 'rose', 'sunflower', 'tulip']

project.import_tasks(
    [{'image_path': image_paths[i], 'class': random.choice(flower_classes),} for i in range(10,20)],
    preannotated_from_fields=['class']
)
```

### Machine Learning Backend

Label Studio has several ways of integrating a Machine Learning (ML) backend which predicts the labels (i.e., pre-annotation).

See these interesting links:

- [ML Integration: a ready Docker Image which is launched as an ML Backend](https://labelstud.io/guide/ml)
- [ML Tutorials: Many examples](https://labelstud.io/guide/ml_tutorials)
- [Writing custom ML backends](https://labelstud.io/guide/ml_create)
- [ML Backend Github examples](https://github.com/HumanSignal/label-studio-ml-backend)

I followed the tutorial [Create the simplest Machine Learning backend](https://labelstud.io/tutorials/dummy_model), which shows how to create and integrate a custom and simple image classification backend with a dummy model, i.e., a model which produces random predictions.

Let's consider the Flowers project, with this XML specification (the same as before):

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

We need to create a [`model.py`](./model.py) file which is then used by `label-studio-ml` to create a Docker project with all the necessary files:

```bash
# Install label-studio-ml in our environment
conda activate label
pip install --user label-studio-ml

# Create a ml_backend directory with the default model.py which contains the files to run a backend
cd path/to/model.py
label-studio-ml init ml_backend
# If the file is somewhere else
labal-studio-ml init ml_backend --script /path/to/my/script.py
```

The generated folder ml_backend/ contains:

    docker-compose.yml      redis container & ML backend server container
    Dockerfile              image definition which runs _wsgi using guinicorn
    model.py                copy of this file
    requirements.txt
    _wsgi.py                web server/API that runs the ML model

We can start the backend as follows:

```bash
# Launch in development model
label-studio-ml start my_backend
# The server started on http://localhost:9090 and outputs logs in console.

# Launch in production model
cd ml_backend/
docker-compose up

# Start Label Studio - by default in http://localhost:8080.
label-studio start
```

Then, we need to [add the ML backend to Label Studio](https://labelstud.io/guide/ml#Add-an-ML-backend-to-Label-Studio):


> - From Label Studio, open the project that you want to use with your ML backend.
> - Select Settings > Machine Learning.
> - Click Add Model.
> - Enter a title for the model and provide the URL for the ML backend. For example, http://localhost:9090.
> - (Optional) Enter a description.
> - (Optional) Select Allow version auto-update. See [Version auto-update](https://labelstud.io/guide/ml#Enable-auto-update-for-a-model) for more.
> - (Optional) Select Use for interactive preannotation. See Get interactive pre-annotations for more.
> - Click Validate and Save.

Also, we can [add the ML backend using the API](https://labelstud.io/api/#operation/api_ml_create).

The API allows to train the model, too; here are the URLs:

```bash
# Get ML backends (ids)
https://localhost:8080/api/ml?project={project_id}

# Train an ML backend
http://localhost:8080/api/ml/{id}/train
```

In the following, the content of [`model.py`](./model.py) is added:

```python
'''
This module contains a dummy ML model interface
which can be used to generate an ML backed for Label Studio.

I followed the tutorial in the link below, but made some changes:

    https://labelstud.io/tutorials/dummy_model

We can use a project with the following XML definition:

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

Other examples:

    https://github.com/HumanSignal/label-studio-ml-backend
    
    https://github.com/HumanSignal/label-studio-ml-backend/blob/master/label_studio_ml/examples/dummy_model/dummy_model.py
    
    https://github.com/HumanSignal/label-studio-ml-backend/blob/master/label_studio_ml/examples/the_simplest_backend/model.py
    
    https://github.com/HumanSignal/label-studio-ml-backend/blob/master/label_studio_ml/examples/simple_text_classifier/simple_text_classifier.py

Usage: Use this model.py to create an ml_backend folder with the
necessary files to run the ML backend

    conda activate label
    pip install --user label-studio-ml
    cd path/to/model.py
    label-studio-ml init ml_backend

The generated folder ml_backend/ contains:

    docker-compose.yml      redis container & ML backend server container
    Dockerfile              image definition which runs _wsgi using guinicorn
    model.py                copy of this file
    requirements.txt
    _wsgi.py                web server/API that runs the ML model

To start the ML backend

    label-studio-ml start .\ml_backend

The server started on http://localhost:9090 and outputs logs in console.

Read the README.md for more details.
'''

import os
import random
import requests
import json
from label_studio_ml.model import LabelStudioMLBase

from dotenv import load_dotenv
# Load .env file
load_dotenv()

LABEL_STUDIO_HOSTNAME = 'http://localhost:8080'
LABEL_STUDIO_API_TOKEN = os.getenv("LABEL_STUDIO_API_TOKEN", "token-value")

class DummyModel(LabelStudioMLBase):

    def __init__(self, **kwargs):
        # don't forget to call base class constructor
        super(DummyModel, self).__init__(**kwargs)
    
        # you can preinitialize variables with keys needed to extract info from tasks and annotations and form predictions
        from_name, schema = list(self.parsed_label_config.items())[0]
        self.from_name = from_name
        self.to_name = schema['to_name'][0]
        self.labels = schema['labels'] # classes specified in the XML
        
        # We can define/set the model
        self.model = None

    def _get_annotated_dataset(self, project_id):
        """Just for demo/example purposes:
           retrieve annotated data from Label Studio API.
           UNUSED in this model!
        """
        download_url = f'{LABEL_STUDIO_HOSTNAME.rstrip("/")}/api/projects/{project_id}/export'
        response = requests.get(download_url,
                                headers={'Authorization': f'Token {LABEL_STUDIO_API_TOKEN}'})
        if response.status_code != 200:
            raise Exception(f"Can't load task data using {download_url}, "
                            f"response status_code = {response.status_code}")
        return json.loads(response.content)
        
    def set_model(self, model):
        """Just for demo/example purposes:
           set a model instance from outside.
           UNUSED in this model!"""
        self.model = model
    
    def predict(self, tasks, **kwargs):
        """ This is where inference happens: model returns 
            the list of predictions based on input list of tasks 
        """
        predictions = []
        #flower_classes = ['daisy', 'dandelion', 'rose', 'sunflower', 'tulip']

        # Now, we could use the self.model to
        # predict the class of each task
        
        for task in tasks:
            #prediction_class = random.choice(flower_classes)
            prediction_class = random.choice(self.labels)
            predictions.append({
                'score': random.uniform(0.0, 1.0),  # prediction overall score, visible in the data manager columns
                'model_version': 'delorean-2023-10-02',  # all predictions will be differentiated by model version
                'result': [{
                    'from_name': self.from_name,
                    'to_name': self.to_name,
                    'type': 'choices',
                    #'score': 0.5,  # per-region score, visible in the editor 
                    'value': {
                        'choices': [prediction_class]
                    }
                }]
            })
        return predictions

    def fit(self, annotations, **kwargs):
        """ This is where training happens: train your model given list of annotations, 
            then returns dict with created links and resources.
            
            In some other examples, the function has the following definition:
                def fit(self, event, data, **kwargs)
            In those examples
            - event is not used
            - data is a dictionary with many information; it can be used as
            
                project_id = data['project']['id']
                tasks = self._get_annotated_dataset(project_id)
        """
        # In this function, may things should happen, not really done here:
        # - Pick the annotations, either from annotations of using _get_annotated_dataset()
        # - Pick the model, e.g., if self.model is defined, from there
        # - Extract the X (image vectors, tabular, etc.) and y (labels) vectors
        # - Train the model wit hits specific interface, e.g., self.model.fit(X,y)
        # - Save the model to disk, e.g. with pickle/joblib
        # - Pack the train_output dictionary with the data we want and return it
        
        train_output = {
            "model_path": "path/to/model.pkl"
        }
        
        return train_output

```

## Notes on Examples and Use-Cases

### Object Detection

:construction:

TBD.

### Semantic Segmentation

:construction:

TBD.
