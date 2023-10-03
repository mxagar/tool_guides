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
