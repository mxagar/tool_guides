# ONNX: Open Neural Network Exchange

Some sources:

- [ONNX Tutorials](https://github.com/onnx/tutorials)
- [Youtube: ONNX Tutorial](https://www.youtube.com/watch?v=BEXQS6_YB8A&list=PLkz_y24mlSJZJx9sQJCyFZt50S4ji1PeR)

Table of contents:

- [ONNX: Open Neural Network Exchange](#onnx-open-neural-network-exchange)
  - [Setup](#setup)
  - [Vanilla Example: Scikit-Learn](#vanilla-example-scikit-learn)
  - [ONNX Runtime](#onnx-runtime)
  - [ONNX Model Zoo](#onnx-model-zoo)
  - [Example: Convert a Pytorch Model into Tensorflow](#example-convert-a-pytorch-model-into-tensorflow)

Key ideas:

- With ONNX we can convert models from and to different networks, e.g., `Pytorch <-> Tensorflow`.
  - We convert a model from framework A to ONNX and from ONNX to framework B.
  - Even though it was initially implemented for ANNs, it also supports *classical* ML models, e.g., from Scikit-Learn.
- We have also the **ONNX Runtime** module to run ONNX models.
  - ONNX models are also optimized, i.e., quantized (`float -> int`), so that it can run in devices with less capabilities than the ones used for training.
  - See the dedicated section: [ONNX Runtime](#onnx-runtime).
- ONNX closely tracks the framework updates
  - It is an organization maintained by the big ML players.
- We can visualize ONNX models as graphs.
- ONNX models have:
  - Operators: the usual suspects: `tanh()`, `relu()`
  - Types: also the usual suspects
    - Tensors: with elements of desired precision (`int8`, `float16`, etc.)
    - Non-sensor types: `Sequence`, `Map`

List of notebooks:

- [`01_Vanilla_Sklearn_ONNX.ipynb`](01_Vanilla_Sklearn_ONNX.ipynb)
- [`02_ONNX_Model_Zoo.ipynb`](02_ONNX_Model_Zoo.ipynb)
- [`03_Pytorch2Tensorflow.ipynb`](03_Pytorch2Tensorflow.ipynb)


## Setup

Each framework has its own ONNX module installation process.

In any case, we first need to install:

- ONNX: for model conversion.
- ONNX RUntime: if we want to run ONNX models.

```bash
# ONNX
pip install onnx
pip install onnxruntime
pip install onnxruntime-gpu # if GPU support is available

# Scikit-Learn Support
# https://github.com/onnx/sklearn-onnx
pip install skl2onnx

# Tensorflow Support
# https://github.com/onnx/tensorflow-onnx
pip install -U tf2onnx
git clone https://github.com/onnx/onnx-tensorflow.git && cd onnx-tensorflow && pip install -e .

# Pytorch Support
# It is built-in in torch
# https://pytorch.org/docs/master/onnx.html

# Other tools used in this guide
pip install scikit-learn
pip install skl2onnx
pip install joblib
pip install netron
pip install numpy
pip install opencv-python
pip install tensorflow-addons
pip install torch
pip install torchvision
```

## Vanilla Example: Scikit-Learn

Source: [entbappy/ONNX-Open-Neural-Network-Exchange/ML_example](https://github.com/entbappy/ONNX-Open-Neural-Network-Exchange/tree/master/ML_example).

This section is implemented in [`01_Vanilla_Sklearn_ONNX.ipynb`](./01_Vanilla_Sklearn_ONNX.ipynb).

Set environment:

```bash
conda activate ds

# If requirements are missing
pip install scikit-learn
pip install onnx
pip install skl2onnx
pip install joblib
pip install onnxruntime
pip install netron
pip install numpy
pip install opencv-python
pip install tensorflow-addons
pip install onnx_tf
```

Train and save pickle:

```python
from sklearn.datasets import load_iris
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
import joblib

iris = load_iris()
X,y = iris.data, iris.target
X_train, X_test, y_train, y_test = train_test_split(X,y,random_state=0)

clf = RandomForestClassifier()
clf.fit(X_train, y_train)
joblib.dump(clf, 'output/model_sklearn.pkl', compress=9)
```

Convert to ONNX:

```python
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType
import joblib

cls = joblib.load('output/model_sklearn.pkl')
initial_type = [('float_input', FloatTensorType([None, 4]))]
onx = convert_sklearn(cls, initial_types=initial_type)

with open('output/model_sklearn.onnx', 'wb') as f:
    f.write(onx.SerializeToString())
```

Visualize model with [netron](https://github.com/lutzroeder/Netron):

```python
import netron

# Start visualization server
# Browser opens to visualize the ONNX model
# http://localhost:8080/
# We can click as see what's inside!
netron.start('output/model_sklearn.onnx')

# Stop visualization server
netron.stop()
```

Inference: If we want to use the pickle to run the inference, we need the `sklearn` library. However, if we have exported to ONNX and we want to use the ONNX model for inference, we don't need the `sklearn` library, just ONNX (runtime).

```python
import onnxruntime as rt
import numpy as np

data = np.array([[4.5,4.9,5.1,5.4],[1.5,2.9,3.1,1.4],[7.5,6.9,8.1,6.4]])

sess = rt.InferenceSession("output/model_sklearn.onnx")
input_name = sess.get_inputs()[0].name
label_name = sess.get_outputs()[0].name

pred_onnx = sess.run([label_name], {input_name: data.astype(np.float32)})[0]
print(pred_onnx)
```

## ONNX Runtime

Key ideas:

- ONNX Runtime was founded by Microsoft and is offered under the MIT license.
- ONNX Runtime builds up on ONNX.
- Specific inference hardware types are handled seamlessly: CPU, GPU, etc.; thus, we don't have to worry about the HW.
- The ONNX models run very fast using ONNX runtime.

Official site documentation: [Get started with ONNX Runtime in Python](https://onnxruntime.ai/docs/get-started/with-python.html).

## ONNX Model Zoo

Similarly as in DL frameworks, we have models with pre-trained weights which can be downloaded and used.

List of all models: [Model Zoo](https://github.com/onnx/models); we have models for many tasks:

- Image classification
- Object detection
- Face and gesture detection
- Style transfer
- Speech and audio processing
- Machine translation
- Language modelling
- etc.

Example with MNIST:

- Source: [entbappy/ONNX-Open-Neural-Network-Exchange/ML_example](https://github.com/entbappy/ONNX-Open-Neural-Network-Exchange/tree/master/ML_example).
- Implemented in [`02_ONNX_Model_Zoo.ipynb`](02_ONNX_Model_Zoo.ipynb).

```python
# %%
# Download the model
%%sh
curl -k -o mnist.tar.gz https://www.cntk.ai/OnnxModels/mnist/opset_7/mnist.tar.gz

# %%
# Uncompress the model
%%sh
tar -xzf mnist.tar.gz
# in the folder mnist, we'll find the model.onnx

# %%
# For the inference
# we need hand-written digits;
# these can be downloaded from the source example repository, for instance.
import sys
import json
import cv2
import numpy as np
import onnx
import onnxruntime
from onnx import numpy_helper

model_dir = './mnist' # model directory
model = model_dir + '/model.onnx' # model file
image_path = "./data/mnist_3.png" # image path
#image_path = "./data/mnist_7.png" # image path

# Preprocessing the image
img = cv2.imread(image_path)
# Extract RGB (not alpha) and convert to grayscale with weighted channel values
img = np.dot(img[...,:3], [0.299, 0.587, 0.114])
img = cv2.resize(img, (28, 28), interpolation=cv2.INTER_AREA)
img.resize((1,1,28,28)) # batch of image

data = json.dumps({"data": img.tolist()})
data = np.array(json.loads(data)["data"]).astype(np.float32)
session = onnxruntime.InferenceSession(model, None)
input_name = session.get_inputs()[0].name
ouput_name = session.get_outputs()[0].name

result = session.run([ouput_name], {input_name: data})
prediction = int(np.argmax(np.array(result).squeeze(), axis=0))
print("Predicted: ",prediction)
```

## Example: Convert a Pytorch Model into Tensorflow

Sources:

- [Convert a PyTorch model to Tensorflow using ONNX](https://github.com/onnx/tutorials/blob/main/tutorials/PytorchTensorflowMnist.ipynb)
- [Pytorch to Tensorflow](https://github.com/entbappy/ONNX-Open-Neural-Network-Exchange/blob/master/Pytorch_to_Tensoflow/PyTorch%20to%20Tensorflow.ipynb)

Unfortunately, the inference with Tensorflow doesn't work, because the seems to be a miss-match between the expected and actual input node names. I don't have t ime to fix this at the moment...

```python
# Install the specific package for ONNX-Tensorflow support
# Restart kernel
# !git clone https://github.com/onnx/onnx-tensorflow.git && cd onnx-tensorflow && pip install -e .
# Or, altenatively
# !pip install onnx_tf

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms
from torch.autograd import Variable

import onnx
from onnx_tf.backend import prepare

# %%
# Pytorch Model Definition

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 10, kernel_size=5)
        self.conv2 = nn.Conv2d(10, 20, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.fc1 = nn.Linear(320, 50)
        self.fc2 = nn.Linear(50, 10)

    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
        x = x.view(-1, 320)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        return F.log_softmax(x, dim=1)

def train(model, device, train_loader, optimizer, epoch):
    model.train()
    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()
        output = model(data)
        loss = F.nll_loss(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx % 1000 == 0:
            print('Train Epoch: {} \tLoss: {:.6f}'.format(
                    epoch,  loss.item()))

def test(model, device, test_loader):
    model.eval()
    test_loss = 0
    correct = 0
    with torch.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            test_loss += F.nll_loss(output, target, reduction='sum').item() # sum up batch loss
            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            correct += pred.eq(target.view_as(pred)).sum().item()

    test_loss /= len(test_loader.dataset)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
        test_loss, correct, len(test_loader.dataset),
        100. * correct / len(test_loader.dataset)))

# %%
# MNIST Dataset

train_loader = torch.utils.data.DataLoader(
    datasets.MNIST('./data', train=True, download=True,
                   transform=transforms.Compose([
                       transforms.ToTensor(),
                       transforms.Normalize((0.1307,), (0.3081,))
                   ])),
    batch_size=64, shuffle=True)

test_loader = torch.utils.data.DataLoader(
    datasets.MNIST('./data', train=False, transform=transforms.Compose([
                       transforms.ToTensor(),
                       transforms.Normalize((0.1307,), (0.3081,))
                   ])),
    batch_size=1000, shuffle=True)

# %%
# Training

torch.manual_seed(1)
#device = torch.device("cuda")
device = torch.device("cpu")

model = Net().to(device)
optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.5)
 
for epoch in range(2):
    train(model, device, train_loader, optimizer, epoch)
    test(model, device, test_loader)

# %%
# Save and Load Pytorch Model

# Save
torch.save(model.state_dict(), './output/model_mnist_pytorch.pth')

# Load
trained_model = Net()
trained_model.load_state_dict(torch.load('./output/model_mnist_pytorch.pth'))

# %%
# Export to ONNX

dummy_input = Variable(torch.randn(1, 1, 28, 28)) 
torch.onnx.export(trained_model, dummy_input, "./output/model_mnist_pytorch.onnx")

# Load the ONNX file
model = onnx.load('./output/model_mnist_pytorch.onnx')

# Visualize
import netron

# Start visualization server
# Browser opens to visualize the ONNX model
# http://localhost:8080/
# We can click as see what's inside!
netron.start('output/model_mnist_pytorch.onnx')
# ...
netron.stop()

# %%
# Inference with ONNX
import onnxruntime as rt
import numpy as np
from PIL import Image

#img = Image.open('./data/mnist_3.png').resize((28, 28)).convert('L')
img = Image.open('./data/mnist_7.png').resize((28, 28)).convert('L')
data = np.asarray(img, dtype=np.float32)[np.newaxis, np.newaxis, :, :]
print(data.shape)

sess = rt.InferenceSession('output/model_mnist_pytorch.onnx')
input_name = sess.get_inputs()[0].name # it is called input.1
label_name = sess.get_outputs()[0].name # it is called 20

pred_onnx = sess.run([label_name], {input_name: data})[0][0]
print(np.argmax(pred_onnx))

# %%
# Convert ONNX to Tensorflow

# Import the ONNX model to Tensorflow
tf_rep = prepare(model)

# Input nodes to the model
print('inputs:', tf_rep.inputs)

# Output nodes from the model
print('outputs:', tf_rep.outputs)

# All nodes in the model
print('tensor_dict:')
print(tf_rep.tensor_dict)

# %%
# Inference with Tensorflow
# This part doesn't work in my setting: "KeyError: 'input.1'"

import numpy as np
from IPython.display import display
from PIL import Image

print('Image 1:')
img = Image.open('./data/mnist_3.png').resize((28, 28)).convert('L')
display(img)
output = tf_rep.run(np.asarray(img, dtype=np.float32)[np.newaxis, np.newaxis, :, :])
print('The digit is classified as ', np.argmax(output))

print('Image 2:')
img = Image.open('./data/mnist_7.png').resize((28, 28)).convert('L')
display(img)
output = tf_rep.run(np.asarray(img, dtype=np.float32)[np.newaxis, np.newaxis, :, :])
print('The digit is classified as ', np.argmax(output))

```