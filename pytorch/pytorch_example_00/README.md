# Basic Usage of Pytorch

This project collects the very basic functionalities of Pytorch.

In order to use these examples, we need to install Pytorch in out environment (eg., by using Anaconda):

``` bash
conda create -n myenv python=3.6
source activate myenv
conda install opencv-python matplotlib numpy pillow jupyter scipy pandas
conda install pytorch torchvision -c pytorch
````

Files and content:
- `tensors.py`: basic tensor usage.
- `helper_nn.py`: utils for model testing and visualization of dataset & inferred values
- `fullyconnected_neuralnetwork.py`: 
    - class `Network` created, fully connected (linear), relu, dropout (optional), desired hidden layers with their sizes
    - `validation` function for test split validation during training
    - `train` function
    - `save_model` function; `*.pth` file is saved, with architecture information & weight states
    - `load_model` function
- `application_notebook.ipynb`: notebook in which `fullyconnected_neuralnetwork.py` and `helper_nn.py` are used to train a model (defined in the notebook) with Fashion-MNIST dataset and infer some images