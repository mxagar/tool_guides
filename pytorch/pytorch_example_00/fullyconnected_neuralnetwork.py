'''
    Basic functionalities to create a nerual network in Pytorch: fully connected, relu.
    Dataset must be provided for training.
    Loss function & optimization strategy can be defined.
    Save & load functions are also provided.
    Companion helper_nn.py helps visualize.

    Example of use with Fashion-MNIST dataset:

    # IMPORTS
    import matplotlib.pyplot as plt
    import torch
    from torch import nn
    from torch import optim
    import torch.nn.functional as F
    from torchvision import datasets, transforms
    import helper_nn as hnn
    import fullyconnected_neuralnetwork as fc_nn

    # LOAD DATASET: example, Fashion-MNIST (28x28 pixels, 1 channel, 10 classes)
    transform = transforms.Compose([transforms.ToTensor(),
                                    transforms.Normalize((0.5,), (0.5,))])
    trainset = datasets.FashionMNIST('../../../DL_PyTorch/F_MNIST_data/', download=True, train=True, transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=64, shuffle=True)
    testset = datasets.FashionMNIST('../../../DL_PyTorch/F_MNIST_data/', download=True, train=False, transform=transform)
    testloader = torch.utils.data.DataLoader(testset, batch_size=64, shuffle=True)

    # CHECK DATSET
    image, label = next(iter(trainloader))
    print(trainset.classes)
    hnn.imshow(image[0,:])
    # Have a clear idea of the image tensor shape
    # [Batch size, channels, width, height]
    image.shape

    # CREATE NETWORK
    #input_size = 1*28*28 = 728
    input_size = image.shape[2]*image.shape[3]
    #output_size = 10
    output_size = len(trainset.classes)
    # Select desired number of hidden layers and their sizes
    hidden_sizes = [512, 256, 128]
    model = fc_nn.Network(input_size, output_size, hidden_sizes)
    criterion = nn.NLLLoss() # Alternatives: nn.CrossEntropyLoss(), nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001) # Alternatives: optim.SGD()

    # TRAIN
    fc_nn.train(model, trainloader, testloader, criterion, optimizer, epochs=2)

    # SAVE
    filename = 'my_model_checkpoint.pth'
    fc_nn.save_model(filename, model, input_size, output_size, hidden_sizes)

    # LOAD
    filename = 'my_model_checkpoint.pth'
    model = fc_nn.load_model(filename)
    print(model)

    # INFER & VISUALIZE
    model.eval()
    images, labels = next(iter(testloader))
    img = images[0]
    #img = img.view(1, 28*28)
    img = img.view(1, images.shape[2]*images.shape[3]) # Note: visualization for one channel
    with torch.no_grad():
        output = model.forward(img)
    ps = torch.exp(output)
    hnn.view_classify(img.view(1, images.shape[2], images.shape[3]), ps, trainset.classes)

'''

import torch
from torch import nn
import torch.nn.functional as F

class Network(nn.Module):
    def __init__(self, input_size, output_size, hidden_layers, drop_p=0.5):
        ''' Builds a feedforward network with arbitrary hidden layers.
            All layers are linear (fully connected).
            Activation function: ReLU.
            Output type: log-softmax (needs exp() for obtaining probabilities).
            Dropout probabilty can be optionally specified.
            Arguments
            ---------
            input_size: integer, size of the input layer
            output_size: integer, size of the output layer
            hidden_layers: list of integers, the sizes of the hidden layers
            drop_p (optional): float, dropout probability of nodes 
        '''
        super().__init__()

        # Input to a hidden layer
        self.hidden_layers = nn.ModuleList([nn.Linear(input_size, hidden_layers[0])])

        # Add a variable number of more hidden layers
        layer_sizes = zip(hidden_layers[:-1], hidden_layers[1:])
        self.hidden_layers.extend([nn.Linear(h1, h2) for h1, h2 in layer_sizes])

        # Output layer
        self.output = nn.Linear(hidden_layers[-1], output_size)

        # Dropout function/layer
        self.dropout = nn.Dropout(p=drop_p)

    def forward(self, x):
        ''' Forward pass through the network, returns the output logits '''

        for each in self.hidden_layers:
            x = F.relu(each(x))
            x = self.dropout(x)
        x = self.output(x)

        return F.log_softmax(x, dim=1)  # Alternatives: F.softmax


def validation(model, testloader, criterion):
    accuracy = 0
    test_loss = 0

    # Check if CUDA GPU available
    # with this check, we obatin if we have a CUDA-compatible GPU
    # if so, we just need to transfer .to(device) every time we have a new
    # - model
    # - images or labels batch extracted from dataloader
    # if data already in device, nothing is done
    #device_str = "cuda:0" if torch.cuda.is_available() else "cpu"
    device_str = "cpu"
    device = torch.device(device_str)
    model.to(device)

    for images, labels in testloader:

        # Transfer to CUDA device if available
        images, labels = images.to(device), labels.to(device)

        # pixels = channels x width x height
        pixels = images.size()[1]*images.size()[2]*images.size()[3]
        # batch, pixels
        images = images.resize_(images.size()[0], pixels)

        output = model.forward(images)
        test_loss += criterion(output, labels).item()

        ## Calculating the accuracy
        # Model's output is log-softmax, take exponential to get the probabilities
        ps = torch.exp(output)
        # Class with highest probability is our predicted class, compare with true label
        equality = (labels.data == ps.max(1)[1])
        # Accuracy is number of correct predictions divided by all predictions, just take the mean
        accuracy += equality.type_as(torch.FloatTensor()).mean()

    return test_loss, accuracy


def train(model, trainloader, testloader, criterion, optimizer, epochs=5, print_every=40):

    # Check if CUDA GPU available
    # with this check, we obatin if we have a CUDA-compatible GPU
    # if so, we just need to transfer .to(device) every time we have a new
    # - model
    # - images or labels batch extracted from dataloader
    # if data already in device, nothing is done 
    #device_str = "cuda:0" if torch.cuda.is_available() else "cpu"
    device_str = "cpu"
    device = torch.device(device_str)
    model.to(device)
    print("Training on "+device_str)

    steps = 0
    running_loss = 0
    for e in range(epochs):
        # Model in training mode, dropout is on
        model.train()
        for images, labels in trainloader:
            steps += 1

            # Transfer to CUDA device if available
            images, labels = images.to(device), labels.to(device)

            # Flatten images into a channels x rows x cols long vector (784 in MNIST 28x28 case)
            pixels = images.size()[1]*images.size()[2]*images.size()[3]
            # batch, pixels
            images.resize_(images.size()[0], pixels)

            # Zero/initialize gradients
            optimizer.zero_grad()

            # Forward pass, then compute loss and differentiate it
            output = model.forward(images)
            loss = criterion(output, labels)
            loss.backward()
            optimizer.step()

            # Aggregate/sum loss
            running_loss += loss.item()

            if steps % print_every == 0:
                # Model in inference mode, dropout is off
                model.eval()

                # Turn off gradients for validation, will speed up inference
                with torch.no_grad():
                    test_loss, accuracy = validation(model, testloader, criterion)

                print("Epoch: {}/{}.. ".format(e+1, epochs),
                      "Training Loss: {:.3f}.. ".format(
                          running_loss/print_every),
                      "Test Loss: {:.3f}.. ".format(test_loss/len(testloader)),
                      "Test Accuracy: {:.3f}".format(accuracy/len(testloader)))

                # NOTE: we should track loss and accuracy and stop if necessary, eg
                # when d(abs(loss-accuracy))/d(epoch)>0 -> we're overfitting, stop!

                running_loss = 0

                # Make sure dropout and grads are on for training
                model.train()


def save_model(filename, model, input_size, output_size, hidden_sizes):
    # Convert model into a dict: architecture params (layer sizes) + state (weight & bias values)
    checkpoint = {'input_size': input_size,
                  'output_size': output_size,
                  'hidden_layers': hidden_sizes,
                  'state_dict': model.state_dict()}
    torch.save(checkpoint, filename)


def load_model(filepath):
    # Load saved dict
    checkpoint = torch.load(filepath)
    # Crate a model with given architecture params (layer sizes) + model state (weight & bias values)
    model = Network(checkpoint['input_size'],
                    checkpoint['output_size'],
                    checkpoint['hidden_layers'])
    model.load_state_dict(checkpoint['state_dict'])

    return model
