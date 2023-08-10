# PyTorch TensorBoard Support

The co-located notebook [`pytorch_tensorboard.ipynb`](./pytorch_tensorboard.ipynb) shows how to use Tensorboard with Pytorch.

In this `README.md`, only the Tensorflow-specific lines are highlighted, whereas the notebook contains a complete CNN training and evaluation with the [Fashion-MNIST](https://github.com/zalandoresearch/fashion-mnist) dataset.

This notebook is based on the following tutorials and sites:

- [PyTorch TensorBoard Support](https://pytorch.org/tutorials/beginner/introyt/tensorboardyt_tutorial.html)
- [Visualizing Models, Data, and Training with TensorBoard](https://pytorch.org/tutorials/intermediate/tensorboard_tutorial.html)
- [Documentation: torch.utils.tensorboard](https://pytorch.org/docs/stable/tensorboard.html?highlight=summarywriter)
- [How to use TensorBoard with PyTorch](https://github.com/christianversloot/machine-learning-articles/blob/main/how-to-use-tensorboard-with-pytorch.md)

Contents:

- Save sampled experiment images.
- Log training metrics:
  - Train and validation loss.
  - Weight histograms
- Visualize model graph.
- Visualize embeddings (not working - check [`../embeddings/`](../embeddings/) instead)
- Model evaluation: PR curves.
- Get a python list of values logged to Tensorboard.

The overall idea is to instantiate a `writer = SummaryWriter(...)` with a `logdir` and to add elements to it wherever and whenever we need, as explained in the [documentation site of torch.utils.tensorboard](https://pytorch.org/docs/stable/tensorboard.html?highlight=summarywriter):

- `write.add_scalar(...)`
- `write.add_scalars(...)`
- `write.add_histogram(...)`
- `write.add_image(...)`
- `write.add_images(...)`
- `write.add_figure(...)`
- `write.add_video(...)`
- `write.add_audio(...)`
- `write.add_text(...)`
- `write.add_graph(...)`
- `write.add_embedding(...)`
- `write.add_pr_curve(...)`
- `write.add_custom_scalars(...)`
- `write.add_mesh(...)`
- `write.add_hparams(...)`

Then, we do `write.flush()` to write to disk and finally `writer.close()`.

Note that `tensorboard` can be launched and interactively consulted as we add data to it!

```bash
# Don't use blank spaces
tensorboard --logdir=./
# Open borwser at http://localhost:6006/
# Refresh it several times until it works
```

We need to select the approproate tab for display.

Also, note that the dashboard can be open during training, so that the training metrics are monitored in realtime.

Notebook summary:

```python
# -- PyTorch TensorBoard support
from torch.utils.tensorboard import SummaryWriter

# -- Load Dataset
# and get an img_grid of shape
# torch.Size([3, 32, 122]): 4 images of shape 32x32x3
# ...

# -- Log Images
# Default log_dir argument is "runs" - but it's good to be specific
# torch.utils.tensorboard.SummaryWriter is imported above
writer = SummaryWriter('runs/fashion_mnist_experiment_1')

# Write image data to TensorBoard log dir
writer.add_image('Four Fashion-MNIST Images', img_grid)
# When we flush, the data is written to disk
writer.flush()

# -- Start Tensorboard
# To view, start TensorBoard on the command line with:
#   tensorboard --logdir=runs
# on a CLI with the propper environment active.
# Open a browser tab to http://localhost:6006/
# and check the IMAGES tab

# -- Layer Weight Histograms

def weight_histograms_conv2d(writer, step, weights, layer_number):
    weights_shape = weights.shape
    num_kernels = weights_shape[0]
    for k in range(num_kernels):
        flattened_weights = weights[k].flatten()
        tag = f"layer_{layer_number}/kernel_{k}"
        writer.add_histogram(tag, flattened_weights, global_step=step, bins='tensorflow')


def weight_histograms_linear(writer, step, weights, layer_number):
    flattened_weights = weights.flatten()
    tag = f"layer_{layer_number}"
    writer.add_histogram(tag, flattened_weights, global_step=step, bins='tensorflow')


def weight_histograms(writer, step, model):
    # Create a list of layers
    layers_list = [
        model.conv1, model.pool, model.conv2,
        model.fc1, model.fc2, model.fc3
    ]

    # Loop over all layers
    for layer_number, layer in enumerate(layers_list):
        # Compute weight histograms for appropriate layer
        if isinstance(layer, nn.Conv2d):
            weights = layer.weight
            weight_histograms_conv2d(writer, step, weights, layer_number)
        elif isinstance(layer, nn.Linear):
            weights = layer.weight
            weight_histograms_linear(writer, step, weights, layer_number)

# -- Log Training Metrics

for epoch in range(10):
    for i, data in enumerate(training_loader):
        # ... -> train_loss
        if i % 1000 == 999:    # Validate every 1000 mini-batches...
            for j, vdata in enumerate(validation_loader):
                # ...
            # ... -> validation_loss

            # Log/Visualize weight histograms
            weight_histograms(writer,
                              epoch * len(training_loader) + i,
                              net)
    
            # Log the running loss averaged per batch
            writer.add_scalars(
                'Training vs. Validation Loss',
                { 'Training' : avg_loss, 'Validation' : avg_vloss },
                epoch * len(training_loader) + i
            )


# When we flush, the data is written to disk
writer.flush()

# -- Visualize the Model

# Again, grab a single mini-batch of images
dataiter = iter(training_loader)
images, labels = next(dataiter)

# add_graph() will trace the sample input through your model,
# and render it as a graph.
writer.add_graph(net, images)
writer.flush()

## -- Evaluation: Log PR Curves

# 1. gets the probability predictions in a test_size x num_classes Tensor
# 2. gets the preds in a test_size Tensor
# takes ~10 seconds to run
class_probs = []
class_label = []
with torch.no_grad():
    for data in validation_loader:
        images, labels = data
        output = net(images)
        class_probs_batch = [F.softmax(el, dim=0) for el in output]

        class_probs.append(class_probs_batch)
        class_label.append(labels)

test_probs = torch.cat([torch.stack(batch) for batch in class_probs])
test_label = torch.cat(class_label)

# helper function
def add_pr_curve_tensorboard(class_index, test_probs, test_label, global_step=0):
    '''
    Takes in a "class_index" from 0 to 9 and plots the corresponding
    precision-recall curve
    '''
    tensorboard_truth = test_label == class_index
    tensorboard_probs = test_probs[:, class_index]

    writer.add_pr_curve(classes[class_index],
                        tensorboard_truth,
                        tensorboard_probs,
                        global_step=global_step)
    writer.close()

# plot all the pr curves
for i in range(len(classes)):
    add_pr_curve_tensorboard(i, test_probs, test_label)


# -- Get a Python List of Values Logged to Tensorboard

# Probably there is a better way than the following,
# but I managed to get Python objects of the logged events/artifacts as shown below.
# Note that we need to first look into the folder structure and print the keys of the logs.
# Then, once the metric is detected, we extract it manually.

from tensorboard.backend.event_processing import event_accumulator

# Path to the TensorBoard log directory
# NOTE that we check in the folder structure the metric we want...
#log_dir = "./runs/fashion_mnist_experiment_1/"
log_dir = './runs/fashion_mnist_experiment_1/Training vs. Validation Loss_Training/'
#log_dir = './runs/fashion_mnist_experiment_1/Training vs. Validation Loss_Validation/'

# Load the TensorBoard event files
event_acc = event_accumulator.EventAccumulator(log_dir)
event_acc.Reload()

# Print the list of events
print("Event Keys:", event_acc.Tags())

# Print the list of scalar tags
print("Scalar Tags:", event_acc.scalars.Keys()) # This output should be used

# Get the scalar events (train and validation losses)
_loss_events = event_acc.Scalars('Training vs. Validation Loss')

# Extract the loss values as Python lists
_loss_values = [event.value for event in _loss_events]

# Print the extracted loss values
print("Loss Values:", _loss_values)
```