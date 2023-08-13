# Pytorch Lightning

Pytorch Lightning uses Pytorch under the hood; however, the usual Pytorch boilerplate code is abstracted to a cleaner implementation similar to Keras. In that implementation, the code is structured in two major classes:

- `LightningModule`: this class takes the Pytorch model and expects the definition of some functions, such as:
  - `training_step()`
  - `validation_step()`
  - `test_step()`
  - `configure_optimizers()`, which returns the optimizer and the scheduler
  - `forward()`
  - `predict_step()`
- `Trainer`: this class takes the `LightningModule` and the datasets in the form of `Dataloaders` and is able to run these methods:
  - `fit()`
  - `test()`
  - `predict()`

In addition to those classes, we also need Pytorch `Dataloaders`.

Many nice operations are automatically performed via arguments/flags in the `Trainer` and minor definitions/calls in `LightningModule`:

- Epoch and batch iteration
- `optimizer.step()`, `loss.backward()`, `optimizer.zero_grad()` calls
- Calling of `model.eval()`, enabling/disabling grads during evaluation
- [Checkpoint Saving and Loading](https://lightning.ai/docs/pytorch/stable/common/checkpointing.html)
- Tensorboard logging (see [loggers](https://lightning.ai/docs/pytorch/stable/visualize/loggers.html) options)
- [Multi-GPU](https://lightning.ai/docs/pytorch/stable/accelerators/gpu.html) support
- etc.

Some additional resources:

- [Lightning Tutorials](https://www.pytorchlightning.ai/tutorials)
- [PyTorch Lightning Masterclass](https://www.youtube.com/playlist?list=PLaMu-SDt_RB5NUm67hU2pdE75j6KaIOv2)
- [UvA Deep Learning Tutorials](https://uvadlc-notebooks.readthedocs.io/en/latest/)
- [Lightning in 15 minutes](https://lightning.ai/docs/pytorch/stable/starter/introduction.html)
- [From PyTorch to PyTorch Lightning - A gentle introduction (by William Falcon, creator of Lightning)](https://towardsdatascience.com/from-pytorch-to-pytorch-lightning-a-gentle-introduction-b371b7caaf09)
- [Lightning in 15 Minutes](https://lightning.ai/docs/pytorch/stable/starter/introduction.html)
- [How to Organize PyTorch Into Lightning](https://lightning.ai/docs/pytorch/stable/starter/converting.html)
- Log histograms of weights: [Track and Visualize Experiments](https://lightning.ai/docs/pytorch/stable/visualize/logging_intermediate.html)
- Save checkpoints by condition: [Customize checkpointing behavior](https://lightning.ai/docs/pytorch/stable/common/checkpointing_intermediate.html)
- [Debugging](https://lightning.ai/docs/pytorch/stable/debug/debugging_basic.html)
- [Profiling](https://lightning.ai/docs/pytorch/stable/tuning/profiler_basic.html)
- [Deploy models into production with ONNX](https://lightning.ai/docs/pytorch/stable/deploy/production_advanced.html)
- [Customize the Trainer with Callbacks](https://lightning.ai/docs/pytorch/stable/extensions/callbacks.html)
- [Effective Training Tricks](https://lightning.ai/docs/pytorch/stable/advanced/training_tricks.html)
  - Accumulate gradient batches
  - Gradient clipping
  - Stochastic Weight Averaging
  - Batch Size Finder
  - Learning Rate Finder
  - N-Bit precision
    - [Basic](https://lightning.ai/docs/pytorch/stable/common/precision_basic.html)
    - [Intermediate](https://lightning.ai/docs/pytorch/stable/common/precision_intermediate.html)

## Setup and Installation

```bash
# Install/activate a basic environment
conda env create -f conda.yaml
conda activate ds
# Alternatively, if you have an env and wnat to install tensorboard
python -m pip install lightning

# OPTIONAL: Pytorch on Windows + CUDA 11.7
# Update your NVIDIA drivers: https://www.nvidia.com/Download/index.aspx
# I have version 12.1, but it works with older versions, e.g. 11.7
# Check your CUDA version with: nvidia-smi.exe
# In case of any runtime errors, check vrsion compatibility tables:
# https://github.com/pytorch/vision#installation
python -m pip install -U torch==1.13+cu117 torchvision==0.14+cu117 torchaudio torchtext==0.14 --index-url https://download.pytorch.org/whl/cu117
```

## Notebook: 01_Lightning_MNIST.ipynb

Alltogether, this is a summary of the contents implemented in the notebook [`01_Lightning_MNIST.ipynb`](./01_Lightning_MNIST.ipynb)

- The encoder/decoder models are created with Pytorch.
- A Lightning model is created with `LightningModule`; it has the most common methods:
  - `training_step()` with loss computation and logging
  - `validation_step()`
  - `test_step()`
  - `configure_optimizers()`, which returns the optimizer and the scheduler
  - `forward()`, if the model is called
  - `predict_step()` for using it in `Trainer().predict()`
- A `Trainer` is instantiated and `fit()` with:
  - `EarlyStopping` passed as a callback
  - `ModelCheckpoint` passed as a callback
  - Train and validation data loaders
- The model is tested with `Trainer().test()`
- The checkpoint is loaded and used:
  - The Pytorch model is used independently from Lightning
  - The `Trainer().predict()` interface is used to predict a dataset
- Training is resumed starting with a desired framework.
- Logs are visualized with Tensorboard
- Interesting `Trainer` tricks are shown
  - Learning rate finding
  - Accumulated gradient batching
  - Mixed precision
  - Gradient clipping

### 0. GPU Setup

```python
import os
import torch
import torchvision

print(torch.__version__)
# '1.13.0+cu117'

# Get info of all GPU devices
!nvidia-smi

# Set environment variable with possible device ids
os.environ["CUDA_VISIBLE_DEVICES"] = "0,1"
print(os.environ["CUDA_VISIBLE_DEVICES"])
# Set device: 0 or 1
# NOTE: indices are not necessarily the ones shown by nvidia-smi
# We need to try them with the cell below
torch.cuda.set_device("cuda:0")

# Check that the selected device is the desired one
print("Torch version?", torch.__version__)
print("Torchvision version?", torchvision.__version__)
print("Is cuda available?", torch.cuda.is_available())
print("Is cuDNN version:", torch.backends.cudnn.version())
print("cuDNN enabled? ", torch.backends.cudnn.enabled)
print("Device count?", torch.cuda.device_count())
print("Current device?", torch.cuda.current_device())
print("Device name? ", torch.cuda.get_device_name(torch.cuda.current_device()))
```

### 1. Define the Lightning Model

A LightningModule enables your PyTorch nn.Module to play together in complex ways inside the training_step (there is also an optional validation_step and test_step).

```python
import os
from torch import optim, nn, utils, Tensor
import torch.nn.functional as F
from torchvision.datasets import MNIST
from torchvision.transforms import ToTensor
import lightning.pytorch as pl

# ensure reproducibility
pl.seed_everything(42, workers=True)

# define any number of nn.Modules (or use your current ones)
# actually, we can define the model outside and then pass
# it to the LightningModule
# we can also do it with 1-liners
# encoder = nn.Sequential(nn.Linear(28 * 28, 64), nn.ReLU(), nn.Linear(64, 3))
# decoder = nn.Sequential(nn.Linear(3, 64), nn.ReLU(), nn.Linear(64, 28 * 28))

class Encoder(nn.Module):
    def __init__(self):
        super(Encoder, self).__init__()
        self.encoder = nn.Sequential(nn.Linear(28 * 28, 64), nn.ReLU(), nn.Linear(64, 3))
        
    def forward(self, x):
        return self.encoder(x)

class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()
        self.decoder = nn.Sequential(nn.Linear(3, 64), nn.ReLU(), nn.Linear(64, 28 * 28))
        
    def forward(self, x):
        return self.decoder(x)

# Optional class, not used
class AutoEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = Encoder()
        self.decoder = Decoder()

    def forward(self, x):
        x = x.view(x.size(0), -1) # in this case, the batch is reshaped to (B, 28*28)
        return self.decoder(self.encoder(x))    

# define the LightningModule
class LitAutoEncoder(pl.LightningModule):
    def __init__(self, encoder, decoder, learning_rate=0.001, batch_size=64):
        super().__init__()
        # the model can be defined here or outside
        self.encoder = encoder
        self.decoder = decoder
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        # OPTIONAL: automatically save all the hyperparameters passed to init,
        # but exclude things we don't want to save
        # NOTE: hyperparams saved to lightning_logs/version_X/hparams.yaml
        self.save_hyperparameters("learning_rate", "batch_size", ignore=["encoder", "decoder"])
        # then, hyperparams are also available as
        # self.hparams.learning_rate

    def training_step(self, batch, batch_idx):
        # training_step defines the train loop
        # it is independent of forward()
        x, y = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z) # x_hat is (B, 28*28)
        loss = F.mse_loss(x_hat, x)
        # Logging to TensorBoard (if installed) by default
        self.log("train_loss", loss)
        return loss

    def validation_step(self, batch, batch_idx):
        # this is the validation loop (optional)
        x, y = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        val_loss = F.mse_loss(x_hat, x)
        self.log("val_loss", val_loss)

    def test_step(self, batch, batch_idx):
        # this is the test loop (optional)
        x, y = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        test_loss = F.mse_loss(x_hat, x)
        self.log("test_loss", test_loss)

    def configure_optimizers(self):
        # intantiate return optimizer and learning rate schedulers
        optimizer = optim.Adam(self.parameters(), lr=self.learning_rate)
        lr_scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=70, gamma=0.1)
        return [optimizer], [lr_scheduler]
    
    def forward(self, x):
        x = x.view(x.size(0), -1) # in this case, the batch is reshaped to (B, 28*28)
        return self.decoder(self.encoder(x))
    
    def predict_step(self, batch, batch_idx, dataloader_idx=0):
        # this calls forward
        # this will be used in trainer.predict(...)
        # we can make this step more sophisticated, if desired
        # NOTE: we can also avoid the forward() call
        # and implement any desired predict step here!
        return self(batch[0]) # a batch is a list with one batch inside, so we take element 0


# init the autoencoder
encoder = Encoder() # nn.Sequential(nn.Linear(28 * 28, 64), nn.ReLU(), nn.Linear(64, 3))
decoder = Decoder() # nn.Sequential(nn.Linear(3, 64), nn.ReLU(), nn.Linear(64, 28 * 28))
autoencoder = LitAutoEncoder(encoder, decoder)
```

### 2. Define a Dataset

Lightning supports ANY iterable (DataLoader, numpy, etc…) for the train/val/test/predict splits.

```python
# setup data
train_set = MNIST(os.path.join(os.getcwd(), "data"), download=True, transform=ToTensor())
test_set = MNIST(os.path.join(os.getcwd(), "data"), download=True, train=False, transform=ToTensor())
# use 20% of training data for validation
train_set_size = int(len(train_set) * 0.8)
valid_set_size = len(train_set) - train_set_size
# split the train set into two
seed = torch.Generator().manual_seed(42)
train_set, valid_set = utils.data.random_split(train_set, [train_set_size, valid_set_size], generator=seed)

# loaders
# we can/should define batch_size as a hyperparameter in the LightningModule to be able to tune it
# but the trainer.text() which uses the test_loader does not use the hyperparameter
# HOWEVER, the recommended way is to tune/find the batch size and then instantiate the loaders with a hard-coded value
train_loader = utils.data.DataLoader(train_set) # , batch_size=64)
valid_loader = utils.data.DataLoader(valid_set) #, batch_size=64)
test_loader = utils.data.DataLoader(test_set, batch_size=64)
```

### 3. Train the Model

The Lightning [Trainer](https://lightning.ai/docs/pytorch/stable/common/trainer.html) “mixes” any [LightningModule](https://lightning.ai/docs/pytorch/stable/common/lightning_module.html) with any dataset and abstracts away all the engineering complexity needed for scale.

The Lightning [Trainer](https://lightning.ai/docs/pytorch/stable/common/trainer.html) automates [40+ tricks](https://lightning.ai/docs/pytorch/stable/common/trainer.html#trainer-flags) including:

- Epoch and batch iteration
- `optimizer.step()`, `loss.backward()`, `optimizer.zero_grad()` calls
- Calling of `model.eval()`, enabling/disabling grads during evaluation
- [Checkpoint Saving and Loading](https://lightning.ai/docs/pytorch/stable/common/checkpointing.html)
- Tensorboard logging (see [loggers](https://lightning.ai/docs/pytorch/stable/visualize/loggers.html) options)
- [Multi-GPU](https://lightning.ai/docs/pytorch/stable/accelerators/gpu.html) support
- [TPU](https://lightning.ai/docs/pytorch/stable/accelerators/tpu.html)
- [16-bit precision](https://lightning.ai/docs/pytorch/stable/advanced/speed.html#speed-amp) AMP support

One of the most important functionalities to improve our training is [early stopping](https://lightning.ai/docs/pytorch/stable/common/early_stopping.html), which is implemented with callbacks.

Additionally, we can also use the `ModelCheckpoint` callback to save both the last and the best model in our specified folder.

```python
from lightning.pytorch.callbacks.early_stopping import EarlyStopping

early_stop_callback = EarlyStopping(
    monitor="val_loss", # must be saved with self.log() in the LightningModule
    min_delta=0.00, # minimum change in the monitored quantity to qualify as an improvement
    patience=3,
    verbose=False,
    mode="min" # or max if val_accuracy
)

from lightning.pytorch.callbacks import ModelCheckpoint

# NOTE: if no ModelCheckpoint is specified, after each epoch the model checkpoint
# is saved in ./lightning_logs/version_X/checkpoints
checkpoint_callback = ModelCheckpoint(
    monitor='val_loss', # which metric to monitor
    dirpath='./output_mnist',  # specify the path to save the checkpoints
    filename='best',  # best.ckpt - we can also use naming formats: {epoch}-{val_loss:.2f}
    save_last=True,  # ensures that the last epoch's model is saved: last.ckpt
    mode='min',  # save the model with the minimum 'val_loss'
)

# train the model (hint: here are some helpful Trainer arguments for rapid idea iteration)
# if GPU is available, we can invoke it with the accelerator flag
# the checkpoint and the logs are saved to lightining_logs/version_X
trainer = pl.Trainer(
    # OPTIONAL arguments
    limit_train_batches=100,
    limit_val_batches=100,
    limit_test_batches=100,
    max_epochs=1,
    accelerator="gpu", # "mps" for Apple Silicon! Metal Performance Shaders
    devices=1, # in case we have several GPUs
    callbacks=[early_stop_callback]
    # if we want to disable saving / checkpointing
    # enable_checkpointing=False
    # if we want to control the directory where the checkpoint is saved
    # default_root_dir="some/path/"
)
trainer.fit(
    model=autoencoder,
    train_dataloaders=train_loader,
    val_dataloaders=valid_loader
    # if we want to resume training where we left
    # ckpt_path="some/path/to/my_checkpoint.ckpt"
)
```

### 4. Test the Model

```python
# test the model
trainer.test(model=autoencoder,
             dataloaders=test_loader)
```

### 5. Load Checkpoint and Use the Model

Once we’ve trained the model you can export to onnx, torchscript and put it into production or simply load the weights and run predictions.

```python
# load checkpoint as Lightning object
# check logs path for correct version_X and checkpoint
checkpoint = "./lightning_logs/version_19/checkpoints/epoch=0-step=100.ckpt"
lit_autoencoder = LitAutoEncoder.load_from_checkpoint(checkpoint, encoder=Encoder(), decoder=Decoder())

# we can also load the checkpoint with torch
# many things are saved: 'state_dict', 'optimizer_states', 'hyper_parameters', etc.
checkpoint_torch = torch.load(checkpoint, map_location=lambda storage, loc: storage)
print(checkpoint_torch.keys())
print(checkpoint_torch["hyper_parameters"])

# choose your trained nn.Module
# having the model defined outside makes it easier to use it later
encoder = lit_autoencoder.encoder
encoder.eval()

# embed 4 fake images!
fake_image_batch = torch.rand(4, 28 * 28, device=lit_autoencoder.device)
embeddings = encoder(fake_image_batch)
print("⚡" * 20, "\nPredictions (4 image embeddings):\n", embeddings, "\n", "⚡" * 20)

# Also, it is possible to use the forward() and the predict_step() functions
# of the LitAutoencoder from the Trainer
# NOTE that predict_step() is called
trainer = pl.Trainer()
predictions = trainer.predict(model=lit_autoencoder, dataloaders=test_loader)

# Number of batches
len(predictions)

# Batch 0, all outputs from Autoencoder: (B, 28*28)
# NOTE that in this implementation the images were not resized to 28x28
predictions[0].shape
```

### 6. Visualize Logs

```python
# %load_ext tensorboard
# %tensorboard --logdir ./lightning_logs
# http://localhost:6006/
```

#### Optional: Extract Tensorboard Logs

```python
from tensorboard.backend.event_processing import event_accumulator

# Path to the TensorBoard log directory
# NOTE that we check in the folder structure the metric we want...
#log_dir = "./runs/fashion_mnist_experiment_1/"
log_dir = './lightning_logs/version_21'
#log_dir = './runs/fashion_mnist_experiment_1/Training vs. Validation Loss_Validation/'

# Load the TensorBoard event files
event_acc = event_accumulator.EventAccumulator(log_dir)
event_acc.Reload()

# Print the list of events
print("Event Keys:", event_acc.Tags())

# Print the list of scalar tags
print("Scalar Tags:", event_acc.scalars.Keys()) # This output should be used

# Get the scalar events (train and validation losses)
metric = event_acc.Scalars('train_loss')

# Extract the loss values as Python lists
metric_values = [event.value for event in metric]

# Print the extracted loss values
print("Metric Values:", metric_values)
```

### 7. Re-Train or Resume Training

```python
#encoder = nn.Sequential(nn.Linear(28 * 28, 64), nn.ReLU(), nn.Linear(64, 3))
#decoder = nn.Sequential(nn.Linear(3, 64), nn.ReLU(), nn.Linear(64, 28 * 28))
encoder = Encoder()
decoder = Decoder()
autoencoder = LitAutoEncoder(encoder, decoder)

trainer = pl.Trainer(
    limit_train_batches=100,
    limit_val_batches=100,
    limit_test_batches=100,
    max_epochs=1,
    accelerator="gpu"
)

# automatically restores model, epoch, step, LR schedulers, etc...
trainer.fit(
    model=autoencoder,
    train_dataloaders=train_loader,
    val_dataloaders=valid_loader,
    ckpt_path="./lightning_logs/version_21/checkpoints/epoch=0-step=100.ckpt"
)
```

### 8. Trainer Tricks

See notebook:

- Automatic batch size finder
- Automatic learning rate finder
- Accumulated gradient batching
- Gradient clipping
- Precision mixing
- Others