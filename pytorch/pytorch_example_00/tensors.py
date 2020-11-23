import torch
import numpy as np

# 3 rows, 2 cols
x = torch.rand(3, 2)
print(x)
y = torch.ones(x.size())
print (y)

# Element-wise sum
z = x + y
print(z)

# Access: First row
z[0]
# Slicing works as usual
z[:, 1:]
# Methods return NEW object: add 1 element-wise
z_new = z.add(1)
# Except when followed by _: then, z is changed
z.add_(1)
z.mul_(2)

# Convert numpy array to pytorch tensor, and vice-versa
# BUT: memory is shared between numpy a and pytorch b: changing one affects the other!
a = np.random.rand(4, 3)
b = torch.from_numpy(a)
c = b.numpy()

# Gradients are computed if we set requires_grad = True; this is necessary for training models (autograd)
# Gradients of a tensor are computed with .backward & obtained with .grad
# Therefore, during training, we explicitly need to
# - (activate) and zero gradients: optimizer.zero_grad()
# - compute gradients: loss.backward(); that's done after the forward pass operation: model.forward()
# and during inference, we need to deactivate gradient computing: with torch.no_grad()
x = torch.ones(1, requires_grad=True)
print(x.requires_grad)
f = x**2 # function definition
f.backward() # df
print(x.grad) # dx -> get df/dx -> 2

# Important operations on numpy arrays and pytorch tensors
# 1. numpy squeeze(): remove single-dimensional entries from the shape of an array
d = np.random.rand(1, 1, 2)
print(d.shape) # (1, 1, 2) -> we really have 2 random values, so we could remove the first to single dimensions
e = d.squeeze()
print(e.shape) # (2,)
# 2. numpy view(): it is a shallow copy: it is a new array that can have a new shape, but the underlying memory is the same ias in the org array
v = np.array([1,2,3,4,5,6])
print(v)
w = v.view().reshape((3, 2))
print(w)
# 3. pytorch resize_(): resize tensor re-arranging data; underlying data is preserved, but no new memory is initialized if new storage is needed! 
x = torch.tensor([[1, 2], [3, 4], [5, 6]])
x.resize_(2, 2)
print(x)
# 4. pytorch item(): get the value from a tensor containing a single value
x = torch.ones(1)
print(x)
print(x.item())
