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

# Convert numpy array to pytorch tensor
# BUT: memory is shared between numpy a and pytorch b: changing one affects the other!
a = np.random.rand(4, 3)
b = torch.from_numpy(a)
