import dummylib

# print an object
dummylib.print(33)

# call a function with a list (returns another list), with an optional named argument. The list in C++ becomes an std::vector
print(dummylib.mult([1, 2, 3], 2.0, all=True))

# instantiate objects of a class
circle = dummylib.Circle(10.0)

# Assign value to a property through a setter
circle.radius = 15.0

# Call a method
print(circle.area())
