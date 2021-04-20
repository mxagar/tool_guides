# Dummy example with pybind11

A sample of a wrapper of C++ code that can be imported and called from Python.
This example was created by Alvaro.

**IMPORTANT NOTE**: It works in a limited fashion for me, since I am having conflicts between python libraries used for compilation and execution. See below.

## Usage

Go to the default python environment and install:

`conda install -c conda-forge pybind11`

Get pybind11 code into `pybind11`; it is already there.

Compile this example:

```bash
make build
cd build
cmake ..
make
cd ..
```

Link the generated libraries:

```bash
ln -is build/dummylib.cpython-36m-darwin.so .
```

Run the `test.py` example, which uses the code generated and interfaced in `pymodule.cpp`.

**WARNING**: I am having issues with the different versions of the `python` library; it seems the compilation happens with python 3.6, and when I try to run `test.py` with another python version it crashes. This should be addressed by using `pybind11` correctly as explained in

[https://pybind11.readthedocs.io/en/stable/index.html](https://pybind11.readthedocs.io/en/stable/index.html)


Example file `./test.py`:
```python

import dummylib

# print an object
dummylib.print(33)

# call a function with a list (returns another list), with an optional named argument. The list in C++ becomes an std::vector
print( dummylib.mult([1, 2, 3], 2.0, all=True) )

# instantiate objects of a class
circle = vicdummylibomlib.Circle(10.0)

# Assign value to a property through a setter
circle.radius = 15.0

# Call a method
print(circle.area())

```