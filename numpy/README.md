# Numpy

[Numpy](https://numpy.org/doc/stable/index.html) is the go-to library when you'd like to use arrays in Python. It's implemented in C, so it's very fast, and easy to use. But which are the best practices? I'll try answer that question in this guide.

I follow the official documentation: [NumPy user guide](https://numpy.org/doc/stable/user/index.html).

Altogether, 3 notebooks are contained in here, with the following contents:

- [`01_UserTutorial.ipynb`](01_UserTutorial.ipynb): complete (beginner-intermmediate) tutorial based on [NumPy quickstart](https://numpy.org/doc/stable/user/quickstart.html):
  - 1. Basics
    - 1.1 Array creation: `np.array(), np.zeros(), np.ones(), np.arange(), n.random.random()`
    - 1.2 Array operations: `+, -, *, *, @, np.dot(), np.sum(), np.min(), np.cumsum()`
    - 1.3 Universal functions: `np.frompyfunc()`
    - 1.4 Indexing, slicing, iterating
  - 2. Shape manipulation
    - 2.1. Changing the shape: `reshape()`, `ravel()`, `resize()`
    - 2.2. Stacking and splitting arrays: `np.vstack()`, `np.hstack()`, `np.vsplit()`, `np.hsplit()`
  - 3. Copies and views
    - 3.1. No copy at all
      - Assignments make no copies!
      - Passing an array (mutable) to a function is done by reference, i.e., no copies!
    - 3.2 Views or shallow copies: Different array objects can share the same data, i.e., the memory buffer containing the data is the same for all of them
      - Views:
        - Slicing an array returns a view of it!
        - `reshape()`
        - `ravel()`
        - In some cases, `hsplit()` and `vsplit()`
      - Not views, but copies or new data in memory:
        - `resize()`
        - `hstack()`, `vstack()`
    - 3.3. Deep copies: `copy()`
  - 4. [Functions and Methods Overview](https://numpy.org/doc/stable/user/quickstart.html#functions-and-methods-overview)
  - 5. Advanced indexing and index tricks
    - 5.1 Indexing with Arrays of Indices
    - 5.2 Indexing with Boolean Arrays
- [`02_Advanced.ipynb`](02_Advanced.ipynb): more advanced or deepened topics picked from [NumPy fundamentals](https://numpy.org/doc/stable/user/basics.html)
- [`03_PerformanceTests.ipynb`](03_PerformanceTests.ipynb): personal performance and other tests.

So select the topic and open the corresponding notebook.

Mikel Sagardia, 2024.  
No guarantees.  
