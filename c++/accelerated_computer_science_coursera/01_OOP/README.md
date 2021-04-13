# Object Oriented Data Structures in C++

Notes made when following the course offered in Coursera by University of Illinois at Urbana-Champaign.
Prof. Wade Fagen-Ulmschneider.

## Week 1: Introduction to Writing C++ Programs

### 1.1 Introduction

- C++ is strongly typed: each declared variable has type (memory size) + name + value in memory.
- Two types: primitive (int, bool, etc.) and user-defined (classes, containers, etc.).
- A program always needs `int main() { return 0; }`.
    - Return is 0 if OK, non-zero if program fails.
    - `main()` is the starting point
- Makefiles: compile source files into binary coode
    - `_make/generic.mk`: it contains the general makefile used by any local project `Makefile`
    - Command: `make`

### 1.2 C++ Classes

- Encapsulation: Data and Functionality are separated and eclosed into a single unity
- Member variables with `_` at the end: `length_`
- Member functions with camelCase: `getLength()`
- Layers of protection:
    - public: anyone can access it
    - private: only members
- Encapsulation is also achieved with header files and source code files
    - Header: API: `*.h`
    - Source: The implementation goes here: `*cpp`
- Header files: declaration must be only once!
    - Add: `#pragma once`

`cpp-class/Cube.h`: the API
```c++
#pragma once // compile this declaration/file once
class Cube {
    public:
        double getLength();
        double getVolume();
        void setLength(double length);
    private:
        double length_;
}; // always end declarations with ;
```

`cpp-class/Cube.cpp`: the implementation
```c++
#include "Cube.h"
double Cube::getLength() {
    return length_;
}
double Cube::getVolume() {
    return (length_ * length_ * length_);
}
void Cube::setLength(double length) {
    length_ = length
}
```

`cpp-class/main.cpp`: the application
```c++
#include "Cube.h"
#include <iostream>
int main() {
    Cube c;
    c.setLength(3.14);
    std::cout << "Cube volume = " << c.getVolume() << std::endl;
    return 0;
}
```
### 1.3 C++ Standard Library: STD / STL

- Standard library interfaced by several headers which must be included
- We need to put namespace: `std::cout << ...`
    - Alternatively: `using std::cout;` -> `cout`
    - Or: `using namespace std;`; but that is dangerous...
- Typical interfaces:

```c++
#include <iostream> // console out: std::cout, end line: std::endl
```

- Classes or actually anything can be wrapped in a `namespace` to avoid conflicts due to same names and make clear where each class belongs to

```c++
namespace mkl {
    // Declaration
    class Cube {
        public:
            void setLength(double length);
    };
} // no ;

...

namespace mkl {
    // Implementation
    void Cube::setLength(double length) {
        length_ = length;
    }
}

...

mkl::Cube c;
``` 

## Week 2: C++ Memory Model

Key concepts:

- Pointers
- Heap memory
- Stack memory

### 2.1 Stack Memory and Pointers

- We have access to the memory and lifecycle of variables
- By default, variables are stored in the **stack memory**
- Every variable always has 4 things: name, type, value, memory address
- To get the memory address, we use the `&` operator: it returns the memory address of the variable

```c++
int num = 7;
std::cout << &num << std::endl; // address returned, since in stack, usually large: 0xFFF...
```

- Typical stack addresses are large values, eg.: `0xFFF...`
- Stack memory is associated with the current function: the memory lifecycle is tied to that function! If function ends/returns, variable is lost, because the stack memory is realeased to the system
- Stack memory starts at high memory values and grows down to lower values towards 0, in the order we allocate variables, functions, etc.
    - Note that when we call a function from `main()`, memory is allocated for it in the stack
    - When we exit the function, its memory in the stack is destroyed
    - When a new function or variable is instantiated, the same memory part can be allocated
    - Note also: although stack memory usually grows downwards, C++ compilers often use optimization strategies that flex that rule; therefore, it's not impossible to find cases in which memory addresses change unexpectedly

- **Pointers** are variables that store memory addresses of data
- Pointers are created with `*`

```c++
int num = 7;
int * p = &num; // p stores the address of num in stack memory
```

- If `p` is a pointer, it does not contain the data, but the address to it; if we want to access the data, we use the dereference operator `*`

```c++
int num = 7;
int * p = &num; // p stores the address of num in stack memory
int value_in_num = *p; // dereference
*p = 42; // we can modify the value also
```

- Given a pointer, its address nature is removed and replaced by the value with `*`

- Arrow operator: `->`:

```c++
Cube* c;
(*c).setLength(4);
// It is better to use the arrow operator,
// which is equivalent to the previous (*c).
// -> works when we want to access the object memory of pointers
// that contain addresses of classes
c->setLength(4); 
```

#### Implications

- We cannot return the address of a local variable created inside a function, because the memory allocated for that function is destroyed after we get out of the function! Usually, even the compiler alerts form those issues.

- The source files for the above points are:

```bash
cpp-memory/addressOf.cpp
cpp-memory/foo.cpp
```

- A very interesting file in which the effect of allocating and destroying stack memory as we enter in functions is depicted: `cpp-memory/puzzle.cpp`
    - An address of a variable allocated in the stack memory of a function is returned
    - Since the function's stack memory lives only during the execution of the function, it is afterwards destroyed
    - Any function/variable we execute in the main afterwards will overwrite the previous memory
    - Therefore, best practices
        - Initialize variables to avoid undefined behavior
        - Do not deliver or access invalid memory, keep in mind what part of memory your are using and how long it lives

- Summary of pointer-related concepts in `cpp-memory/main.cpp`:

```c++
#include <iostream>

int main() {
  int num = 7;
  std::cout << " num: " <<  num << std::endl; // 7
  std::cout << "&num: " << &num << std::endl; // address of num

  int *p = &num; // pointer p stores te address of num
  std::cout << " p: " <<  p << std::endl; // address of num
  // the address of p is slightly smaller than the address contained by p
  // because it's the next (downwards) element to num in the stack memory 
  std::cout << "&p: " << &p << std::endl;
  // the value of teh address contained by p = the value of num
  std::cout << "*p: " << *p << std::endl; 

  // The value of the variable stored in the address is changed
  *p = 42;
  std::cout << "*p changed to 42" << std::endl; 
  std::cout << " num: " <<  num << std::endl;

  return 0;
}
```
### 2.2 Heap Memory

-  Heap memory, in contrast to stack memory, usually has low memory values!
    - It is in the opposite end of the memory
    - It grows up (memory addresses become larger) as we allocate in it
    - It is persistent: even when we exit the function/program using it, it is not de-allocated unless we do it manually
- If memory needs to exist for longer than the lifecycle of the function, we must use heap memory: the only way to create heap memory in C++ is `new`.
    - `new` requires `delete` at the end so that we clean/reclaim the heap memory afterwards; that's the only way! (or restart the computer)
    - so, `new` and heap are extremely powerful, but we need to be extremely conscius of what we are doing to avoing filling the heap with rubish that remains even after we exit the program...
    - `new` returns a pointer to the memory storing the data, not an instance of the data itself
- The `new` operator does **always** 3 things
    1. Allocate data on the heap for the data structure
    2. Initialize the data structure
    3. Return a pointer to the start of that data structure
- The `delete` operator must be called after we finih to use our structures in the heap
    - `delete` erases the heap memory, but not the pointer in the stack which contains the adress to it (see code below)
    - therefore, we assign `nullptr` to the pointer in the stack
    - `nullptr` is the address `0x0`: reserved to never be used
        - if we use it, `Segmentation Fault` is raised
        - we understand if a variable has the value `nullptr` it means its an non-initialized pointer
        - `nullptr` exists since C++11 and it's equivalent to the C `NULL`
        - `delete nullptr` is ignored, which is very convenient to avoid errors

- Syntax and meaning:

```c++
// VERY IMPORTANT: syntax + meaning (heap and stack)
// numPtr contains an address to a heap memory part
// but numPtr is in the stack!
// the int value is stored in the heap
// so: the pointer we use is in the stack, but the content is in the heap!
int * numPtr = new int;
// VERY IMPORTANT: use of numPtr and clean-up/memory re-claim
// We need to perform delete to destroy the heap memory and free it
// Since numPtr would still contain the address of the erased memory
// we can assign it the reserved address 0x0
// which is understood as a non-initialized memory part = non-initialized pointer
delete numPtr; numPtr = nullptr;
```

- Using `new` and `delete`: `cpp-heapMemory/heap1.cpp`

```c++
#include "Cube.h"
using uiuc::Cube;
int main() {
  // p is in the stack and contains an address
  // to an int in the heap
  int *p = new int;
  // c is in the stack and contains an address
  // to a Cube in the heap
  Cube *c = new Cube;

  // De-reference (go to memory) and fill in values
  *p = 42;
  (*c).setLength(4);
  // It is better to use the arrow operator,
  // which is equivalent to the previous (*c).
  // -> works when we wnat to access the object memory of pointers
  // that contain addresses of classes
  c->setLength(4); 

  // delete destroys the heap memory allocated for the actual variables
  // but the pointer in the stack still remains
  // and it is pointing to a non-allocated address!
  // Since C++11 we have nullptr, which refers to memory address 0x0
  // nullptr is equivalent to the C NULL
  // Address 0x0 is reserved and never used by the system
  // so we understand if var == nullptr, var contains a non-initialized address
  delete c;  c = nullptr;
  delete p;  p = nullptr;
  return 0;
}
```

- See `cpp-heapMemory/heap2.cpp` for another point: What happens when we try to delete heap memory parts that have been already deleted?

### Heap Memory Puzzels

Puzzle 1: `cpp-heapPuzzles/puzzle1.cpp`:

```c++
// Since we have no new keyword, everything is in the stack
int  i =  2,  j =  4,  k =  8;
int *p = &i, *q = &j, *r = &k;
k = i; // k = i = 2
cout << i << j << k << *p << *q << *r << endl; // 2 4 2, 2 4 2
p = q; // the adress in p is the one in q -> j = 4
cout << i << j << k << *p << *q << *r << endl; // 2 4 2, 4 4 2
*q = *r; // the value pointed by q (j) is the one pointed by r (k) -> j = k = 2
cout << i << j << k << *p << *q << *r << endl; // 2 2 2, 2 2 2
```

Puzzle 2: `cpp-heapPuzzles/puzzle2.cpp`: **stack vs heap**, **referencing or aliasing with &**
```c++
// Since we have the new keyword
// the value pointed by new is in the heap!

// STACK: x = 0x1...: it contains an address to heap memory (small)
// HEAP: *x = 0: it contains an int value, still uninitialized
int *x = new int;

// & is used for referencing variables: aliases/links to memory parts
// y is equivalent to the content pointed by x
// &y is the the address contained by x!
int &y = *x; // y is linking to the content pointed by x
y = 4; // content pointed by x is filled

cout << &x << endl; // 0xF...: address in the stack (large)
cout << x << endl; // 0x1...: address in the heap (small)
cout << *x << endl; // 4: value in the heap assigned through alias y
cout << &y << endl; // 0x1... the address contained in x
cout << y << endl; // 4: value pointed by x in the heap
// cout << *y << endl; // it does not make sense: an int is not an address pointing somewhere...
```

Puzzle 3: `cpp-heapPuzzles/puzzle3.cpp`:
```c++
// Since we use new, some variables live in the heap!
// Pointers in the stack
int *p, *q;
// p points to a memory part in the heap
p = new int;
q = p; // q contains the same address as p
*q = 8; // fill in heap memory
cout << *p << endl; // 8

q = new int; // we create a new memory part in the heap
*q = 9; // fill in latter heap memory
cout << *p << endl; // 8
cout << *q << endl; // 9
```

Puzzle 4: `cpp-heapPuzzles/puzzle4.cpp`: **arrays** in the heap

```c++
  // Pointer in the stack
  int *x;
  int size = 3;
  // Content in the heap
  // Arrays
  x = new int[size]; // we allocate a sequence of ints of size size = 3
  for (int i = 0; i < size; i++) {
    x[i] = i + 3;
  }
  delete[] x; // since x points to a sequence, all sequence must be de-allocated
```

### Additional notes

- Comments: `// Single line`, `/* Multi-line */`
- Header and source files: declarations and implementation
- Headers have `#pragma once` at the top to prevent including them several times (preprocessor directive)
- The preprocessor directive `#include` copies the include code into the source code temporarily!
- Headers contain class member declarations/signatures only, not the implementation
- Source files need to include the headers with the declarations

```c++
#include "Cube.h" // "" for local or user-defined libraries and locations
#include <iostream> //<> for system-wide libraries and standard locations
```

- If we have `main.cpp`, `Cube.cpp`, `Cube.h`:
    - `Cube.cpp` and `main.cpp` include `Cube.h`; `#pragma once` avoids double declarations
    - Compiler creates compiled objects: `main.o` and `Cube.o`
    - Linker combines all `.o` objects into an executable: `main`

- When compiling, header declarations must be known, but not the exact implementation
- When linking, the missing implementations are solved and a whole executable or library is created
- We link against our classes, libraries, and automatically also against system-wide libraries (e.g., `iostream`)

- Copiling with `Makefile`

```bash
cd folder
make
./main
make clean
```

- Typical Linux bash commands:

```bash
sudo, man, pwd, ls, cd, cp, mkdir, mv, rm
```

- Conditionals (`if-else`), ternary operator `A>B?C:D`
- Casting and implicit conversion: usually to higher precission, but also to `bool`.

- Block scope: we create a new isolated stack on top of the current stack if we encapsulate code in `{}`; variables prior to the block stack remain hidden and the variables in the block stack are destroyed when we exit the block.

```c++
  int x = 2;
  std::cout << "Outer scope value of x (should be 2): " << x << std::endl;
  {
    int x = 3;
    int y = 4;
    std::cout << "Inner scope vaue of x (should be 3): " << x << std::endl;
    std::cout << "Inner scope vaue of y (should be 4): " << y << std::endl;
  }
  std::cout << "Outer scope value of x (should be 2): " << x << std::endl;
```

- Some keywords like `if () {}` can have a block stack:

```c++
int x = 2;
if (true) {
    int x = 3;
    std::cout << x << std::endl; // 3
}
std::cout << x << std::endl; // 2
```
- Loops: `for`, `while`
- Modern range-based for loops: `for (temp variable: container) {body}`

## Week 3: C++ Classes

- Every class has an **automatic default constructor**, and all member values are initialized to default values; default primitive values are undefined.
- If we want to control the default values, we need to define a **custom default constructor**: a constructor without arguments that we write with custom initialization values. Three conditions necessary for default constructor
    - Member function with same name as the class
    - Function takes no params
    - Function does not have a return type

```c++
// Header: Cube.h
class Cube {
    Cube() // Custom default constructor
    ...
}
...
// Source: Cube.cpp
Cube::Cube() {
    length_ = 1; // Default will be a unit cube!
}
```

- We can also define not default, but **custom constructors**
    - That happens when we pass arguments
- We can have several constructors!
- **VERY IMPORTAT NOTE**: if we define any constructor, the **automatic default constructor** is not created for us! That means we need to take care of any necessary initialization on our own!
    - For example, if we create custom constructors with parameters, we need to create a default constructor too in order to just instantiate objects without params, eg: `Cube c;`

```c++
// Header: Cube.h
class Cube {
    Cube() // Custom default constructor
    Cube(double length) // Custom constructor
    ...
}
...
// Source: Cube.cpp
Cube::Cube(double length) {
    length_ = length;
}
...
// Executable: main.cpp
uiuc::Cube c(2); // The costom constructor with argument double is and executed
std::cout << Volume << c.getVolume() << std::endl; // 2^3 = 8
```

- See examples in `cpp-ctor`: `ex1`, `ex2`, `ex3`

### Copy Constructors

- If we do not provide any copy constructor, an **automatic copy constructor** is created implicitly for us; it copies all variables, which can be good and bad -- bad (1) if no copy operator is defined for all member variables, or (2) if want to control how copy is made, or (3) if we want to share resources between objects.

- **Custom copy constructor**: defined when we have: (1) a class constructor and (2) exactly one argument: const reference of class type

```c++
// Custom copy constructor
// Constant Cube/same-class-type passed by reference
Cube::Cube(const Cube& c);
...
// Implementation
// In this case, an automatic copy constructor would do the same
// But, we'll see that we usually are going to need custom copy constructors
// and manual transference/copy of data
Cube::Cube(const Cube& c) {
    length_ = c.length_;
}
```

- **VERY IMPORTANT**: Copy constructors are invocated all the time:
    - When passing an object by values as a param
    - Returning an object by value from a function
    - Initializig a new object (with =) 

```c++
// Remember each function has its stack space
// If we pass variables by value, we need to copy them between stack spaces
void foo(Cube c) {
    // Nothing done.
}
Cube foo() { 
    Cube c; // Default constructor invoked
    return c; // Copy constructor invoked, since passed by value
}
int main() {
    Cube c; // Default constructor invoked
    foo(c); // Copy constructor invoked, since passed by value
    // First, the constructors in foo() are invoked: default & copy
    // Then, the copy constructor '=' is invoked to pass object to c2
    Cube c2 = foo();
    return 0;
}
```

- Constructors are invoked only when objects are created in the memory space, not when objects are assigned one to another after being already created.

```c++
Cube c1; // Default constructor
Cube c2 = c1; // Copy constructor
Cube c3; // Default constructor
// No constructor invoked!
// Instead: we have a copy assignment, not a creation!
// See next section on the copy assignment operator =
c3 = c1; 
```
### Copy Assignment Operator `=`

- Whereas a copy constructor creates a new object, the copy assignment operator assigns or replaces values to an existing object; in other words, the copy assignment operator `=` is always called on objects that have previously been created.
- Again, if not manually defined, C++ provides us with an **automatic assignment operator** `=`
- The automatic assignment operator will suffice if we do simple things, but we need our own assigment operator in more complex cases, eg, when we have:
    - externally allocated resources, like memory
    - multiple objects pointing to the same resources
- A **custom assignment operator** needs 4 things to be true
    - Public member
    - Function name: `operator=`
    - Return value: reference of class
    - Argument: exactly one, const reference of class

```c++
// Custome assignment operator
// The values of c will be copied to the instance of the class it's called upon
Cube& Cube::operator=(const Cube& c)
...
Cube& Cube::operator=(const Cube& c) {
    length_ = c.length_
    // We always return *this: the instance of the class itself!
    return *this;
}
```

### Summary of Constructor and Copying Functions

- We have three automatically generated functions (i.e., compiled even if no code is explicitly present)
    - Automatic default constructor: `Cube::Cube()`
    - Automatic copy constructor: `Cube::Cube(double length)`
    - Automatic copy assignment operator: `Cube& Cube::operator=(const Cube& c)`
- All three are invoked in several stages of object instatiation and copy
- All three can be manually overloaded and defined; in that case, they are not automatic

### Variable Storage: Creating, Passing and Returning by Value / Reference / Pointer

- In C++, an instanbce of a variable can be stored in 3 was
    - stored directly in memory
    - accessed by a pointer
    - accessed by a reference

```c++
// 1. Direct storage
// - type has no modifiers
// - object takes its size in memory
Cube c;
int i;
uiuc::HSLAPixel p;

// 2. Storage by pointer
// - Type is modified with *
// - A pointer takes a "memory address width" = 64 bits in 64B systems
// - The pointer points to the alölocated space of the object
Cube* c; // Pointer to Cube
int* i; // Pointer to int
uiux::HSLAPixel* p;

// 3. Storage by reference
// - A reference variable is an alias: it takes 0 bytes of memory to reference a variable
// - Type is modified by &
// - A reference does not store memory itself, it is an alias to another variable!
// - The alias must be assigned when the variable is initialized
Cube& c = cube; // Alias to cube
int& i = count; // Alias to count
uiuc::HSLAPixel& p; // ILLEGAL! It must alias sth when variable initialized! This won't compile
```

- An alias is like a link: given `int& i = count`, when we change either `i` or `count`, the other is changed, too
- It is important to be aware of what we are creating in memory with each approach; see `cpp-memory2/ex1` and the following lines

```c++
Cube c(10);

// By value
// Effect: we CREATE 2x1000 units volume
Cube myCube = c;

// By reference
// Effect: the cube is constructud once, when creating c, not myCube!
// We have only 1000 units in volume altogether!
// We have two variables owning the same cube!
// It's said we have two variables aliasing the same object
// So an alias is like a name: we have an object with two names
Cube & myCube = c;

// By pointer
// Effect: the cube is constructud once, when creating c, not myCube!
// BUT: we create a pointer myCube which is pointing to c
// We have 2 variables: the pointer and the cube itself
// We have 1000 units of volume altogether
Cube * myCube = &c;
```

- Something similar happens when **passing variables by value / reference / pointer**

```c++
bool sendCubeValue(Cube c);
bool sendCubeReference(Cube& c);
bool sendCubePointer(Cube* c);

Cube c(10);

// By value: we make a copy of c and pass it
sendCubeValue(c);

// By reference: we make no copies of c, we pass an alias/reference 
sendCubeReference(c);

// By pointer: we make no copies of c, we pass an address/pointer to it
sendCubePointer(&c);
```

- Finally, we can also return by value / pointer / reference
    - return by value: default, no modifiers
    - return by pointer: modified with `*`
    - return by reference: `&`; **BUT: never return a reference to a stack vaiable created on the stact of your current function, because it will be destroyed when leaving the function!**

### Class Destructor

- Class destructors are called at the end of te lifecycle of the class and they clean up the memory allocated to the class instance
- There is an **automatic default destructor** provided if we do nothing
    - The only action by it is to call the default destructor of all member objects
    - So, if we want some logging or additional clean-up (e.g., free heap memory with `delete`), we need our custom destructor
- **We should not call the destructor explicitly**: the compiler places its calls whenever (1) we leave stack scopes or frames (e.g., function returns) or (2) clear heap memory with `delete` in code; then, it is called in runtime at those points
- Custom destructor:

```c++
// Custom destructor: to add some custom action to the end-of-life of Cube
// - member function
// - name: name of class preceeded by ~
// - all destructors have zero arguments and no return type
Cube::~Cube();
```

- An example is provided in `coo-dtor/main.cpp`
    - Cube instances are created within functions in stack and heap (`new`)
    - When we leave the functions, the objects created within are destroyed with the destructor: output is displayed
    - Whenever we create Cube instances on the heap with `new`, the construstors are called; when we destroy one instance with `delete`, the destructor is called

```c++
double cube_on_stack() {
  Cube c(3); // Construtor called, object memory allocated on function stack
  return c.getVolume(); // We leave the function stack, destructor called
}

void cube_on_heap() {
  Cube * c1 = new Cube(10); // Constructor called, object memory allocated in heap
  Cube * c2 = new Cube; // Constructor called, object memory allocated in heap
  delete c1; // We delete one object from te heap
  // BUT: one object instance remeins in the heap when we leave the function!
}

int main() {
  cube_on_stack(); // create 1, destroy 1
  cube_on_heap(); // create 2, destroy 1
  cube_on_stack(); // create 1, destory 1
  return 0;
}
```

- Therefore, destructors are necesssary for
    - Clearing stack memory automatically
    - Cleaing heap memory: that is not done automatically, we need to do it manually in the destructor
    - Clean open files
    - Manage shared memory

### Errors due to Missuse of Pointers

- **Segmentation fault** (Unix) occurs when we access a restricted memory part

```c++
int* n = nullptr;
std::cout << *n << std::endl; // Segmentation Fault: the content of nullptr is restricted!
```

- **Undefined behavior** occurs when we unintentionally access undesired memory parts that might or might not be initialized; basically, we handle memory address with pointers wrongly, but it compiles, and we can access the memory parts, but these are random uninitialized regions

- **Initialization** of variables is a good practice and necessary for pointers!

```c++
// No initialization: Dangerous, because x contains no address!
// This can lead to careless mistakes and crashes when dereferencing x
int* x;
// Better: Explicitly initializing a pointer to nullptr
// while there is no address to be assigned yet
// BUT: we need to check y != nullptr before dereferencing or using it
int* y = nullptr;
// Equivalent
int* y2(nullptr);
int* y2{nullptr}; // Since C++11, for better differetiation from initialization vs function call

// Initialize EVERYTHING, at least with default values
int i = 0;
int* z = &i;
Cube c; // Initialization with default constructor
```

- **Initialization is also necessary with heap memory**

```c++
int* q = new int; // *q points to a rubbish heap memory value, depending on the compiler, unreliable 
int* r = new int(0); // *r points to value 0 in the heap memory, better
```

- **IMPORTANT**: Reset `deleted` pointers to `nullptr`, because the heap is freed, but we need to somehow signal that to the stack pointer to avoid
    - `deleting` it again
    - dereferencing it mistakenly
 
```c++
// Allocate an integer on the heap:
int* x = new int;
// Now x holds some memory address to a valid integer.
// Do some kind of work with the integer.
// We'll just set that integer to 7:
*x = 7;
// Now delete the pointer to deallocate the heap memory:
delete x;
// This destroys the integer on the heap and frees the memory.
// But now x still holds the memory address!
// Set x to nullptr for safety:
x = nullptr;
```

### Modern Range-based `for`-Loops

```c++
std::vector<int> int_list;
int_list.push_back(1);
int_list.push_back(2);
int_list.push_back(3);
for (int x : int_list) {
    // This version of the loop makes a temporary copy of each
    // list item by value. Since x is a temporary copy,
    // any changes to x do not modify the actual container.
    x = 99;
}
for (int& x : int_list) {
    // This version of the loop will modify each item directly, by reference!
    x = 99;
}
for (const int& x : int_list) {
    // This version uses references, so it doesn't make any temporary copies.
    // However, they are read-only, because they are marked const!
    std::cout << "This item has value: " << x << std::endl;
    // This line would cause an error:
    //x = 99;
}
```

### Unsigned Integers

- Unsigned `ints` have no negative values, instead, double the positive values.
- The underlying bit representation is the same for both, therefore, a negative signed `int` can be interpreted as a very large `unsigned int`, or vice versa (so negative values have higher bit values)
- If `signed` and `unsigned ints` are summed or subtracted, issues might arise if we operate in large bit sizes (either negative values or high positive values)
    - We need to explicitly cast them to work securely
- Comparing `signed` and `unsigned ints` can lead to issues
- Conclusion: use `unsigned ints` only when you really need them
    - Counters without complex arithmetics operations on them
    - When we want to optimize memory for large integer values

```c++
unsigned int x = 10;
unsigned int y = 20;
std::cout << x - y << std::endl; // 4294967286, close to the maximum for an unsigned 32-bit integer
std::cout << (int) (x - y) << std::endl; // -10
int z = x - y;
std::cout << z << std::endl; // -10
```

- Conrainers = generic data structure classes; the STL provides many of these, e.g., `std::vector`
- Container sizes are often `unsigned ints`, which might lead to issues in comparisons if `signed ints` are used

```c++
std::vector<float> v;
// Warning
for (int i = 0; i < v.size(); ++i) {
    std::cout << v[i] << std::endl;
}
// Correct, but i is unsigned,
// so consider that if arithmetics done inside the loop
for (unsigned int i = 0; i < v.size(); ++i) {
    std::cout << v[i] << std::endl;
}
// Error: unsigned 0 - 1 -> very large value for v.size()-1
// -> we alwas enter the loop -> some large i value won't have v[i] -> SegFault
for (int i = 0; i <= v.size()-1; ++i) {
    std::cout << v[i] << std::endl;
}
// Correct
for (int i = 0; i <= (int)v.size()-1; ++i) {
    std::cout << v[i] << std::endl;
}
```

## Week 4: C++ Software Solutions: Templates, Class Inheritance

- Template type: it can take different underlying types when initialized, eg: the containter `std::vector` can be of the form `std::vector<char>`, `std::vector<int>`, `std::vector<Cube>`.

- `std::vector` basics:
    - many functions available
    - they grow dynamically, we don't have to take care of memory allocation

```c++
#include <vector>
//std::vector<T> v; T is the type: int, char, Cube, etc
std::vector<int> v;
v.push_back(4); // append an entry
v.size(); // get number of elements
v[0]; // access element # 0 for rw
```

### Tower of Hanoi: Game Explanation

Problem description:

- We have cubes of ascending size stacking one on top of the other on a location.
- We want to move the cubes two locations to the right one by one without stacking a larger cube on top of a smaller one.
- One cube can be moved at a time, ie., the top cube on a stack.
- We can move the cubes back and forth between any stack.

Problem analysis from a programming perspective:

- We have 3 stacks/locations in which cubes can be stacked.
- All 3 stacks form the game.
- Each cube has a size and a color

The game structure is in `cpp-tower/`
- `Cube.h/cpp`: `length_`, `color_`
- `Stack.h/cpp`: we want to be able to
    - contain ordered cubes stacked one on the other
    - be able to add a cube on top
    - be able to remove/pop/take one cube from the top
    - get size of stack
    - have a look at the cube at the top, ie., get its `length_`
- `Game.h/cpp`: here we have the game itself
    - 3 stacks of cubes, initialized in the constructor
    - `Game::solve()`: solve the game: that is the task to be solved

`cpp-tower/Stack.h`:
```c++
class Stack {
  public:
    void push_back(const Cube & cube);
    Cube removeTop();
    Cube & peekTop();
    unsigned size() const;

    // An overloaded operator<<, allowing us to print the stack via `cout<<`
    // Always this form, the only thing that changes is the type to be printed
    friend std::ostream& operator<<(std::ostream & os, const Stack & stack);

  private:
    std::vector<Cube> cubes_;
};
```

`cpp-tower/Game.h`:
```c++
class Game {
  public:
    Game();
    void solve();

    // An overloaded operator<<, allowing us to print the stack via `cout<<`
    // Always this form, the only thing that changes is the type to be printed
    friend std::ostream& operator<<(std::ostream & os, const Game & game);

  private:
    std::vector<Stack> stacks_;
};
```
### Tower of Hanoi: Solution 1

Game strategy: if we look closely at the movements that need to be taken, we see that the following sequence is repeated:

```
stack 0 -> stack 1
stack 0 -> stack 2
stack 1 -> stack 2
stack 0 -> stack 1
stack 0 <- stack 2
stack 1 <- stack 2
stack 0 -> stack 1
stack 0 -> stack 2
stack 1 -> stack 2
...
```

The direction of the arrow `<->` is decided depending on the size of the top cube on each pair of stacks, but the sequence `[[0,1],[0,2],[1,2]]` is constant.

Hence, **we have done the first steps of the solution manually and have identified by induction a pattern.**

We program then, the code in `cpp-tower-solution/`, summarized like this:

```c++
void Game::solve() {
    while (stacks_[2].size() != 4) {
        _legalMove(0,1);
        _legalMove(0,2);
        _legalMove(1,2);
    }
}

void Game::_legalMove(unsigned index1, unsigned index2) {
    
    if (stacks_[index1].size() == 0 &&
        stacks_[index2].size() > 0) {
        _move(index2, index1);
    } else if ( stacks_[index1].size() > 0 &&
                stacks_[index2].size() == 0) {
        _move(index1, index2);
    } else if ( stacks_[index1].size() > 0 &&
                stacks_[index2].size() > 0) {
        if (stacks_[index1].peekTop().getLength() <
            stacks_[index2].peekTop().getLength()) {
            _move(index1, index2);
        } else {
            _move(index2, index1);
        }
    }
    std::cout << *this << std::endl;
}

void Game::_move(unsigned index1, unsigned index2) {
    Cube cube = stacks_[index1].removeTop();
    stacks_[index2].push_back(cube);
}
```

### Tower of Hanoi: Solution 2

Another approach for solving the problem consists in designing a broader strategy rather than obtaining by induction. The master plan would look like this:

- We want to transfer the cubes from stack 0 (left) to stack 2 (right), following the rule having a smaller cube on top of a bigger at any moment.
- If we have 4 cubes stacked in stack 0, we want to
    1. transfer the top 3 to a spare stack (1) temporarily
    2. transfer the bottom biggest cube to the stack 2
    3. transfer the top 3 from the spare stack (1) to final stack 2

- We notice that the stack can be separated in layers from bottom to top; deeper layers can be ignored while working  on top ones - thus, we simplify th eproblem to the top sub-towers and work recursively
- The 3 stacks are leballed to be `Source`, `Spare`, `Target`, and each time, each of the `Source` and `Spare` labels are re-assigned to different stacks

The generic recursion pseudocode is the following:
```
Initial state:
Stacks 0, 1, 2 = Source, Spare, Target
Blocks in Source: 0, 1, 2, 3

move(Source[0,1,2,3] -> Target (Stack 2))
    move(Source[1,2,3] -> Spare (Stack 1)) // We sawp Spare & Target
        here, we recursively call move():
        move(Source[2,3] -> Spare)
            ...
        move(Source[1] -> Target)
        move(Source[2,3] -> Target)
            ...
    move(Source[0] -> Target)
    move(Spare[1,2,3] -> Target)
        recursion, again

move(Source[start,...,end] -> Target)
    move(Source[start+1,...,end] -> Spare) // We sawp Spare & Target
    move(Source[start] -> Target)
    move(Spare[start+1,...,end] -> Target)

```

The code is implemented in `cpp-tower-solution2`, in the function `void Game::_move(...)`:

```c++
// Move the cubes in the range [start...end] from `source` to `target`, using spare as a spare spot:
void Game::_move(
  unsigned start, unsigned end,
  Stack & source, Stack & target, Stack & spare,
  unsigned depth
) {
  cout << "Planning (depth=" << depth++ << "): Move [" << start << ".." << end << "] from Stack@" << &source << " -> Stack@" << &target << ", Spare@" << &spare << "]" << endl;

  // Check if we are only moving one cube:
  if (start == end) {
    // If so, move it directly:
    _moveCube( source, target );
    cout << *this << endl;
  } else {
    // Otherwise, use our move strategy:
    _move(start + 1, end  , source, spare , target, depth);
    _move(start    , start, source, target, spare , depth);
    _move(start + 1, end  , spare , target, source, depth);
  }
}
```

### Templates

C++ allows creating template functions and classes, which are basically objects that have parameters of types defined at another point in code.

- Templates are defined declaring `template <typename T>` before the beginning of the class/function, where `T` is the type later defined.
- Templated variables are checked at compile time.

```c++
// Class
template <typename T>
class List {
    ...
    private:
        T data_;
}

// Function
template <typename T>
int my_max(T a, T b) {
    if (a > b) { return a; }
    return b;
}

// Templated variables are checked at compile time

// OK: max() is instantiated automatically as max<int>
my_max(4, 7);
// OK: two chars compared
my_max('a', 'b');
// OK: since '>' is defined for std:strings
my_max(std::string('hello'), std::string('world'));
// ERROR: if no '>' operator is defined for Cube
my_max(Cube(4), Cube(7));

```

### Inheritance

- We can generate generic classes which are then inherited by more specific classes, e.g. `Shape -> Cube`; we call them **base** class and **derived** class
- Base classes have all member variables and methods declarations that are share by all derived ones.
- Usually public inheritance is done: `class Cube : public Shape { ... }`: the derived class
    - can access all **public** members (variables & methods) of the base class
    - cannot access the **private** members of the base class

- When a derived class is initilized, the derived class **must** construct the base class:
    - `Cube` must construct `Shape`
    - Default constructor is used by default
    - Custom constructor can be used with an **initialization list**

- Initializer list: custom constructor which can
    - Initialize the base class
    - Initialize the current class using another constructor
    - Initialize the default values of member variables

Example in `cpp-inheritance/`
```c++
//// Shape.h
class Shape {
  public:
    Shape();
    Shape(double width);
    // All derived classes will have this method
    double getWidth() const;

  private:
    // All derived classes will have this variable
    double width_;
};

//// Cube.h
namespace uiuc {
  class Cube : public Shape {
    // Since we inherit from Shape,
    // we get all public methods and variables from it
    // Public methods from Shape should give access
    // to private variables, eg: getWidth() -> width_
    public:
      Cube(double width, uiuc::HSLAPixel color);
      double getVolume() const;

    private:
      uiuc::HSLAPixel color_;
  };
}

//// Shape.cpp

// From the default constructor
// We call the custom constructor below by using Shape(1)
Shape::Shape() : Shape(1) {
  // Nothing.
}

// We have an initialized list
// with which we initialize the member varieble width_
// Initializer list: MyClass::MyClass(double var) : var_(var)
// With the initializer list we avoid writing redundant lines of code
// and initialize everything in one place in the code!
Shape::Shape(double width) : width_(width) {
  // Nothing.
}

double Shape::getWidth() const {
  return width_;
}

//// Cube.cpp
namespace uiuc {
  // width value is passed to the Shape constructor
  Cube::Cube(double width, uiuc::HSLAPixel color) : Shape(width) {
    color_ = color;
  }

  double Cube::getVolume() const {
    // We cannot access Shape::width_ due to it being `private`
    // ...instead we use the public Shape::getWidth(), a public function
    return getWidth() * getWidth() * getWidth();
  }
}
```

