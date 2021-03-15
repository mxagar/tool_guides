# Introduction

This folder contains two C++ projects:

- The files in this level belong to a complete C++14 case, with static and dynamic libraries.
- The files in the folder `basic/` compose a basic C++ main file, with the main features of the language.

# Project in current level

Very basic DevOps:

- Make
- CMake
- Doxygen
- General C++ project structure: main file, objects, static libraries, dynamic libraries, library dependencies.
- Linux path setting: initLibs.

C++ related contents:

- `objectCode.h/cpp`

	- Classes I: General, Constructors/Destructors, Overloading, Pointers
	- Classes II: Overloading, this, Static
	- Classes III: Friends, Inheritance, Multiple Inheritance, Polymorphism, Virtual Members, Abstract Classes

- `dynamicCode.h/cpp`

	- Templates: Functions, Classes
	- Namespaces
	- Exceptions

- `staticCode.h/cpp`

	- Type Casting
	- Preprocessor Directives
	- File I/O

# Project in `basic`

See also `basic/README.md`:

- Basic "Hello World!"
- Numbers and Basic Operations
- Basic Strings
- Constants & Literals
- Input / Output ans String-Number Conversion
- Control Structures: if, for, while
- Functions
- Arrays and Character Strings
- Pointers and Dynamic Memory
- Other Data Types
- Pointer/Reference return types in functions

# Compile

```bash
make -f Makefile.unix cleanall && make -f Makefile.unix
```

# Execute

```bash
source initLibs
./mainApp
```
