# Preprocessor macros C++

This document compiles notes of preprocessor macros for C++.

Note:
- Preprocessor is executed **before** compilation.
- General syntax: `#directive paramters`.
- No `;`
- New lines are new directives, except when we write `\`

## Macro definitions: `#define, #undef`

```c++
#define <identifier> <replacement>

#define TABLE_SIZE 100
	<identifier> is replaced in the following code by <identifier>

#define getmax(a,b) a>b?a:b
	if also works with functions, in which function structure is found and replaced by its implementation

#undef <identifier>
	when found, previously defined <identifier> is removed
```

## Conditional inclusions: `#if #ifdef, #ifndef, #elif, #else, #endif`

```c++
#ifdef TABLE_SIZE
int table[TABLE_SIZE]; // created iff TABLE_SIZE is defined!!
#endif

#ifndef TABLE_SIZE
#define TABLE_SIZE 100
#else
// ...
#endif

#if TABLE_SIZE<50
#undef TABLE_SIZE
#define TABLE_SIZE 100
#elif TABLE_SIZE>100
#undef TABLE_SIZE
#define TABLE_SIZE 100
#endif
```

## Error: `#error`

```c++
#ifndef __cpluplus
#error A C++ compiler is required
#endif
```

Error thrown in compilation.
`__cplusplus` is a macro defined in all C++ compilers.

## File inclusion

```c++
#include <iostream>
	First looked in default directories.
	User with standard stuff: std, C++, etc.

#include "Header.h"
	First looked in the file directory for Header.h, then in other default directories.
	Use with external libraries and own stuff.
```

## Compiler options

```c++
#pragma
	Look manual, compiler specific.
```

## Standard predefind macros

Google them up, there are many.

```c++
__cplusplus
__LINE__
__FILE__
__DATE__
__TIME__
```