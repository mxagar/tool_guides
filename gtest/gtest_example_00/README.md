# Simple Compilation Test

This project compiles an executable against GTest.
However, this is not the recommended approach; see the [generic build instructions of googletest](https://github.com/google/googletest/tree/master/googletest).
A better approach is implemented in `../gtest_example_01`.
## Compile

**Mac**

Since I compiled googletest on Terminal, every code using gtest needs to be compiled against it the same way.
If you try to compile it, eg., using VS Code, it will fail unless you manage to use the exact same toolchain employed for compiling gtest.

``` bash
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS=-std=c++11 .. -G "Unix Makefiles"
make
```

The toochain/C compiler used to compile gtest is

`/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc`

**Windows**

I have not managed to make it work.
`gtest.lib` is not found during compilation, although paths are correctly set.

**Linux**

TBD.