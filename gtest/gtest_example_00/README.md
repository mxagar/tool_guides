# How to use

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

TBD

**Linux**

TBD