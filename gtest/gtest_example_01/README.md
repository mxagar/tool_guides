# Simple Compilation Test

This project downloads googletest from GitHub and compiles its together with our existing CMake project.
It is the recommented way; see [generic build instructions of googletest](https://github.com/google/googletest/tree/master/googletest).

Basically, 

- you have `CMakeLists.txt.in` file, in which the GTest repository download commands are specified,

- and, additionally, your `CMakeLists.txt` file, related to the project. The first lines of this file include `CMakeLists.txt.in` and prepare GTest; the last lines add the binaries of pur project and link them against GTest.