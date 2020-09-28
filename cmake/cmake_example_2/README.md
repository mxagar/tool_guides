# Small project compilation using CMake

Mikel Sagardia, 2013.
No warranties.

## Contents in root folder

The root folder contains:

- this `README.md` file with instructions,
- a `CMakeLists.txt` file for configuring the project build parameters,
- an empty `share` folder which should contain all necessary data for the gerated binaries, if necessary,
- and a `src` folder with all the source files of the project

```bash
.
|-- readme.txt
|-- share
`-- src
    |-- CMakeLists.txt
    |-- Doxyfile.in
    |-- app_1
    |   |-- CMakeLists.txt
    |   `-- program1.cpp
    |-- app_2
    |   |-- CMakeLists.txt
    |   `-- program2.cpp
    |-- doc
    |   |-- CMakeLists.txt
    |   `-- Doxyfile.in
    |-- lib_1
    |   |-- CMakeLists.txt
    |   |-- factorial.cpp
    |   `-- factorial.h
    `-- lib_2
        |-- CMakeLists.txt
        |-- hello.cpp
        `-- hello.h
```

## How to build

1. Go to the root folder (the folder where this file is).

2. Build preferably out-of-source, so crate a folder called build in the root folder:

    ```bash
    mkdir build
    ```

3. Enter 'build' folder:

    ```bash
    cd build
    ```

4. Generate project build configuration using CMake:

    ```bash
        cmake ../src -G "Unix Makefiles"
    ```

    You can use any other configuration type also.

5. Make and install:

    ```bash
    make
    make install
    ```

6. If you want to generate the documentation:

    ```bash
    make doc
    ```

## How to use after building

After the project has been built, you will find the following new folders in the root folder:

**`lib`** contains all generated libraries.

**`include`** contains all necessary header files to use the generated libraries.

**`bin`** contains all generated programs. Do not forget setting correctly:

- Unix (Bash):

    ```bash
    cd bin
    setenv LD_LIBRARY_PATH ../lib:$LD_LIBRARY_PATH
    ./app1
    ```

- Windows:

    ```bash
    TODO
    ```

**`build`** contains all built material (and more).
If you executed `make install` you do not need to enter here, exept for the documentation.
If you executed `make install` use the objects located in the previous three folders.

**`build/doc`** contains all Doxygen documentation if you executed `make doc`.

- Open `build/doc/html/index.html` with your browser if you want the documentation in HTML format.
- Execute `build/doc/latex/make` for generating the Latex format: `build/doc/latex/refman.pdf`.
