#CMakeLists.txt in src/Hello/ dir

#Adds a library called Hello (libHello.a in Linux) from the source file hello.cc
#I understand it compiles a library from source code hello.cc
#ADD_LIBRARY(PRODUCT=Library_name INPUT=source_filename)
ADD_LIBRARY(Hello hello)
