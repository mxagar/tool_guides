CC=g++
CFLAGS=-g -Wall

OBJS=ObjectCode.o

INCLUDE:=-I.
INCLUDE:=$(INCLUDE) -I/usr/include

LDFLAGS=
LIBS=
LIBPATHS=

EXECUTABLE=mainApp

#all: mainApp
all: $(EXECUTABLE)
	@+echo "(c) Mikel Sagardia 2018"
	echo "Generic C++ Project"
	
#mainApp: mainApp.o $(OBJS)
	#$(CC) mainApp.o $(OBJS) -o mainApp
mainApp: mainApp.o $(OBJS)
	$(CC) mainApp.o $(LDFLAGS) -L. $(LIBPATHS) $(LIBS) $(OBJS) -o mainApp

mainApp.o: mainApp.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c mainApp.cpp

ObjectCode.o: ObjectCode.cpp ObjectCode.h
	$(CC) $(CFLAGS) $(INCLUDE) -c ObjectCode.cpp 
	
cleanall: clean
	-rm -R -f $(EXECUTABLE) *.dylib *.so *.a *.dSYM

clean:
	-rm *.o
	
doc:
	-mkdir  ../doc/latex ../doc/html ../doc/rtf
	-doxygen Doxyfile
	
cleandoc:
	-rm -R ../doc/latex/* ../doc/html/* ../doc/rtf/*
	-rm -R ../doc
