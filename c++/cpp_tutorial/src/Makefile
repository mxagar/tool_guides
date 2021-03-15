CC=g++
CFLAGS=-g -Wall

#OBJS=staticCode.o dynamicCode.o
OBJS=objectCode.o

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
mainApp: mainApp.o libStatic.a libDynamic $(OBJS)
	$(CC) mainApp.o $(LDFLAGS) -L. $(LIBPATHS) -lDynamic $(LIBS) libStatic.a $(OBJS) -o mainApp

mainApp.o: mainApp.cpp dynamicCode.h staticCode.h
	$(CC) $(CFLAGS) $(INCLUDE) -c mainApp.cpp

libDynamic: dynamicCode.o
	$(CC) -dynamiclib -o libDynamic.dylib dynamicCode.o
	#$(CC) -shared -o libDynamic.so dynamicCode.o

libStatic.a: staticCode.o
	ar -rv libStatic.a staticCode.o

objectCode.o: objectCode.cpp objectCode.h
	$(CC) $(CFLAGS) $(INCLUDE) -c objectCode.cpp 

staticCode.o: staticCode.cpp staticCode.h
	$(CC) $(CFLAGS) $(INCLUDE) -c staticCode.cpp 

dynamicCode.o: dynamicCode.cpp dynamicCode.h
	#$(CC) $(CFLAGS) $(INCLUDE) -c dynamicCode.cpp -o dynamicCode.o
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -c dynamicCode.cpp -o dynamicCode.o
	
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
