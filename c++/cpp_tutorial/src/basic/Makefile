# Compilation with GNU g++:
# g++ -Wall mainApp.cpp -o mainApp

# Define compiler
CC=g++

# Define options for the compiler
#CFLAGS=-c -g
CFLAGS=-c -g -Wall

all: mainApp

mainApp: mainApp.o
	$(CC) mainApp.o -o mainApp

mainApp.o: mainApp.cpp
	$(CC) $(CFLAGS) mainApp.cpp

clean:
	rm -f *.o
	
cleanall:
	rm -R -f *.o mainApp *.dSYM