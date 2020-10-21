/**
*\file hello.cpp
*\author Mikel Sagardia
*\date 16.02.2010
*/

#include <iostream>
#include "hello.h"

void print_hello(void){
   std::cout << "Hello World!";
}

void PrintHello::printHello(void) {
   print_hello();
}