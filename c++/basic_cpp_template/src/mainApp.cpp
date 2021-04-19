/**
*\file mainApp.cpp
*\author Mikel Sagardia
*\date 2018
*/

#include <iostream> //std::cout, std::endl
#include <string> //std::string

#include "ObjectCode.h"

/**
 * EVERY C++ code starts to be executed from the main() function
 *\param number of space separated words in program call: 1 (binary name) + number of argument identifiers & values
 *\param character chain with program call (name + arguments); arvg[0] refers to all letters/characters of the program binary name
 *\return int, nothing
 */
int main(int argc, char** argv) {

	std::cout << "Hello World!" << std::endl;

	test_empty();

    return (0);
    //exit(0); // exitcode = 0: program finished normally - may be used by OS or calling programs


}