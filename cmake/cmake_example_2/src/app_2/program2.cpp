/**
*\file program2.cpp
*\author Mikel Sagardia
*\date 16.02.2010
*/

#include <iostream>
#include "hello.h"
#include "factorial.h"

/**
 *\param void, nothing
 *\return int, nothing
 */
int main(){
	std::cout << "Program 2:" << std::endl;
    print_hello();
    std::cout << std::endl;
    std::cout << "The factorial of 5 is " << factorial(5) << std::endl;
    return 0;
}
