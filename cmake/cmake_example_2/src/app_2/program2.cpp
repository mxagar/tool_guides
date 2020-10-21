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
int main(void){

    std::cout << "\n--> Program 2:" << std::endl;
    std::cout << "Without classes:" << std::endl;
    print_hello();
    std::cout << std::endl;
    std::cout << "The factorial of 5 is " << factorial(5) << std::endl;
	
    std::cout << "With classes:" << std::endl;
    Factorial f;
    PrintHello p;
    p.printHello();
    std::cout << std::endl;
    std::cout << "The factorial of 5 is " << f.computeFactorial(5) << std::endl;

    return 0;

}
