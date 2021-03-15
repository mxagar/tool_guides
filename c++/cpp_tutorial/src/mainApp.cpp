/**
*\file mainApp.cpp
*\author Mikel Sagardia
*\date 2018
*/

#include <iostream>
#include "dynamicCode.h"
#include "staticCode.h"
#include "objectCode.h"

// initalization of static variable N which counts number of created objects of CVector
// static variables always innitialized outside of the class and at the begining of the code which is using the class!
// type is included also, although it is specified in class
// they are initialized as global
unsigned int CVector::N = 0;

/**
 *\param void, nothing
 *\return int, nothing
 */
int main(){

	std::cout << "C++ Tests: Classes, Templates, etc." << std::endl;
    
    //std::cout << "The factorial of 5 is " << factorial(5) << std::endl;

    //::::: objectCode.h/cpp

    // Classes: Basic Features Tested: general class definition, access/assignment, (default) constructors & destructors, pointers and arrays
    test_classes_basic(); // Implementations in CRectangle

    // Classes: Intermmediate Features Tested: operator overloading, this, static members
	test_classes_intermediate(); // Implementations in CVector

	// Classes: Advanced Features Tested: friends, inheritance, polymorphism: virtual methods and abstract classes
	test_classes_advanced(); // Implemenations in CShape, CSphere, CCube

    //::::: dynamicCode.h/cpp

	// Templates: functions
	test_template_functions();

	// Templates: classes
	test_template_classes();

	// Namespaces
	test_namespaces();

	// Exceptions
	test_exceptions();

	// Type Casting
	test_casting();

	// File I/O + number <-> string conversion
	test_file_io();

    //::::: staticCode.h/cpp

    test_stl();

    return (0);

}
