/**
*\file factorial.h
*\author Mikel Sagardia
*\date 16.02.2010
*/

#ifndef FACTORIAL_H_
#define FACTORIAL_H_

// DLLPORT should be defined once to contain the macro with __declspec() necessary for Windows
// Every function & class in the header definitions must be preceeded with DLLPORT
// On Linux, Mac, Windows-STATIC DLLPORT = ""
// On Windows with SHARED libraries compiled, DLLPORT = __declspec(dllexport|dllimport)
#ifndef DLLPORT
    #if !defined(_WIN32) || defined(BUILD_STATIC) /* Linux, Mac, Windows-STATIC*/
        #define DLLPORT
    #elif defined(CMake_Example_EXPORTS) // replace CMake Project name!
        #define DLLPORT __declspec(dllexport)
    #else
        #define DLLPORT __declspec(dllimport)
    #endif
#endif /* DLLPORT */

/**
 * Computes factorial of the given number.
 *\param integer to be processed
 *\return factorial of the given integer
 */
DLLPORT int factorial(int n);

/**
 * Class which computes a factorial.
 */
class DLLPORT Factorial {
public:
    /*
    * Compute a the factorial of n.
    */
    int computeFactorial(int n);
};

#endif /* FACTORIAL_H_ */
