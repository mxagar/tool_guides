/**
*\file hello.h
*\author Mikel Sagardia
*\date 16.02.2010
*/

#ifndef HELLO_H_
#define HELLO_H_

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
 * Prints 'Hello' printed on the screen.
 *\param void, nothing
 *\return void, nothing
 */
DLLPORT void print_hello(void);

/**
 * Class which prints "hello".
 */
class DLLPORT PrintHello {
public:
    /*
    * Print "hello".
    */
    void printHello();
};

#endif /* HELLO_H_ */
