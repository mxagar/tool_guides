/**
*\file mainApp.cpp
*\author Mikel Sagardia
*\date 2018
*/

#include <iostream> //std::cout, std::endl
#include <string> //std::string
#include <sstream> //std::stringstream: e.g., to convert strings into numbers
#include <new> // new (std::nothrow)

# define PI 3.14159 //constants defined by the preprocessor for all the code / compiled files - name them in CAPS!
# define NEWLINE '\n'

// To avoid writing namespace std:: - dont overuse it with external libraries
//using namespace std;

// Global variables - avoid them, unless they must be known in every code corner every time
unsigned int g_appCounter = 0;
float g_array[] = {1.0, 2.0, 3.0};

// Function prototype declarations
// No variable name is necessary, it is enough with type
int addition(int a , int b); // General
int substraction(int a , int b); // General
int addition_default(int a, int b = 2); // Default values
void print_hello(void); // No return type
int substraction_reference(int& a, int& b); // Arguments by reference
int substraction_constant(const int& a, const int& b); // Arguments by reference, but unmutable
float addition(float a, float b); // Overloading of functions of same name but different argument number/types
inline float substraction(float& a, float& b); // Inline: function call substituted by function code if possible
int factorial(int a); // Recursive functions
void functionWithArray(float my_array[], int size); // If address of array passed, pass its length too
void increase_generic(void* data, int psize); // Function with void pointer
int operation(int x, int y, int (*function_call)(int, int)); // Function with another generic function passed as pointer
// Return pointer/reference: ONLY with GLOBAL or STATIC return values (known outside the scope of the function)
unsigned int* return_pointer(); // Function with return type pointer: address to a variable returned, might be array
float& assign_reference(int i); // Function with return type reference: used on left hand side to assign values!
// Especial sections in dedicated functions
void arraysAndCharacterSequences();
void pointersAndDynamicMemory();
void otherDataStructuresAndTypes();
void pointerAndReferenceReturnTypes();


/**
 * EVERY C++ code starts to be executed from the main() function
 *\param number of space separated words in program call: 1 (binary name) + number of argument identifiers & values
 *\param character chain with program call (name + arguments); arvg[0] refers to all letters/characters of the program binary name
 *\return int, nothing
 */
int main(int argc, char** argv) {

	// It could be also:
	// 		int main() {...}

	//::::: 1) Hello World! Output
	
	/* 
	This is multi-line a comment
	*/
	// This is single-line comment
	std::cout << "Hello World!" << std::endl;

	//::::: 2) Numbers and Basic Operations

	// Local variables
	int a = 2 + 3 * 10 - 5;
	int b(2); // b = 2
	float c = float(a) / 3.0; // casting
	unsigned long int d, e, f;
	d = e = f = c;

	std::cout << "Computed value: " << d << std::endl;
 
	int modulo = a % b; // rest of a/b

	d += 3; // d = d + 3
	std::cout << "Increased value: " << d << std::endl;
	// Also:
	// -=, *=, /=, ...
	d++; // d = d + 1
	// Also --

	// IMPORTANT: ++ and -- can go before or after
	//int A, B;
	//B = 3;
	//	A = ++B; 					A = B++;
	//	Result: A is 4, B is 4		Result: A is 3, B is 4
	//
	// Therefore
	// - Be careful if assignment (=) is used with ++ or --
	// - Usually, you mean to use it as a prefix: ++i rather than i++
	// - Although without assignment (=), both produce the same result 

	// Comparison and Logic Operations

	bool comparison_result_1 = (a == b);
	bool comparison_result_2 = (a != b);

	bool comparison_result_3 = comparison_result_1 && comparison_result_2;
	comparison_result_3 = !(comparison_result_1);
	comparison_result_3 = comparison_result_1 || comparison_result_2;

	// Operator ?: if this (?) then that else (:) another thing 
	d = a >= b ? a : b;
	// Operator ,: two operations in a row; rightmost is the assigned value
	d = (a = 3, a + 2);

	//::::: 3) Strings

	std::string myString = "This is a string";
	std::string myOtherString; // declaration without value
	myOtherString = "This is another string"; // value defined at runtime
	myOtherString = "This is another, different string"; // value changed at runtime

	std::cout << myString << ". " << myOtherString << std::endl;

	//::::: 4) Constants & Literals

	int decimal = 75;
	int octal = 0113; // starts with 0
	int hexadecimal = 0x4b; // starts with 0x
	unsigned long int big_value = 75ul; // unsigned long forced
	// Conversion: only dec -> hex/oct makes sense; computer stores it always as binary...

	float pi_value = 3.14159;
	float large_float = 6.02e23;
	float tyny_float = 1.6e-19;
	double pi_precise = 3.141592653589793;
	float my_float = 3.0f; // float forced
	long double my_double = 3.0l; // long double forced

	//char				1 Byte			-128, 127; 0, 255
	//short int			2 Bytes			-32.768, 32.767; 0, 65.535
	//int				4 Bytes			-2.147.483.648, 2.147.483.647; 0, 4.294.967.295
	//long int (long)	4 Bytes			same as int!
	//bool				1 Bytes			true or false; typedef char?
	//float 			4 Bytes			+/- 3.4e +/- 38 ~ 7 digits
	//double			8 Bytes			+/- 1.7e +/- 308 ~ 15 digits
	//long double		8 Bytes			same as double!

	// IMPORTANT: int has same size as float; the real difference is between float and double

	std::cout << "Sizes of types: bool " << sizeof(bool)
							<< ", int " << sizeof(int)
							<< ", float " << sizeof(pi_value) 
							<< ", double " << sizeof(pi_precise) <<  std::endl;


	char my_character = 'm'; // Single characters with '
	const char* my_char_string = "Mikel is typing"; // Character strings with " and const char*

	std::cout << "Printing characters: " << my_character << ", " << my_char_string << std::endl;
	
	std::string my_string(my_char_string);	// after converting to std::string, more functions available, mutable, etc.

	//Escaping: special symbols in characters/strings
	const char* my_char_other_string = "Mikel\nis typing\tseveral thins";
	// \n 	new line
	// \t 	tab
	// \? 	question mark
	// \' 	single quote
	// \" 	double quote

	// Constants: preprocessor (see at beginning of file) and declared
	float radius = 1.0;
	const float perimeter = 2.0 * PI * radius;
	std::cout << "Perimeter of circumference of radius " << radius << " is " << perimeter << std::endl;

	//::::: 5) Input / Output ans String-Number Conversion

	//Uncomment the 3 lines with std::cin!

	int age = 34;
	std::cout << "Write your age and press enter: " << std::endl;
	//std::cin >> age;
	std::cout << "The age you entered is: " << age << std::endl;	
	// Note: std::cin stops when blank space is found; for those cases, better use getline()
	std::string myname = "Mikel S";
	std::cout << "Write your name and surname and press enter: ";
	//std::cin.ignore(); //Necessary in order to remove the ENTER key from the age question (not name question)
	//getline(std::cin, myname);
	std::cout << "Hello, " << myname << ", you are " << age << " years old" << std::endl;

	std::string my_number_string = "1024";
	int my_number_int;
	std::stringstream(my_number_string) >> my_number_int;
	std::cout << "String " << my_number_string << " converted to integer: " << my_number_int << std::endl;

	//::::: 6) Control Structures: if, for, while

	// Blocks defined with {}
	// if only ONE line inside block, no {} needed, no new line needed

	// if, else if, else
	int condition = 0;
	if (condition > 0) {
		std::cout << "Do something" << std::endl;
	} else if (condition == 0) {
		condition++;
	} else {
		condition--;
	}

	// while: loop performed iff condition met
	std::cout << "while" << std::endl;
	int counter = 10;
	while (counter > 0) {
		// We enter here iff counter > 0
		counter--;
	}

	// do while: as while, but first loop is always performed
	unsigned int n = 10;
	std::cout << "do while" << std::endl;
	do {
		n--;
	} while (n > 0); // Do not forget ; after while

	//for
	std::cout << "for loop: " << std::endl;
	for (int i = 0; i < 10; ++i) {
		if (i == 3) continue; // jumps to the next loop ignoring the rest of the instructions in the current loop
		else if (i == 5) break; // goes out from enclosing loop
		std::cout << "\titeration " << i << std::endl;
	}

	//switch: compare expression/variable against constants
	//similar to if-else, but 
	std::cout << "switch: " << std::endl;
	int x = 3;
	switch (x) {
		case 1:
			// since no break added, next cases will be checked also
		case 2: // checked if case 1 does not fulfill or it fulfills but no break written
			// since no break added, next cases will be checked also
		case 3:
			std::cout << "x is 1, 2 or 3" << std::endl;
			break; // we get out of switch
		default: //optional: if previous cases were not fulfilled, it enters here
			std::cout << "x is not 1, 2 nor 3" << std::endl;
	}

	//::::: 7) Functions

	// General: call, definition/declaration above, implementation below
	int value_a = 3;
	int value_b = 5;
	// The call to a function is replaced by its return value
	int added_value = addition(value_a, value_b);
	std::cout << "Added value = " << added_value << std::endl;

	// Default values: only when arguments passed by value or when ref. + const
	// Write the default vaue in function declaration - not in implementation
	added_value = addition_default(value_a); // value_b = 2
	std::cout << "Added value = " << added_value << std::endl;
	added_value = addition_default(value_a, value_b);
	std::cout << "Added value = " << added_value << std::endl;

	// Functions with no type: void
	print_hello();

	// Arguments by value and reference: &
	// - By value: arguments are copied in the function, not altering the outside value of the passed argument
	//		Example: previous function: int addition(int, int)
	// - By reference: the address to the variable outside which is being passed is processed
	//		Example: 
	int substracted_value = substraction_reference(value_a, value_b);

	// Arguments by reference (faster than by value), but unmutability assured by const
	substracted_value = substraction_constant(value_a, value_b);

	// Overloading: functions can have same name if their argument types or number of arguments are different
	// Return type is irrelevant: arguments must be different in number or TYPE - using 'type&' or 'type' is considered SAME TYPE
	float a_float = 2.0;
	float b_float = 3.0;	
	float added_float = addition(a_float, b_float);

	// Inline functions: we tell the compiler that we prefer if the code of the function would substitute the call
	// It is usually faster than calling a function - many compiler do that already
	float substracted_float = substraction(a_float, b_float);

	// Recursivity: functions that call themselves
	int intereger_to_factor = 5;
	int factorial_value = factorial(intereger_to_factor);
	std::cout << intereger_to_factor << "! = " << factorial_value << std::endl;

	//::::: 8) Arrays and Character Sequences
	arraysAndCharacterSequences();

	//::::: 9) Pointers and Dynamic Memory
	pointersAndDynamicMemory();

	//::::: 10) Other Data Types
	otherDataStructuresAndTypes();

	//::::: 11) Pointer/Reference return types in functions
	pointerAndReferenceReturnTypes();

    return (0);
    //exit(0); // exitcode = 0: program finished normally - may be used by OS or calling programs

}

// :::
// ::: AUXILIARY FUNCTIONS: IMPLEMENTATION (declaration at beginning, call in main())
// :::

// General function implementation (case: arguments by value)
int addition(int a, int b) {
	// Variables created within a function are seen in only it: (memory) scope is limited to the function block
	// Variables created in main() or calling block are not visible to the function, unless passed as arguments
	// Global variables are visible everywhere
	int r = a + b;
	//return (r);
	return (a + b); // better style, because no new variable created
}

// General function implementation (case: arguments by value)
int substraction(int a, int b) {
	return (a - b);
}

// Function with default values
int addition_default(int a, int b) {
	// If b is not entered, it is assumed b = 2
	// Else, if b is entered, its default value 2 is ignored
	// BUT: ONLY put b = 2 IN DECLARATION, not in implementation
	// It also works with ref. + const arguments
	int r = a + b;
	return (a + b);
}

// Function with no type
//void print_hello(void) { // Passing (void) as argument is also valid - use always a TYPE (also void) in function definitions
void print_hello() {
	std::cout << "Hello from print_hello() function!" << std::endl;
}

// Function: case: arguments by reference
int substraction_reference(int& a, int& b) {
	a -= b;
	return (a);
	// a & b passed by reference: changes in them affect values outside
	// Advantage: no copies needed for arguments (as per value) -> faster
	// It is actually not necessary to return a, since we modify it already inside the function and it passed as reference
	// 		Therefore, we could have had void as return type
	// CONCLUSION: arguments passed by reference can be also output variables!
}

// Function: Arguments by reference, but unmutability assured with const
int substraction_constant(const int& a, const int& b) {
	return (a - b);
	// By reference is faster, but can be dangerous, since values are changed inside the function
	// This way, they are passed by reference but cannot be modified inside the function
	// CONCLUSION: arguments passed by reference but const cannot be the putput variable
	// Ref. + const can also work with default argument values
}

// Overloaded function: same name as addition(int, int), but different types
float addition(float a, float b) {
	// Return type is irrelevant: arguments must be different in number or type
	return (a + b);
}

// Inline functions: we tell the compiler that we prefer if the code of the function would substitute the call
inline float substraction(float& a, float& b) {
	// It is usually faster than calling a function - many compiler do that already
	return (a - b);
}

// Recursive function
int factorial(int a) {
	int f = a;
	if (a > 1) {
		f *= factorial(a - 1);
	} else {
		f = 1;
	}
	return f;
}

// Function with array
void functionWithArray(float my_array[], int size) {
	// If the address of an array is passed, we should pass its length as argument too
	for (int i = 0; i < size; ++i)
		my_array[i] = 0.0;
}

// Function with void pointer
void increase_generic(void* data, int psize) {
	// void pointers:
	// - pointers that point to elements that have no assigned type
	// - no deference (access through *) possible with them, because type size is unknown!
	// - they need to be casted before accessing to the memory they are targeting
	// - use: generic parameters for functions usable with different argument types
	if (psize == sizeof(char)) {
		char* p_char;
		p_char = (char*) data;
		++(*p_char);
	} else if (psize == sizeof(int)) {
		int* p_int;
		p_int = (int*) data;
		++(*p_int);
	}
}

// Function with another generic function passed as pointer
int operation(int x, int y, int (*function_call)(int, int)) {
	// Any function with the following prototype is valid:
	// 		int function_name(int, int)
	// 			e.g.
	// 				int addition(int a , int b);
	//				int substraction(int a , int b);
	// The pointer function: aka callback function
	// The pointer function (argument) doesnt need to be declared in a special way
	// When using operation(), pointer function name is passed, without *, nor return type, nor argument types
	//		e.g.
	//			int z = operation(x, y, addition);
	int g;
	g = (*function_call)(x, y); //
	return (g);
}

// Function with return type pointer: address to a variable returned, might be array
// IMPORTANT: returned reference/address MUST be known outside the scope of the function: either GLOBAL or in-function created but STATIC
unsigned int* return_pointer() {

	// When a variable is STATIC, no matter in which scope it is, it will be like global, i.e. visible in all scopes of the code
	static unsigned int x = 5;
	return (&x);

}

// Function with return type reference: used on left hand side to assign values!
// IMPORTANT: returned reference MUST be known outside the scope of the function: either GLOBAL or in-function created but STATIC
float& assign_reference(int i) {

	// A reference is returned, but you dont need to put '&'' here - it is implicitly done, as in pass-by-reference
	return g_array[i];

}



// Arrays and character sequences
void arraysAndCharacterSequences() {

	//:: Arrays ::

	// Array: series of elements of the same type stored contiguously in memory
	int billy[5]; // array of 5 uninitialized elements - size must be constant if declared like this!
	int sally[5] = {1, 2, 3, 4, 5}; // initialized
	int sammy[] = {1, 2, 3, 4, 5}; // if initialized, size can be left empty

	// Access and assignment
	billy[0] = 1; // first index 0
	billy[1] = billy[0] + 1;
	billy[2] = 3;
	billy[3] = 4;
	billy[4] = 5;
	//billy[5] = 6; // bad memory access! not necessarily compilation error (syntactically not incorrect), but can cause runtime error

	// Multi-dimensional arrays: they are really an abstraction, since [n][m] is equivalent to [n * m]
	float matrix[3][3];

	// Arrays as parameters of functions
	// Not possible to pass entire blocks of memory, but their address! -> much faster and efficient
	float my_array[10];
	functionWithArray(my_array, 10); // Pass the length of the array as argument too

	//:: Character Sequences ::

	// Use better std::string - however, learn how arrays of characters work

	char my_text[20]; // we can store sequences up to 20 chars, but shorters also - finish sequence with null character \0
	my_text[0] = 'H';
	my_text[1] = 'e';
	my_text[2] = 'l';
	my_text[3] = 'l';
	my_text[4] = 'o';
	my_text[5] = '\0'; // null character: it means sequence finishes here, although there is space for more in array

	char my_word[] = {'H','e','l','l','o','\0'};
	// this is identical as to declaring it with string literals, as follows:
	char my_other_word[] = "Hello"; // null character \0 is automatically included
	const char* my_exra_word = "Hello"; // null character \0 is automatically included; use 'const char*' when initializing with string literal

	// IMPORTANT:
	// 		- when initializing with string literals, the array of characters has constant length
	// 		- you can change content of cells separately, but not redefine it with a string literal!
	// 		- every time you use a character array, you need to somehow specify its length: with [length] or, implicitly, when initializing it with a string literal
	// 		- cin, cout support character arrays with \0 at the end

	// Convert to std::string
	std::string my_string;
	my_string = my_word;

	std::cout << my_string << " " << my_word << std::endl;

}

void pointersAndDynamicMemory() {

	// Memory: succession of 1-Byte cells, each one numbered/with address one unit bigger than its predecessor
	// Differentiate
	// 		- Content stored in memory
	// 		- Address to the memory location: this is a POINTER

	// Operators * (DEFERENCE) and & (REFERENCE)
	int age_andy = 25;
	int* pointer_age_andy; // declared POINTER (type*): variable contains an ADDRESS, in this case, to an integer
	pointer_age_andy = &age_andy; // REFERENCE (&variable): ADDRESS of a memory cell that contains an int is passed
	int age_sammy = *pointer_age_andy; // DEFERENCE (*variable): CONTENT targeted by the pointer that has the address of a int cell is passed
	
	int age_tommy = 35;
	int* pointer_age_tommy = &age_tommy; // pointer/address to age_tommy
	pointer_age_tommy = pointer_age_andy; // pointer_age_tommy has now the address to age_andy, not to age_tommy
	*pointer_age_tommy = 30;

	age_andy++; // All variables pointing to will access to the updated value: 30 + 1

	// These are true:
	if (	(age_andy == *pointer_age_andy)
			&& (pointer_age_tommy != &age_tommy)
			&& (*pointer_age_tommy == age_andy)
			&& (age_sammy == 25)
			&& (age_andy == 31))
		std::cout << "Pointer check correct" << std::endl;


	char* p_char;
	bool* p_bool;
	int* p_int;
	float* p_float;
	double* p_double;

	std::cout 	<< "Sizes (in Bytes) of pointers: "
				<< " char " << sizeof(p_char)
				<< " bool " << sizeof(p_bool)
				<< " int " << sizeof(p_int)
				<< " float " << sizeof(p_float)
				<< " double " << sizeof(p_double) << std::endl;

	// Notes:
	// 		- Pointer is declared with: type*, e.g., int*, float*
	// 		- Size of a pointer (depends on machine/architecture?): 8 Bytes (Macbook Air - 10.14.1 Mojave 64B, LLVM 10.0.0, clang-1000.11.45.5, x86_64-apple-darwin18.2.0)
	//		- Although pointers occupy same space in memory, their pointed/target content does not!
	//		- Reference & is not used in declarations, except in function definitions (pass by reference)

	int *p_0; // same as int* p_0
	int *p_1, *p_2; // if 1+ pointers are created, write always * before variable name

	//Arrays are essentially pointers (+ size) - but the address they point to is CONSTANT / IMMUTABLE
	int numbers[10]; // numbers is the pointer to &numbers[0], and its pointing address is IMMUTABLE - thus, CONSTANT
	int* p; // p is a pointer whose address is MUTABLE / VARIABLE / NON-CONSTANT

	if (numbers == &numbers[0])
		std::cout << "Array name is pointer to its first value" << std::endl;

	// Operations with pointers and arrays
	// - [] is DEFERENCE operator for arrays, aka OFFSET operator
	// - Pointer arithmetics: only + and -
	// - Adding an integer to a pointer means advancing one step to next memory cell (subtraction: analogous: backwards)
	// - Each step size is the size of the stored type
	p = numbers; // p is now pointing to &numbers[0]
	*p = 1; // p[0] = numbers[0] = 1
	p++; *p = 2; // p[1] = numbers[1] = 2
	*(p + 1) = 3; // p[2] = numbers[2] = 3
	*(numbers + 3) = 4; // numbers[3] = p[3] = 4
	p = numbers + 2; // p[0] = numbers[2]

	std::cout << "p[0] = " << p[0] << ", p[1] = " << p[1] << std::endl;

	// Character strings/sequences
	const char* my_word = "Hello"; // null character \0 is automatically included; use 'const char*' when initializing with string literal
	if (&my_word[0] == my_word)
		std::cout << "Character strings/sequences are arrays of characters: pointers with constant address and a fixed length" << std::endl;

	// PRECEDENCE PRIOROTIES: BE CAREFUL!!
	int *q, *r;
	int n[] = {1, 2, 3, 4, 5};
	q = n;
	r = q; // q = r = n

	if (*q++ == n[0]) {
		std::cout << "++ has precedence over *: *q++ == *(q++) != (*q)++" << std::endl;
		std::cout << "++, if applied AFTER a variable, increases value AFTER its expression is evaluated:" << std::endl;
		std::cout << "\t*q++ = *r++" << std::endl;
		std::cout << "\tis roughly equivalent to" << std::endl;
		std::cout << "\t*p = *r; ++q; ++r;" << std::endl;
	}

	// Pointers to pointers
	char a;
	char* b;
	char** c;
	a = 'z';
	b = &a;
	c = &b;

	// void pointers and functions with generic parameters
	// - pointers that point to elements that have no assigned type
	// - no deference (access through *) possible with them, because type size is unknown!
	// - they need to be casted before accessing to the memory they are targeting
	// - use: generic parameters for functions usable with different argument types

	char my_char = 'a';
	int my_int = 1;
	float my_float = 1.0;

	std::cout << "Value of my_char: " << my_char;
	increase_generic(&my_char, sizeof(my_char));
	std::cout << " -> " << my_char << std::endl;

	std::cout << "Value of my_int: " << my_int;
	increase_generic(&my_int, sizeof(my_int));
	std::cout << " -> " << my_int << std::endl;

	// Null pointer: it not the void pointer, but a pointer targeting nowhere
	int *null_pointer;
	null_pointer = 0; // it has no valid address, it is as if were initialized with no real value or a TBD value

	// Pointers to functions: callback functions
	// - when functions are passed as arguments to other functions (callback)
	// - passed functions are defined with normal prototypes
	// 		e.g.
	// 			int addition(int a , int b);
	//			int substraction(int a , int b);
	// - function which accepts pointer to function (callback) needs the prototype definition as argument
	//		e.g.
	//			int operation(int x, int y, int (*function_call)(int, int))
	// - when using, function name is passed, without *, nor return type, nor argument types
	//		e.g.
	//			int z = operation(x, y, addition);

	int x = 1;
	int y = 2;
	int z = operation(x, y, addition);

	std::cout << "operation(" << x << ", " << y << ", addition) = " << z << std::endl;

	// You can also rename/redefine a pointer to a function
	int (*minus)(int, int) = substraction;
	z = operation(y, x, minus);

	std::cout << "operation(" << y << ", " << x << ", substraction) = " << z << std::endl;

	// operators new and new[] - ONLY C++
	// - these allow dynamic memory allocation: amount of memory necessary defined at runtime / by user input
	// - in contrast, regular arrays require to specify size when programming
	// - with 'new', memory to allocate (which is a limited!) is taken from heap
	// 		- if there is a problem, a bad_alloc exception is thrown and program terminates
	//		- it is possible to catch that exception (see below)
	// - due to limitation of memory, after using 'new', then you need to free dynamically allocated memory with 'delete'

	int *my_number_array; // first pointer declaration
	int *my_number; // first pointer declaration

	my_number = new int; // for allocating one int - my_number has now a new address!	
	my_number_array = new int[5]; // for allocating an array of 5 ints (5 not necessarily constant) - my_number_array has now a new address!

	// operators delete and delete[] - ONLY C++
	// - due to limitation of memory, after using 'new', then you need to free dynamically allocated memory with 'delete'
	// - if a null pointer is passed to delete nothing happens
	delete [] my_number_array; // for arrays: delete [] pointer_name
	delete my_number; // for single values: delete pointer_name

	// if you want to check if new worked correctly (good practice)
	// - add header <new>: #include <new>
	// - use label (std::nothrow) for new: new (std::nothrow) type
	// - check if newly allocated pointer is the null pointer (if so, something went wrong)

	int* my_variable;

	my_variable = new (std::nothrow) int; // if allocation problems, program doesnt terminate
	if (my_variable == 0) {
		// null pointer => there was an allocation problem with my_variable
		std::cout << "There was an allocation problem with new!" << std::endl;
	} else {
		// NOT null pointer => my_variable was correctly allocated
		std::cout << "There was NO allocation problem with new!" << std::endl;
		// PROCEED AS NORMAL WITH my_variable		
	}

	// Dynamic Memory as in C
	// - 'new' and 'delete' are exclusively from C++
	// - in C, you can use: malloc, calloc, realloc, free
	// - C functions are available in C++ through '#include <cstdlib>'

	//:: MATRIX: DYNAMIC CREATION, MULTIPLICATION, DELETION

	float **matrix_A, **matrix_B, **matrix_C;

	int rows = 3;
	int columns = 3;
	int common = rows;

	// A: rows x common
	// B: common x columns
	// C: rows x columns

	matrix_A = new float*[rows];
	matrix_B = new float*[rows];
	matrix_C = new float*[rows];
	
	for (unsigned int i = 0; i < rows; ++i){
		matrix_A[i] = new float[columns];
		matrix_B[i] = new float[columns];
		matrix_C[i] = new float[columns];
	}

	// Fill in matrices
	for (unsigned int r = 0; r < rows; ++r){
		for (unsigned int c = 0; c < columns; ++c){
			matrix_A[r][c] = 1.0;
			matrix_B[r][c] = 1.0;
			matrix_C[r][c] = 0.0;
		}
	}

	// Multiply
	for (unsigned int r = 0; r < rows; ++r){ // row constant r
		for (unsigned int c = 0; c < columns; ++c){ // vary column c
			matrix_C[r][c] = 0.0;
			for (unsigned int i = 0; i < common; ++i) { // vary common length i
				matrix_C[r][c] += matrix_A[r][i] * matrix_B[i][c]; // A fixed row r, B fixed column c 
			}
		}
	}

	// Output
	std::cout << "Matrix = [ ";
	for (unsigned int r = 0; r < rows; ++r){ // row constant r
		for (unsigned int c = 0; c < columns; ++c){ // vary column c
			if ((r == rows - 1) && (c == columns - 1)) {
				std::cout << matrix_C[r][c];				
			} else {
				std::cout << matrix_C[r][c] << ", ";				
			}
		}
	}
	std::cout << " ]" << std::endl;

	// Delete pointers
	for (unsigned int r = 0; r < rows; ++r){
		delete [] matrix_A[r];
		delete [] matrix_B[r];
		delete [] matrix_C[r];
	}

	delete [] matrix_A;
	delete [] matrix_B;
	delete [] matrix_C;


}

void otherDataStructuresAndTypes() {

	// NOTE: '_t' ending can be used after a new data structure to make clear its a type

	//:::: structs

	// structs
	// - grouped elements of different type in one compound structure; elements called members
	// - similar to classes, with less functionalities, e.g. all members/methods public
	// - new types are created when defining them; sometimes, '_t' ending is given, to make clear it's a new type
	// - '_t' ending appears also with typedef, union, enum
	// - use ';' always after a struct / enum / union declaration

	struct movie {
		std::string title;
		int year;
	};

	// new type: a struct definition creates a new type
	movie film_1, film_2;
	
	// access / assignment to members
	film_1.title = "Matrix";
	film_1.year = 1999;
	film_2.title = "Arrival";
	film_2.year = 2017;

	// new objects can be optionally created/instantiated directly when defining struct
	struct fruit {
		float weight;
		float price;
	} apple, banana, melon; // instantiated objects of struct fruit; optional

	// use-case: databases, by creating arrays of a struct
	movie filmoteque[10];
	filmoteque[0].title = "Volver";
	filmoteque[0].year = 2010;

	// pointers to structures
	// - treated as pointers to regular types
	// - difference: members can be accessed with operator '->' (deference for struct/class object pointers)
	//		pointer_object->member == (*pointer_object).member
	movie* p_movie;
	p_movie = &film_1;
	std::cout << "Title of my first movie: " << p_movie->title << std::endl;
	p_movie = &film_2;	
	std::cout << "Title of my second movie: " << (*p_movie).title << std::endl;

	// nesting and functions: structs can be nested and have member functions (methods)
	struct director {
		std::string name;
		int number_of_movies;
		movie first_movie;
		void print() {
			std::cout << name << " has directed " << number_of_movies << " movies; " << first_movie.title << " was his first." << std::endl;
		};
	};

	director dir_1;
	dir_1.name = "Mikel";
	dir_1.number_of_movies = 1;
	dir_1.first_movie.title = "Panamericana";
	dir_1.first_movie.year = 2020;
	dir_1.print();

	//:::: typedefs: it creates alias or synonyms for existing data types: typedef existing_type new_type_name
	// - '_t' ending can be used to make clear its a type; also with struct, union, enum

	typedef double real; // if float/double should be decided later
	//typedef float real;
	typedef char field[50]; // if arrays typedefed

	real r = 1.3;
	field greeting = "Hello!";

	std::cout << greeting << " " << r << std::endl;

	//:::: unions
	// - All elements of a union share SAME physical memory
	// - Use cases: 1) unite an array of structures of smaller elements, 2) anonimous unions
	// - Watch out: you can use any of the types to access/assign it, but once chosen one for an instance, stick to it!
	// - '_t' ending can be used to make clear its a type; also with typedef, struct, enum

	// 1) smaller structures united
	union mix_t {
		long l; // each of the members has same size: 4 Bytes
		struct {
			short hi; // 2 Bytes
			short lo; // 2 Bytes
		} s; // 4 Bytes
		char c[4]; // 4 Bytes
	} mix; // 4 Bytes

	// 2 ) anonymous unions: if a union is declared without a name
	struct book{
		char title[50];
		char author[50];
		union {
			float dollars;
			int yens;
		}; // 4 Bytes - union has no name, its members are part of the struct directly, but with variable type of same size
	} my_book;

	my_book.dollars = 12.99;
	//my_book.yens = 150; // We could have assigned yens INSTEAD of dollars

	std::cout << "Price of the book: " << my_book.dollars << std::endl; // if yens assigned, wrong float output
	//std::cout << "Price of the book: " << my_book.yens << std::endl; // if dollars assigned, wrong int output

	//:::: enums
	// - New discrete data types with categorical values
	// - Each category has an int value really, starting usually with 0 for the first category
	// - Access tests work with ints, but assignments with categories
	// - Output is always the int of the category 
	// - '_t' ending can be used to make clear its a type; also with typedef, struct, union

	enum colors_t {
		black, // 0: if not explicitly specified otherwise, first category has int value 0, and then ++
		blue, // 1
		green, // 2
		cyan, // 3
		red, // 4
		purple, // 5
		yellow, // 6
		white // 7
	};

	// instances, access / assign
	colors_t my_color;
	my_color = green;
	if (my_color == green) {
		my_color = red;
		std::cout << "My color is " << my_color << std::endl;
	} else if (my_color == 3) { // 3: cyan - access tests work with ints, but assignments with categories
		my_color = white; // 7: white		
	}

	enum intensity_t {
		low = 1, // first value explicitly given; rest ++
		medium, // 2
		high // 3
	};

	intensity_t power;
	power = medium;
	if (power > 1) { // access tests work with ints, but assignments with categories
		std::cout << "You have a considerable intensity" << std::endl;
	}

}

void pointerAndReferenceReturnTypes() {

	// IMPORTANT: return pointer/reference: ONLY with GLOBAL or STATIC return values (known outside the scope of the function)
	// Use cases:
	// - pointer: obtain address to a static/global array or variable/class-object
	// - reference: assign (function on left) to a static/global array or variable/class-object
	// General useful guidelines
	// - I would avoid the return of pointers/references unless with the aforementioned 2 use cases
	// - I would use pass-by-reference to modify content of variables created outside of the scope of the function

	// Function with return type pointer: address to a variable returned, might be array
	unsigned int* my_counter = return_pointer(); // address to in-function created static x returned

	std::cout << "Counter = " << *my_counter << std::endl;

	// Function with return type reference: used on left hand side to assign values!
	assign_reference(1) = 10.0; // global array cell g_array[1] modified

	std::cout << "g_array[] = " << g_array[0] << ", " << g_array[1] << ", " << g_array[2] << std::endl;

}

