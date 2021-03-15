/**
*\file dynamicCode.cpp
*\author Mikel Sagardia
*\date 2018
*/

#include <iostream> // std::cout, etc
#include <exception> // std::exception
#include <typeinfo> // typeid
#include <fstream> // std::fstream: open and write files
#include <sstream> // std::istringstream: convert number <-> string, std::ostringstream: convert number <-> string
#include "dynamicCode.h"

//:: Class templates: implementation of methods -> always in header files!

//:: Test functions that use implemented structures

void test_template_functions(void) {

	// - Function templates work for several types: we dont have to repeat same code for each type
	// - Template parameters (= abstract or generic types) are passed as arguments in function call
	// - Compiler basically replaces the generic type
	// - Function prototype: template <class T> function_name(int a, float b, T c, ...) {implementation with generic type T};
	// 		- T is a generic type, which can have any name, e.g. myTpe
	// - Function call: function_name<float>(a, b, c, ...)
	//		- T is specified by user to be float

	float a_f = 2.5;
	float b_f = 3.3;
	double a_d = 2.5;
	double b_d = 3.3;

	float max_float = GetMax<float>(a_f, b_f);
	float min_float = GetMin<float>(a_f, b_f);

	std::cout << "Generic/Template function: float: min max = " << min_float << ", " << max_float << std::endl;

	double max_double = GetMax<double>(a_d, b_d);
	double min_double = GetMin<double>(a_d, b_d);

	std::cout << "Generic/Template function: double: min max = " << min_double << ", " << max_double << std::endl;

	// If all parameters are of generic type T, compiler understands it and no <T> must be passed

	int a_i = 2;
	int b_i = 1;

	int max_int = GetMax(a_i, b_i);
	int min_int = GetMin(a_i, b_i);

	std::cout << "Generic/Template function: int: min max = " << min_int << ", " << max_int << std::endl;

	// Several generic types: U, V

	long int a_l = 5;
	long int min_long = GetMin<long int, int>(a_l, b_i);

	std::cout << "min_long = " << min_long << std::endl;

	// we can remove <> since it is clear which function it is and the argument types determine one-to-one generic types
	min_long = GetMin(a_l, b_i);

	std::cout << "min_long = " << min_long << std::endl;

}

void test_template_classes(void) {

	// Template classes have generic types defined during compilation

	mypair<float> p_1(2.0, 5.6);
	mypair<float> p_2(3.5, 1.2);

	float max_1 = p_1.getMax();
	float max_2 = p_2.getMax();

	std::cout << "Max value of pairs: " << max_1 << ", " << max_2 << std::endl;

	// Template specialization
	// - If we want to implement a different/optimized implementation for a concrete type
	// - The whole class needs to be defined and implemented, all its members
	// - Precede with: 'template <> class ClassName<my_type> {...}'

	myContainer<int> c_int(2);
	myContainer<char> c_char('b');

	std::cout << "myContainer<int>.increase() = " << c_int.increase() << std::endl;
	std::cout << "myContainer<char>.increase() = " << c_char.increase() << std::endl;

	// Non-type parameters for templates
	// - Instead of a type, a value with fixed type can be passed to the template
	// - Use case: containers of a type with a LENGTH (length is the non-type passed)
	// - It is possible to set default values in the definition
	//		template <class T = char, int N = 10> class mySequence {...};
	// - Then, instantiation would be
	// 		'mySequence<> my_sequence;'

	mySequence<int, 10> numbers;
	for (unsigned int i = 0; i < 10; ++i) {
		numbers.setMember(i, i);
	}
	for (unsigned int i = 0; i < 10; ++i) {
		std::cout << "mySequence<int, 10>[" << i << "] = " << numbers.getMember(i) << std::endl;
	}


}

void test_namespaces(void) {

	// Nemespaces group classes, objects, functions under a name
	// Global scope is divided in different sub-scopes
	// They can be defined in glocal scope
	// They can be defined in part in a file and extended in another part
	// Use-case: when an object or a function has a name that is probably already taken
	// Use: name_namespace::object_name
	// To avoid using scope operator, write before using namespace variables/objects:
	// - 'using namespace name_namespace': block {} in which it is written needs no scope anymore
	// - 'using name_namespace::object_name': block {} in which it is written needs no scope anymore for object_name
	// -> BUT: try to avoid that, since conflicts might arise if object names are already taken; or code becomes confusing
	// Namespace alias can be created: namespace new_name = current_name
	// Typical namespace: std

	mik::a = 1;
	mik::b = 2;

	mik::MyClass my_class;
	my_class.set(mik::a);

	std::cout << "my_class.get() = " << my_class.get() << std::endl;

	sag::a = 3;
	sag::b = 4;

	mik::c = 5;
	mik::d = 6;

	std::cout << "mik::a mik::b mik::c mik::d = " << mik::a << " " << mik::b << " " << mik::c << " " << mik::d << " " << std::endl;
	std::cout << "sag::a sag::b = " << sag::a << " " << sag::b << std::endl;

	{ // block
		using namespace mik;
		using sag::b;
		std::cout << "in namespace mik, a = " << a << std::endl;
		std::cout << "in namespace sag, b = " << b << std::endl;

	}

}

void test_exceptions(void) {

	// Exceptions: way to react to exceptional circumstances (e.g., runtime errors) and transferring control to handlers
	// You might need '#include <exception>'
	// General workflow:
	// - code under inspection inside try block
	// - if error occurs, exception is thrown
	//		- functions throw AUTOMATICALLY exeptions
	//		- we can MANUALLY throw exceptions with 'throw'
	// - handler catches error exception and manages situation; if no exception thrown, catch is ignored
	// - program resumes its execution after handling error
	// try-catch can be nested in try

	try { // Code under inspection inside try block
		throw (20); // if error occurs, int is thrown
	}
	catch (int e) { // handler catches error code (20) and manages situation; if no error thrown, catch is ignored
		// We enter in catch iff error type matches!
		std::cout << "Error " << e << " catched!" << std::endl;
	} // Several handlers can be added in series, but only one will be used, the one that matches with the type
	catch (char e) { std::cout << "char error " << e << std::endl; }
	catch (bool e) { std::cout << "bool error " << e << std::endl; }
	catch (...) { std::cout << "Default: there was an error..." << std::endl; } // (...) catches all tsypes of errors

	/*

	Functions can be not/allowed to throw exceptions by appending 'throw(type)' in their declaration:

	float myfunction(int param); // all exceptions allowed
	float myfunction(int param) throw(int); // only exceptions of type int allowed
	float myfunction(int param) throw(); // no exceptions allowed

	*/

	// Standarn exceptions are of type std::exception and are defined in <exception> header

	try {
		int* my_array = new int[1000000000000000]; // huge array trying to be allocated in heap -> not enough space... -> exception thrown automatically
	} catch (std::exception& e) {
	 	std::cout << "Standard exception: " << e.what() << std::endl; // all std exceptions have a what() function which explains error type
		// Possible standard exception types: we can catch them also
		// 		std::bac_alloc				thrown by new on new allocation failure
		//		std::bad_cast				thrown by dynamic_cast when fails with a referenced type
		//		std::bad_exception			thrown when an exception type doesnt match any catch
		//		std::bad_typeid				thrown by typeid
		//		std::ios_base::failure		thrown by functions in the iostream library
	}
	std::cout << "... but program can continue... " << std::endl;

	// We can also catch a concrete standard exception
	try {
		int* my_array = new int[1000000000000000];
	}	
	catch (std::bad_alloc) {
		std::cout << "std::bad_alloc" << std::endl;
	}

	// We can also derive class exception to create our own exception type with custom what function
	class my_exception: public std::exception { // WATCH OUT: syntax of class and virtual function definition
		virtual const char* what() const throw() {
			return ("My exception happened!");
		}
	} my_exception;

	try {
		throw (my_exception); // Manualy throw my_exception when it is due
	}
	catch (std::exception& e) { std::cout << e.what() << std::endl; }; // catch as exception type

}

void test_casting(void) {

	// Implicit conversion
	// - not explicitly expressed
	// - between fundamental data types
	// - precission can be lost, can be warned by compiler

	int a = 10;
	float b = 1.6;
	a = b; // 1.6 will be converted to 1

	std::cout << a << std::endl;

	// Explicit conversion
	// - 2 methods, both valid: C-like, function like
	// - better to explicitly cast than implicitly
	// - fundamental types and classes - BUT: avoid casting classes like that to prevent runtime errors, unless they are related by inheritance

	a = (int) b; // C-like
	a = int (b); // function-like

	// Why explicit casting sould be avoided with classes:
	class CDummy {
	private:
		int m_a, m_b;
	};

	class CAddition {
	private:
		int m_a, m_b;
	public:
		CAddition(int a, int b) {m_a = a; m_b = b;};
		int add(void) {return (m_a + m_b);};		
	};

	CDummy my_dummy;
	CAddition* my_addition;

	// This is correct syntactically and it will compile
	my_addition = (CAddition*) &my_dummy;

	// BUT: following line can produce runtime error because we are pointing to a CDummy object, which has no add() method... 
	//std::cout << "my_addition->add() = " << my_addition->add() << std::endl;

	// casting with classes is done with following 
	// - dynamic_cast<new_type>(expression)
	// - static_cast<new_type>(expression)
	// - reinterpret_cast<new_type>(expression)
	// - const_cast<new_type>(expression)

	// dynamic_cast<new_type>(expression): to go from derived to bas class pointer
	// - only for pointers and references to objects
	// - always successful when casting from derived to base class; NOT from base to derived!
	// - this cast checks that the conversion can be done
	// - compiler must have runtime type information active
	// - it returns a null pointer if something fails in the conversion (e.g., cast to an incomplete pointer class)
	// - if the conversion is not possible, std::bad_alloc is thrown

	class CBase { protected: int x, y; };
	class CDerived: public CBase { };

	CBase* p_base;
	CDerived* p_derived;

	p_base = dynamic_cast<CBase*>(p_derived); 


	// static_cast <new_type>(expression): to go from base to derived class pointer
	// - conversions of pointers of related classes: from derived to base and from base to derived
	// - no runtime safety check performed, as in dynamic_cast, so less expensive
	// - programmer is responsible for feasibility of casting
	// - also works with no pointer fundamental types

	p_base = static_cast<CDerived*>(p_derived); // Cast works, but pointer wont work
	
	double d = 3.14159;
	int i = static_cast<int>(d);

	// reinterpret_cast<new_type>(expression): to go from any pointer type to any pointer type, even with unrelated classes
	// - conversions of any pointer class to any pointer class
	// - I think this does not make much sense, because you later cant use the casted stuff, although the syntax is correct

	class A {};
	class B {};
	A* class_a;
	B* class_b;

	class_b = reinterpret_cast<B*>(class_a);

	// const_cast<new_type>(expression)
	// - convert a non-const or a const in a const or non-const
	// - use-case: character string literals

	const char* text_1 = "Hello"; // character sequences passed as literal ARE ALWAYS const char*!
	char* text_2;
	text_2 = const_cast<char*>(text_1);
	std::cout << text_2 << std::endl;

	// typeid(variable): information on the type of an object
	// - you need: '#include <typeinfo>'
	// - it returns object 'std::type_info', which
	//		- has overloaded operators == and !=
	//		- function '.name()'
	// - with pointers to classes
	//		- the base class is shown as type if pointer is passed
	//		- the derived class is shown if the (deferenced) *pointer is passed
	// - it throws 'std::bad_typeid' if null pointer or any problem

	int *v1, v2;
	v1 = 0; // null pointer
	v2 = 0; // 0 integer

	if (typeid(v1) != typeid(v2)) {
		std::cout << typeid(v1).name() << " != " << typeid(v2).name() << std::endl;
	}

	// WATCH OUT: on my MacBook Air 2014 it does not work - strange names are output: Pi != i

}

void test_file_io(void) {

	// Test file: share/example.txt

	// #include <fstream>
	// std::fstream read/write from/to files
	//		std::ofstream write on files
	//		std::ifstream read from files

	std::fstream myFileStream;

	myFileStream.open("../share/example.txt"); // if it does not exist, file is created
	myFileStream << "Writing this to the file.\n"; // content written to file
	myFileStream.close(); // close file stream

	// Open mode option:

	// myFileStream.open("../share/example.txt", std::ios::out); // open for writing, but if it exists and has content, nothing done...
	// myFileStream.open("../share/example.txt", std::ios::in); // open only for reading
	// myFileStream.open("../share/example.bin", std::ios::binary); // open in binary mode
	// myFileStream.open("../share/example.txt", std::ios::ate); // set initial position at the end of the file; otherwise, default: beginning
	// myFileStream.open("../share/example.txt", std::ios::app); // together with ios::out; add content starting at the end of the file
	// myFileStream.open("../share/example.txt", std::ios::trunc); // if file exists, open to write, delete content and add new content
	// myFileStream.open("../share/example.bin", std::ios::out | ios::app | ios::binary); // concatenate mode options: write adding at en of file in binary mode
	// default modes, if not mode specified
	//		std::ofstream -> std::ios::out
	//		std::ifstream -> std::ios::in
	//		std::fstream -> std::ios::out | ios::in


	myFileStream.open("./share/example.txt", std::ios::app); // content can be added at the end of file

	if (myFileStream.is_open()) {
		std::cout << "File was correctly opened!" << std::endl;
		myFileStream << "New line appended at the end of the file.\n";
	} else {
		std::cout << "File cannot be opened!" << std::endl;		
	}

	myFileStream.close();

	// :: TEXT FILES	

	// - All files which do not have std::ios::binary mode: All are ASCII files
	// - They store text, if output done, some formatting issues can slip into

	myFileStream.open("./share/example.txt", std::ios::in); // read only

	if (myFileStream.is_open()) {
		std::cout << "Reading file:" << std::endl;
		std::string line;
		while (!myFileStream.eof()) { // not end of file reached
			getline(myFileStream, line); // extract NEXT line from file 
			std::cout << line << std::endl;
		}
		std::cout << "Finished!" << std::endl;
	}

	// Other functions:
	// - bad()
	// - fail()
	// - good()

	// Internal position pointers for reading and writing -> look reference

	// :: BINARY FILES	

	// - All files which do have std::ios::binary mode
	// - Not for text files; no formatting issues if output done
	// - read() and write() functions used -> look reference

	// :: NUMBER <-> STRING CONVERSION

	// This operations can be abstracted to 2 functions:
	//		template <class T>
	//		std::string num2str<class T>(T value);
	//		template <class T>
	//		T str2num<class T>(const std::string& value);
	//		T str2num<class T>(std::string value);
	//		T str2num<class T>(const char* value);


	int number_int = 123;
	std::string number_string_1, number_string_2;
	std::ostringstream convert_o; // input number

	convert_o << number_int;

	number_string_1 = convert_o.str();

	std::cout << "Number conversion: int -> string: " << number_string_1 << std::endl;

	number_string_2 = "456";

	std::istringstream convert_i(number_string_2); // output number

	convert_i >> number_int;

	std::cout << "Number conversion: string -> int: " << number_int << std::endl;

}

