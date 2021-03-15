/**
*\file dynamicCode.h
*\author Mikel Sagardia
*\date 2018
*/


//:: Function prototypes

void test_template_functions(void);
void test_template_classes(void);
void test_namespaces(void);
void test_exceptions(void);
void test_casting(void);
void test_file_io(void);


//:: Function templates

// - Function templates work for several types: we dont have to repeat same code for each type
// - Template parameters (= abstract or generic types) are passed as arguments in function call
// - Compiler basically replaces the generic type
// - Function prototype: template <class T> return_type function_name(int a, float b, T c, ...) {implementation with generic type T};
// 		- T is a generic type, which can have any name, e.g. myTpe
// - Function call: function_name<float>(a, b, c, ...)
//		- T is specified by user to be float

template <class T> // type T passed later in source code, when calling function
T GetMax(T a, T b) {T result; result = (a > b ? a : b); return (result);}; // function prototype as always, but with generic type T if it applies
template <typename T> // typename | class can be used indistinctly for functions
T GetMin(T a, T b) {return (a < b ? a : b);}; 
// function call: float max = GetMax<float>(a, b);
// possible function call if all parameter types one-to-one determine T type: float max = GetMax(a, b);

// More than one generic type
template <class U, class V>
U GetMin(U a, V b) {return (a < b ? a : b);};
// function call still possible without <>, because all parameter types one-to-one determine generic types U, V

//:: Class templates: analogous to function templates: use generic type T + add 'template <class T>' + ClassName<my_types>

template <class T>
class mypair {
private:
	T m_a, m_b;
public:
	mypair(T a, T b) {m_a = a; m_b = b;};
	T getMax(void); // LOOK IMPLEMENTATION: preceded by 'template <class T>' + ClassName<T>
};

// Template implementations should go in the header file, where the declarations are
// Reason: they are compiled on demand
// - No code is generated until a template is instantiated when required
// - Compilers are prepared to allow inclusion of the same template several times!
// This separate implementation in the header shows how the syntax is in cases in which declaration is separated from implementation
template <class T> // Implementation of template member functions: preceded by 'template <class T>' + ClassName<T>
T mypair<T>::getMax() { // Watch out: no '<class T>', but '<T>'
	T r;
	r = m_a > m_b ? m_a : m_b;
	return (r);
}

// Template specialization
// - If we want to implement a different/optimized implementation for a concrete type
// - The whole class needs to be declared and implemented, all its members
// - Precede with: 'template <> class ClassName<my_type> {...}'

template <class T>
class myContainer {
private:
	T element;
public:
	myContainer(T arg) {element = arg;};
	T increase() {return ++element;};
};

template <>
class myContainer<char> {
private:
	char element;
public:
	myContainer(char arg) {element = arg;};
	char increase() {return uppercase();};
	char uppercase() {
		if (element >= 'a' && element <= 'z') {
			element += 'A' - 'a'; // make upper case if it is not
		}
		return (element);
	}
};


// Non-type parameters for templates
// - Instead of a type, a value with fixed type can be passed to the template
// - Use case: containers of a type with a LENGTH (length is the non-type passed)
// - It is possible to set default values in the definition
//		template <class T = char, int N = 10> class mySequence {...};
// - Then, instantiation would be
// 		'mySequence<> my_sequence;'s

template <class T, int N>
class mySequence {
private:
	T memblock[N];
public:
	void setMember(int i, T value);
	T getMember(int i);
};

template <class T, int N>
void mySequence<T, N>::setMember(int i, T value) { // Watch out: no '<class T>', but '<T>'
	memblock[i] = value;
}

template <class T, int N>
T mySequence<T, N>::getMember(int i) {
	return (memblock[i]);
}

//:: Namespaces

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

namespace mik {
	
	int a, b;

	class MyClass {
	private:
		int m_a;
	public:
		void set(int a) {m_a = a;};
		int get(void) {return (m_a);};			
	};

}

namespace sag { // Use-case: when an object or a function has a name that is probably already taken

	int a, b;

}

namespace mik { //Namespaces can be defined in part in a file and extended in another part

	int c, d;

}


/*

GLOSSARY / NOTES
- Generic programming = programming with types to-be-declared-later = in C++ templates = parametrized types in Design Patterns

*/