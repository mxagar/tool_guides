/**
*\file objectCode.h
*\author Mikel Sagardia
*\date 2018
*/

#define PI 3.14159

// ::: Function prototypes (they dont belong to any class)

void test_classes_basic(void); // general class definition, access/assignment, (default) constructors & destructors, pointers and arrays
void test_classes_intermediate(void); // operator overloading, this, static members
void test_classes_advanced(void); // friends, inheritance, polymorphism: virtual methods and abstract classes

// ::: Forward declarations

// Forward declaration of class: since CRactangle appears in prototype before defining it
class CRectangle;
// Forward declaration of class: since CSquare appears in CRectangle
class CSquare;

// ::: Friends

// friend function: declared outside the class but has access to all class members
// it is not part of the class, but its prototype is added to the class
// prototype goes also to the top of the code (header), as with normal functions
CRectangle duplicate(CRectangle& rectangle);

// ::: Class Definitions: General Uses: OOP basics, friends, operators

class CRectangle {

private:
	int m_x, m_y;

public:
	// Constructors (NOTYPE ClassName): called automatically when object is created: (MEMORY) INITIALIZATION
	CRectangle() {m_x = 0; m_y = 0;};
	CRectangle(int x, int y); // constructors can be overloaded like normal functions
	// Destructor (NOTYPE ~ClassName); called automatically when object is destroyed: FREE MEMORY / TIDY UP
	~CRectangle();

	// set() and get() functions
	void setValues(int x, int y);
	int getX(void) {return (m_x);}; // simple methods can be implemented in class definition - treated as 'inline' by compiler
	int getY(void) {return (m_y);};

	int area(void);

	// friend function: declared outside the class but has access to all class members
	// function is declared outside as if it had access to its members and a prototype (with 'friend') is added to class
	// prototype goes also to the top of the code (header), as with normal functions
	// whenever possible, use member functions instead of friends - e.g., this could be a member function
	// using friends is not OOP
	// use case of members: a function needs access to members of several classes
	friend CRectangle duplicate(CRectangle& rectangle);

	// use of friend class
	// This function implementation cannot go in header because CSquare is declared after CRectangle
	void convert(CSquare& s);
	//void convert(CSquare& s) {m_x = s.m_side; m_y = s.m_side;};

}; // Optional: objects of class to be created, e.g.: } my_rectange;


class CSquare {
private: 
	int m_side;
public:
	void setSide(int side) {m_side = side;};
	// friend class: CRectangle has now access to all members of CSquare (not the other way around!)
	// if a class has a friend class, the use of this friendship is executed in the other class - maybe a better token than 'friend' would have been 'allow_access_to'
	friend class CRectangle;	
};

class CVector {

private:
	float m_x, m_y, m_z;

public:

	// number of created objects of class CVector
	// this must be initialized as global with the type, right above main():
	// unsigned int CVector::N = 0
	static unsigned int N;

	CVector(float x, float y, float z);
	CVector() {m_x = 0.0; m_y = 0.0; m_z = 0.0; ++N;}; // Since we have a custom constructor, we also need to declare default ClassName() constructor
	void setValues(float x, float y, float z);
	void getValues(float& x, float& y, float& z);
	float length(void);
	float dot(CVector& v);
	CVector cross(CVector& v);
	CVector operator+ (CVector& v);
	CVector operator- (CVector& v);
	CVector operator* (float scalar);
	float operator[] (int i);
	float& operator() (int i); // Watch out: return type reference: assign function (function on left)	
	CVector& operator= (CVector& v); // Assignment operator - usually not needed, because it's done automatically by compiler

	// check whether v is this object itself; this pointer used
	bool isItMe(CVector& v);
	//bool operator== (CVector& v);

};

// ::: Class Definitions: Inheritance

class CShape {
protected: // only protected and public members can be inherited!
	float m_size;
public:
	CShape() {m_size = 0.0; std::cout << "CShape()" << std::endl;}; // will always be called by children unless another overloaded constructor is specified
	CShape(float size) {m_size = size; std::cout << "CShape(float)" << std::endl;};
	void setSize(float size) {m_size = size;};
	void getSize(float& size) {size = m_size;};
};

class COutput { // for multiple inheritance
public:
	void output(float value) {std::cout << value;};
};

class CSphere: public CShape { // accessible members (public & protected) inherited from CShape + kept with their access specifier
public:
	CSphere() {m_size = 0.0; std::cout << "CSphere()" << std::endl;}; // calls automatically to CShape()
	CSphere(float size): CShape(size) {m_size = size; std::cout << "CSphere(float)" << std::endl;}; // automatic call to CShape() replaced by call to CShape(float)
	//CSphere(float size) {m_size = size; std::cout << "CSphere(float)" << std::endl;}; // calls automatically to CShape()
	float volumen(void) {return ((4.0 / 3.0) * PI * m_size * m_size * m_size);};
};

class CCube: protected CShape, public COutput { // multiple inheritance: CCube has members from CShape (set to be max. protected) and COutput
public:
	CCube() {m_size = 0.0; std::cout << "CCube()" << std::endl;}; // calls automatically to CShape()
	//CCube(float size): CShape(size) {m_size = size; std::cout << "CCube(float)" << std::endl;}; // automatic call to CShape() replaced by call to CShape(float) 
	CCube(float size) {m_size = size; std::cout << "CCube(float)" << std::endl;}; // calls automatically to CShape() 
	float volumen(void) {return (m_size * m_size * m_size);};
};

// ::: Class Definitions: Polymorphism

class CVehicle {
protected:
	float m_max_speed;
public:
	CVehicle() {m_max_speed = 0.0;};
	void setMaxSpeed(float speed) {m_max_speed = speed;};
	virtual float getHighwaySpeed(void) {return (m_max_speed * 0.1);}; // virtual function: function that can be re-defined in derived child class
	virtual float getSnowSpeed(void) = 0; // pure virtual function = virtual function without implementation (= 0) -> ABSTRACT class = INTERFACE -> non-instantiable
	void printHighwaySpeed(void) {std::cout << "\thighway speed " << this->getHighwaySpeed();}; // polymorphism: base class makes derives to its (unknown) children implementation through this
};

class CCar: public CVehicle {
public:
	float getHighwaySpeed(void) {return (m_max_speed * 0.75);};
	float getSnowSpeed(void) {return (m_max_speed * 0.35);};
};

class CBus: public CVehicle {
public:
	float getHighwaySpeed(void) {return (m_max_speed * 0.5);};
	float getSnowSpeed(void) {return (m_max_speed * 0.20);};
};


/*

General Notes on Classes:
- Object-Oriented Programming: data is structured complex in classes which embed member variables (information) and methods to process that information
- Classes are structs with extended characteristics:
	- they hold member variables (attributes) and methods/functions
	- they can be inherited to form new classes
	- each member is ordered under an access specifier: private, protected, public
		- private (default, if nothing specified): members accessible ONLY from INSIDE the class or their FRIENDS
			- for private member variables, set() and get() methods are often implemented
			- private specifier is to avoid misuse of data
		- protected: like PRIVATE, but members also accessible from members of DERIVED classes
		- public: members accessible from ANYWHERE, even from outside		
- An object is an instantiation of a class; a class is a new type, user-defined
- Use ';' always after a class definition - BUT method implementations dont need ';'
- About the member variables and methods
	- accessed/assigned with '.' with instantiated objects, with '->' if pointers to an object
	- operator scope '::' used in function/method implementation: type ClasName::method_name(...) {...}
	- function/method implementation can be in class definition or somewhere else (e.g., same file, or another file)
		- if implemented in definition, compiler considers it 'inline'
- Constructors and Destructors
	- Constructors
		- Same name as class, NO TYPE
		- Can be overloaded
		- Initialization
		- Dynamic memory allocation with 'new'
	- Destructors
		- Same name as class preceded by ~, NO TYPE
		- Dynamic memory freed up here with 'delete'
	- Default: 4 things are done automatically by the compiler if programmer doesnt take care of them
		- If nothing explicitly declared/defined, compiler implements implicitly
			- (NOT ALWAYS) A default constructor ClassName(): THIS IS AKA DEFAULT CONSTRUCTOR OR CONSTRUCTOR WITHOUT PARAMETERS
				Only if no other custom constructor is declared/defined; if a custom constructor is declared/defined, this default must be declared/defined manually
				Example: ClassName::ClassName() {m_variable = 0;}
			- (ALWAYS) A default destructor ~ClassName()
			- (ALWAYS) A default copy constructor that copies all member variables: ClassName c1; ClassName c2(c1);
				Example: ClassName::ClassName(const ClassName& rv) {m_variable = rv.m_variable;}
			- (ALWAYS) A default copy assignment operator (=) that copies all member variables: ClassName c1; ClassName c2 = c1;
				Example: ClassName& ClassName::operator= (const ClassName& rv) {m_variable = rv.m_variable; return *this;}
		- If any default constructor/destructor is explicitly declared/defined, its implicit default definition/implementation is not performed by the compiler
		-> IMPORTANT:
			- if we create a custom constructor, we need to create also the default ClassName() constructor (at least, try to initialize variables with 0)
			- it is not necessary to do that for the copy / assignment constructor
- Operators can be overloaded to perform class specific tasks
	- Prototype syntax: return_type operator sign (parameters) {implementation}
	- Basically it consists in substituting 'function_name' by 'operator sign' and add always '(parameters)'
	- Parameters refer to the ones on the right
	- Parameter must go in (), although no () used later (e.g., operator +)
	- Example: v_1 + v_2 == v_1.operator+(v-2) (and can be used like that)
	- Overloadable operators: +, -, *, /, >, <, [], (), ->, +=, ..., <<, >>, %, ^, !, ..., new, delete, ...
- this == pointer to the object itself
	- Use cases:
		1) Check is passed object is the object itself
		2) Implement the default copy assignment operator (=):
				ClassName& ClassName::operator= (const ClassName& rv) {m_variable = rv.m_variable; return *this;}
- static member of a class = class variable, same for all objects of that class: global within the class
	- Use case: counter of the objects of a class
	- Cannot be initialized within the class, because otherwise it's initialized every time we create an object
	- They are initialized at the beginning of the code, like a global, outside the class (and type is included):
		Example: unsigned int ClassName::n = 0
	- static members can be accessed from the object or the class
		ClassName c; a.n; ClassName::n;
	- static functions are possible also: analogous to static member variables
		- they can use only static member variables
		- they cannot use this
		- use case: process static data?
- Friends
	- friend function: declared/defined outside the class but has access to ALL class members (including private)
		- function is declared/defined outside as if it had access to its members and a prototype (with 'friend') is added to class
		- prototype goes also to the top of the code (header), as with normal functions
		- forward declaration of used class might be necessary before header prototype
		- whenever possible, use member functions instead of friends - e.g., this could be a member function
		- using friends is not OOP
		- use case of members: a function needs access to members of several classes
	- friend class: analogous: class A {friend class B;} -> B has now access to all the members of A (not the other way around!)
		- if a class has a friend class, the use of this friendship is executed in the other class - maybe a better token than 'friend' would have been 'allow_access_to'
		- if we want 2 classes to be friends of each other, we need to explicitly do that: 
			class A {friend class B;}
			class B {friend class A;}
		- class friends are not transitive: the friend of a friend is not considered a friend, unless explicitly specified
		- forward declarations are usually necessary
		- methods that make use of friend class might need to be implemented in source code (not header), because friend interface might be declared after the class using it
		- use case: convert or merge
- Inheritance: children classes created comfortably to replicate certain members of base classes
	- How to do that (example: CShape base, CSphere child):
		class CShape {...}; -> class CSphere: public CShape {...};
			- CShape should contain common members to all its children
			- the access specifier 'public' can be replaced by 'private' or 'protected': all inherited members will be forced to have this maximum access specifier
				e.g.; 'public': no change (most common); 'protected': all 'public' will become 'protected'; 'private': all will become 'private'
			- if no access specifier given, compiler assumes PRIVATE for classes, PUBLIC for structs
	- What is inherited
		- all PUBLIC and PROTECTED members; NOT PRIVATE members -> base classes rarely have private members
		- inherit could be interpreted more like 'grant access' rather than 'copy member'
	- What is NOT inherited
		- PRIVATE members; use PROTECTED if you want inheritable private members
		- Base class default CONSTRUCTOR and DESTRUCTOR;
			- VERY IMPORTANT BUT THESE DEFAULT BASE CONSTRUCTOR/DESTRUCTORS ARE CALLED WHEN INSTATIATING/DESTROYING CHILD OBJETC
			- IF YOU WANT A SPECIFIC BASE CONSTRUCTOR (AND NOT THE DEFAULT) TO BE CALLED, SPECIFY IT IN CHILD CONSTRUCTOR DEFINITION
				class CSphere: public CShape {...};
					in CSphere:
						CSphere(float size) {...}; // will call default CShape()
						CSphere(float size): CShape(size) {...}; // will call CShape(float)
		- Base class FRIENDS
		- Assign operator (=) members
	- Multiple inheritance
		- Put all desired base classes in commas:
			class CCube: protected CShape, public COutput {...};
- Usually:
	- In header (.h), class structure declared: member variables + member function prototypes
	- In source file (.cpp), implementation of memeber functions done: type CClasName::method_name(...) {...} 
	- Source file which implements class declared in a header needs to include that header
		- Exception: forward declaration
- Useful conventions
	- Class names often are chosen to start with capitals or even capital C: class Rectangle, class CRectangle
	- Class member variables are often chosen to start with m_
- Note that 'class' is equivalent to 'struct' except that structs have public members per default (classes private)
	- structs can have also member functions
	- unions can also have member functions; all members are public in them

POLYMORPHISM
- Key idea: A pointer to a derived class is type-compatible with a pointer to its base class
	- Polymorphism consists in using this idea
	- Strategy:
		- Base class is instantiated as a pointer with address to a child class
		- This pointer can be passed to functions without type conflicts
		- Since pointer targets concrete child classes, specific child methods are accessible!
- Virtual functions (of a base class)
	- Functions that can be re-defined in derived child classes
	- For creating them, just precede function prototype with keyword 'virtual'
	- IMPORTANT: Keyword 'virtual' allows a member of a derived class with the same name as in base class to be called from the pointer of a base class
	- Pure virtual functions
		- Virtual functions that have no implementation and are re-defined in derived children
		- For creating them: 'virtual function_prototype = 0;'
		- A base class with at least one pure virtual function is a ABSTRACT CLASS or an INTERFACE: it cannot be instantiated as an object, only as a POINTER

ADDITIONAL GLOSSARY
- Data encapsulation = basically working with classes, in which data and functions are bound together and different access categories are declared (private, protected, public) to avoid misuse
- Data abstraction = exposing interfaces and hiding implementation
- Interface = Abstract class = base class with at least one pure virtual function that can be instantiated only as a pointer
- Polymorphic class = (base) class that inherits a virtual function (not pure virtual)
- declaration = prototype, definition = implementation
	- Declaration = prototype: form of the method, type of variable
	- Definition = implementattion: coding of the algorithm/commands performed by the method

*/

