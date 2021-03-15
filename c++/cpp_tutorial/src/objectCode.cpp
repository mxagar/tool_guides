/**
*\file objectCode.cpp
*\author Mikel Sagardia
*\date 2018
*/


#include <iostream>
#include <cmath> // sqrt / std::sqrt
#include "objectCode.h" // Add file which contains class definition


// ::: Class Member Implementations


// Constructor (Custom)
CRectangle::CRectangle(int x, int y) {

	m_x = x;
	m_y = y;
	// If memory is allocated dynamically here with 'new', it must be 'deleted' in the destructor

}

// Destructor
CRectangle::~CRectangle() {

	// Any allocated dynamic memory (with 'new') must be freed up here with 'delete'

}

void CRectangle::setValues(int x, int y) {

	m_x = x;
	m_y = y;

}

int CRectangle::area(void) {

	return (m_x * m_y);

}

// Use of friend class
void CRectangle::convert(CSquare& s)
{

	// This function implementation cannot go in header because CSquare is defined after CRectangle
	// We have access in CRectangle to private members of CSquare because CRectangle is friend of CSquare
	m_x = s.m_side;
	m_y = s.m_side;

}


// Constructor
CVector::CVector(float x, float y, float z) {

	m_x = x; m_y = y; m_z = z;
	++N; // increase number of created obejcts

}

void CVector::setValues(float x, float y, float z) {

	m_x = x; m_y = y; m_z = z;	

}

void CVector::getValues(float& x, float& y, float& z) {

	x = m_x; y = m_y; z = m_z;	

}

float CVector::length(void) {

	return (std::sqrt(m_x * m_x + m_x * m_x + m_x * m_x));

}

float CVector::dot(CVector& v) {

	float x, y, z;
	v.getValues(x, y, z);
	return (m_x * x + m_y * y + m_z * z);
	
}

CVector CVector::cross(CVector& v) {

	float x, y, z;
	v.getValues(x, y, z);
	CVector r(x, y, z); // wrong implementation - Cramer must be done...
	return (r);
	
}

CVector CVector::operator+ (CVector& v) {

	float x, y, z;
	v.getValues(x, y, z);
	CVector r(m_x + x, m_y + y, m_z + z);
	return (r);

}

CVector CVector::operator- (CVector& v) {

	float x, y, z;
	v.getValues(x, y, z);
	CVector r(m_x - x, m_y - y, m_z - z);
	return (r);

}

CVector CVector::operator* (float scalar) {

	CVector r(m_x * scalar, m_y * scalar, m_z * scalar);
	return (r);

}

float CVector::operator[] (int i) {

	if (i == 0) return (m_x);
	else if (i == 1) return (m_y);
	else return (m_z);

}

float& CVector::operator() (int i) {
	
	// Watch out: return type reference: assign function (function on left)
	// Although same code as operator[], this function is for assignment!
	// A reference is returned, but you dont need to put '&'' here - it is implicitly done, as in pass-by-reference
	if (i == 0) return (m_x);
	else if (i == 1) return (m_y);
	else return (m_z);

}

CVector& CVector::operator= (CVector& v) {

	// Assignment operator - usually not needed, because it's done automatically by compiler
	// IMPORTANT:
	// - prototype and return is always like this
	// - assignment implementation is basically copy from member of argument to member of current object
	float x, y, z;
	v.getValues(x, y, z);
	m_x = x; m_y = y; m_z = z;
	++N; // increase number of created obejcts - this is the main reason why we need to explicitly implement operator=
	return *this;

}

bool CVector::isItMe(CVector& v) {

	// if (&v == this)
	// 	return true;
	// else
	// 	return false;

	return (&v == this);

}

// friend function: defined outside the class but has access to all class members
// it is not part of the class, but its prototype is added to the class
// prototype goes also to the top of the code (header), as with normal functions
CRectangle duplicate(CRectangle& rectangle) {

	// function is defined outside as if it had access to its members and a prototype (with 'friend') is added to class
	// whenever possible, use member functions instead of friends - e.g., this could be a member function
	// using friends is not OOP
	// use case of members: a function needs access to members of several classes

	// friends have access to ALL memebers!
	CRectangle r;
	//r.setValues(rectangle.getX(), rectangle.getY()); // you can do this, but you can also access pirvate memebers!
	r.m_x = rectangle.m_x;
	r.m_y = rectangle.m_y;
	return (r);

}

// ::: Implementation of function prototypes (they dont belong to the classes)

void test_classes_basic(void) {

	// Classes: instantiation to objects + access/assignment of variables
	CRectangle my_rectangle_1;

	my_rectangle_1.setValues(3, 5);	
	std::cout << "Area of my_rectangle_1: " << my_rectangle_1.area() << std::endl;

	// Pointers to a class: memebers accessed with '->' or '(*).'
	CRectangle* my_rectangle_2;
	my_rectangle_2 = &my_rectangle_1;

	std::cout << "Area of my_rectangle_2 (pointer): " << my_rectangle_2->area() << std::endl; // pointer members accessed with '->'
	std::cout << "Area of my_rectangle_2 (pointer): " << (*my_rectangle_2).area() << std::endl; // or with '(*).'

	// Arrays of class object + default constructors (copy & assignment)
	CRectangle r_1[2];
	CRectangle r_0, *r_2;

	//CRectangle* r_2 = new CRectangle[2];
	r_2 = new CRectangle[2];

	r_0.setValues(2, 3);
	CRectangle r_3(r_0); // default copy constructor

	std::cout << "Area of r_0 = " << r_0.area() << std::endl;
	std::cout << "Area of r_3 = " << r_3.area() << std::endl;

	for (int i = 0; i < 2; ++i) {
		r_1[i] = r_0; // default copy assignment constructor used	
		r_2[i].setValues(i, i * 2);
		std::cout << "r_1[" << i << "].area() = " << r_1[i].area() << "; r_2[" << i << "].area() = " << r_2[i].area() << std::endl;
	}

	delete [] r_2;

}

void test_classes_intermediate(void) {

	// This function uses CVector, which has static variable CVector::N (number of CVectors created)
	// Static variables need to be initialized as global before main(), including the type:
	// unsigned int CVector::N = 0;

	CVector v_1, v_2(1.0, 2.0, 3.0);
	v_1.setValues(4.0, 5.0, 6.0);

	float s = v_1.dot(v_2);
	CVector c = v_1.cross(v_2);

	std::cout << "Dot product: " << s << std::endl;

	//:: Operator overloading
	// 		v_1 + v_2
	//		is equivalent to 
	// 		v_1.operator+(v_2)
	// 		and this last call is actually valid

	CVector v_3 = (v_1 * 5.0) + v_2 - c;

	//:: Return reference == function on left == assignment function
	v_3(0) = 0.0;

	std::cout << "v_3 = [" << v_3[0] << ", " << v_3[1] << ", " << v_3[2] << "]" << std::endl;

	//:: Self check with pointer this
	if (v_3.isItMe(v_3))
		std::cout << "v_3 is itself" << std::endl;

	//:: Access static members of a class
	std::cout << "We created " << CVector::N << " CVector objects so far" << std::endl;

}

void test_classes_advanced(void) {

	//:: Friend function

	CRectangle r_1(2, 3);
	CRectangle r_2 = duplicate(r_1);

	std::cout << "r_2 = " << r_2.getX() << ", " << r_2.getY() << std::endl;

	//:: Friend class

	CSquare s_1;
	s_1.setSide(2);
	CRectangle r_3;
	r_3.convert(s_1);

	std::cout << "r_3 = " << r_3.getX() << ", " << r_3.getY() << std::endl;

	//:: Inheritance: CShape, CSphere, CCube

	CSphere sphere(2.0);
	CCube cube(2.0);
	CSphere sphere_2;
	CCube cube_2;
	CShape shape;

	std::cout << "Volumen of cube " << cube.volumen() << "; of sphere " << sphere.volumen() << std::endl;
	
	// Multiple inheritance tested
	std::cout << "Output of cube ";
	cube.output(10.0);
	std::cout << std::endl;

	//:: Polymorphism: CVehicle, CCar, CBus

	CCar polo, golf;
	CBus irizar, man;

	CVehicle* v[4];
	v[0] = &polo;
	v[1] = &golf;
	v[2] = &irizar;
	v[3] = &man;

	// Polymorphism: We can treat all vehicles (cars and buses) as if they were of the same class :)
	// - The pointer of the base class is type-compatible with the pointers of its derived children
	// - CVehicle is an abstract class (it has at least one pure virtual function), but we can instantiate pointers to it
	// - Virtual getHighwaySpeed() and getSnowSpeed() have their specific implementations in the children classes
	// - When a virtual function of a base class pointer is called, the implementation of the derived children run
	// - Method printHighwaySpeed() from base class (CVehicle) calls implementation in derived children using 'this'

	v[0]->setMaxSpeed(180.0); // polo
	v[1]->setMaxSpeed(200.0); // golf
	v[2]->setMaxSpeed(140.0); // irizar
	v[3]->setMaxSpeed(160.0); // man

	for (unsigned int i = 0; i < 4; ++i) {
		std::cout << "Vehicle " << i << ": highway speed " << v[i]->getHighwaySpeed() << ", snow speed " << v[i]->getSnowSpeed() << std::endl;
		v[i]->printHighwaySpeed(); // polymorphism: method from base class (CVehicle) calls implementation in derived children using 'this'
		std::cout << std::endl;
	}

	// Dynamic memory: also can be used

	CVehicle* ferrari = new CCar;
	
	ferrari->setMaxSpeed(280);
	
	std::cout << "My Ferrari has " << std::endl;
	ferrari->printHighwaySpeed();
	std::cout << std::endl;

	delete ferrari;

}

