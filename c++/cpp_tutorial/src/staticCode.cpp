/**
*\file staticCode.cpp
*\author Mikel Sagardia
*\date 2018
*/

#include <iostream>
#include "staticCode.h"

// STL headers
#include <vector> // vector
#include <list> // list
#include <map> // map, multimap
#include <set> // set
#include <stack> // stack
#include <queue> // queue
#include <deque> // deque

// Avoid writing std:: all the time
using namespace std;

void test_stl(void) {

	cout << "--- Testing STL ---" << endl;

	// vector: re-sizeable container: iterators
	test_stl_vector();

	// list: re-sizeable container, elements can be added anywhere, not only at end (as in vector)
	test_stl_list();
	
	// map: key-value pairs, like a look up table/dictionary, list of unique keys
	test_stl_map();

	// multimap: a map with with duplicate keys
	test_stl_multimap();

	// set: stores unique elements
	test_stl_set();

	// stacks and queues: LIFO and FIFO collections
	test_stl_stack_queue();

	// sort (vector)
	test_stl_sort();

	// deque: double ended queue: similar to a vector, but we can additionally push_front() and pop_front() 
	test_stl_deque();

	// container nesting: combining different container types 
	test_stl_container_nesting();

}

void test_stl_vector(void) {

	// vector: re-sizeable container
	
	cout << "-> vector" << std::endl;

 	vector<string> strings(5); // [0, 1, 2, 3, 4]

	strings[3] = "dog"; // [-, -, -, "dog", -]
	strings.push_back("one"); // [-, -, -, "dog", -, "one"]
	strings.push_back("two"); // [-, -, -, "dog", -, "one", "two"]

	cout << strings[3] << endl;
	cout << strings.size() << endl; // 7

	// traditional traverse: NOT recommended
	for (unsigned int i = 0; i < strings.size(); ++i) {
		cout << "strings[" << i << "] = " << strings[i] << endl;
	}

	// ITERATOR traverse: RECOMMENDED, faster
	vector<string>::iterator it = strings.begin(); // pointer to first cell
	cout << "it = " << *it << endl;

	for (	vector<string>::iterator it = strings.begin();
			it != strings.end();
			++it // pointer increased: next cell
			) {
		cout << "strings[] = " << *it << endl;
	}

	// Iterator arithmetics: you can move
	it = strings.end();
	--it;
	--it;
	it -= 2;
	it += 3;
	cout << "it = " << *it << endl;

	// Advantages of iterator vs normal index
	// - 'const interator' ensures elements cannot be changed
	// - maybe more efficient?

	// Memory issues
	// - vector stores an array internally
	// - if we push_back, the capacity usually is bigger than the size
	// - if vector is increased past its capacity
	//		- a new array is created
	//		- content copied to it: this is time expensive!
	//		- capacity is doubled: this can be memory extensive
	// - Why all this? To avoid re-allocation issues

	vector<double> numbers(20, 1); // every element initialized with 1
	cout << "size(numbers) = " << numbers.size() << endl;
	unsigned int capacity = numbers.capacity();
	cout << "capacity(numbers) = " << capacity << endl;

	for (unsigned int i = 0; i < 10000; ++i) {
		if (capacity != numbers.capacity()) {
			capacity = numbers.capacity();
			cout << "i = " << i << endl;
			cout << "capacity = " << capacity << endl;
		}
		numbers.push_back(i);
	}

	numbers.resize(100); // size set to 100, capacity unchanged, contents untouched!
	cout << "size(numbers) = " << numbers.size() << endl;
	cout << "capacity(numbers) = " << numbers.capacity() << endl;
	cout << "numbers[50] = " << numbers[50] << endl;

	numbers.clear(); // size set to 0, capacity unchanged
	cout << "size(numbers) = " << numbers.size() << endl;
	cout << "capacity(numbers) = " << numbers.capacity() << endl;

	numbers.reserve(100); // capacity increased if bigger value than current given - here no effect
	cout << "size(numbers) = " << numbers.size() << endl;
	cout << "capacity(numbers) = " << numbers.capacity() << endl;
	numbers.reserve(20000); // capacity increased if bigger value than current given - here increased
	cout << "capacity(numbers) = " << numbers.capacity() << endl;

	// vectors of vectors: use initialization parameters for sizes
	vector< vector<int> > grid(3, vector<int>(4, 7)); // vector of 3 x 4 ints, initialized with 7

	// It is possible to modify the size of a row
	grid[1].push_back(8);

	cout << "grid[][] = " << endl;
	for (unsigned int i = 0; i < grid.size(); ++i) {
		for (unsigned int j = 0; j < grid[i].size(); ++j) {
			cout << grid[i][j] << " ";
		}
		cout << endl;
	}

}

void test_stl_list(void) {

	// list: re-sizeable container, elements can be added anywhere, not only at end (as in vector)
	// list: nodes linked to each other with pointers - no indeces possible
	// vector: elements stacked one after the other
	// as in vectors, in lists objects are UNORDERED

	cout << "-> list" << std::endl;

	list<int> numbers;

	numbers.push_back(1);
	numbers.push_back(2);
	numbers.push_back(3);
	
	// we can add in the front
	numbers.push_front(0);
	
	// we can add in the middle
	list<int>::iterator it = numbers.begin();
	++it; ++it; // += doesnt work...
	numbers.insert(it, 10);


	// not possible to use indices to traverse, only iterators
	for (list<int>::iterator it = numbers.begin(); it != numbers.end(); ++it) {
		cout << *it << endl;
	}
	cout << "-" << endl;

	// we can remove from the middle
	it = numbers.begin();
	++it; ++it; // += doesnt work...
	it = numbers.erase(it); // a new it is returned pointing to the node after it, since a node ans it links/pointers were destroyed
	cout << *it << endl;

	cout << "-" << endl;

	for (list<int>::iterator it = numbers.begin(); it != numbers.end(); ++it) {
		cout << *it << endl;
	}
	cout << "-" << endl;


}

void test_stl_map(void) {

	// map: key-value pairs, like a look up table/dictionary, list of unique keys
	// objects are ordered automatically (operator< must exist on type)
	map<string, int> ages; // key: string, value: int - keys must be unique!

	cout << "-> map" << std::endl;

	// Items are added every time [] is used
	// If when using [] the key exists, it is modified, not added
	ages["Mike"] = 40;
	ages["Raj"] = 20;
	ages["Vicky"] = 30;
	ages["Mike"] = 70; // entry with key Mike overwritten

	cout << "ages['Raj'] = " << ages["Raj"] << endl;

	// Iterate and access key-value pairs
	for (map<string, int>::iterator it = ages.begin(); it != ages.end(); ++it) {
		// first: key, second: value
		cout << it->first << ": " << it->second << endl;
	}

	// Find key
	if (ages.find("Vicky") != ages.end()) {
		cout << "Vicky found" << endl;
	}

	// map is a container of the structure pair
	pair<string, int> age;
	map<string, int>::iterator it = ages.begin();
	it++;
	age = *it;
	cout << age.first << ": " << age.second << endl;

	// we can insert pairs to a map in several ways
	pair<string, int> insertAge("Peter", 100);
	ages.insert(insertAge); // insert pair object
	ages.insert(pair<string, int>("Sam", 90)); // insert with pair construtor
	ages.insert(make_pair("Jon", 95)); // built-in function

	cout << "ages['Peter'] = " << ages["Peter"] << endl;
	cout << "ages['Sam'] = " << ages["Sam"] << endl;
	cout << "ages['Jon'] = " << ages["Jon"] << endl;

	// maps with custom values and keys
	// - custom values is straightforward
	// - custom keys implies class must be defined accordingly
	// 		- provide all constructors/destructors
	//		- overload operator<
	//		- make all methods const

	class Person {
	public:
		Person(): name("Nobody"), age(0) {};
		Person(string name, int age): name(name), age(age) {};
		Person(const Person &other): name(other.name), age(other.age) {};
		bool operator<(const Person &other) const {
			// const is necessary for use in map as key: keys must be unmutable
			// our logic should take into account all relevant members
			// if we compare only the name, ("Mikel", 90) would be the same pair as ("Mikel", 95) internally in the map
			// -> that means the pair with the first unique name would remain unmutable
			// -> that means we could not add key ("Mikel", 90) if we had ("Mikel", 95) already inside
			if (name == other.name) {
				return (age < other.age);
			} else {
				return (name < other.name);
			}
		}; 
		void print(void) const {
			// const is necessary for use in map as key: keys must be unmutable
			// we mark it const to make sure it wont change the object
			// this is important when coplex object are being used as keys
			//cout << name << ": " << age << endl;
			cout << name << ": " << age << endl;
		}
	private:
		string name;
		int age;
	};

	map<int, Person> people;

	people[5] = Person("Jon", 60);
	people[10] = Person("Mary", 70);
	people[2] = Person("Clark", 80);

	// WATCH OUT / WARNING
	// - We are copying an object of type Person to the elements of map
	// - We usually want to avoid copies of complex objects
	// - Why? Bacause if we have pointers withing the objects, they are copied, and instead we might want to have relative pointers with the class
	// - Solution: assignment operator and copy constructor must be overwritten correctly

	// map always sorts items wrt key values: 2: Clark, 5: Jon, 10: Mary
	for (map<int, Person>::iterator it = people.begin(); it != people.end(); ++it) {
		it->second.print();
	}

	// maps with custom keys
	// - custom keys implies class must be defined accordingly
	// 		- provide all constructors/destructors
	//		- overload operator<
	//		- make all methods const

	// operator '<' must be defined for the key type

	map<Person, int> persons;

	persons[Person("Mikel", 95)] = 95;
	persons[Person("Ana", 96)] = 96;
	persons[Person("Unai", 98)] = 98;
	persons[Person("Mikel", 100)] = 100; // since we overload '<' correctly, this pair is treated as new element in map

	for (map<Person, int>::iterator it = persons.begin(); it != persons.end(); ++it) {
		it->first.print();
	}

}

void test_stl_multimap(void) {

	// multimap: a map with with duplicate keys
	// same header as map
	// objects are ordered automatically (operator< must exist on type)

	cout << "-> multimap" << endl;

	multimap<int, string> lookup;

	lookup.insert(make_pair(30, "Mike"));
	lookup.insert(make_pair(10, "Vicky"));
	lookup.insert(make_pair(30, "Raj")); // we can add pair with same key!
	lookup.insert(make_pair(20, "Bob"));
	lookup.insert(make_pair(40, "Sam"));

	// iterate with iterators
	for (multimap<int, string>::iterator it = lookup.begin(); it != lookup.end(); ++it) {
		cout << it->first << ": " << it->second << endl;
	}

	cout << "-" << endl;

	// find: finds iterator to first element with passed key value
	for (multimap<int, string>::iterator it = lookup.find(30); it != lookup.end(); ++it) {
		cout << it->first << ": " << it->second << endl; // (30, "Mike"), (30, "Raj"), (40, "Sam")
	}

	cout << "-" << endl;

	// equal_range: returns a pair of first and last iterators of elements with given key value
	pair<multimap<int, string>::iterator, multimap<int, string>::iterator> its = lookup.equal_range(30);
	for (multimap<int, string>::iterator it = its.first; it != its.second; ++it) {
		cout << it->first << ": " << it->second << endl; // (30, "Mike"), (30, "Raj")
	}

}

void test_stl_set(void) {

	// set: stores unique elements
	// objects are ordered automatically (operator< must exist on type)

	cout << "-> set" << endl;

	set<int> numbers;

	numbers.insert(50);
	numbers.insert(10);
	numbers.insert(30);
	numbers.insert(20);
	numbers.insert(30); // won't be added, because we have already 30

	// iterate with iterators
	for (set<int>::iterator it = numbers.begin(); it != numbers.end(); ++it) {
		cout << *it << endl;
	}

	// find
	set<int>::iterator itFind = numbers.find(30);

	if (itFind != numbers.end()) {
		cout << "Found " << *itFind << endl;
	}

	// custom objects
	// - similar steps are necessary as with map: constructors, < operator, make all methods const

	cout << "-" << endl;

	class Test {
	private:
		int id;
		string name;
	public:
		Test(): id(0), name("") {};
		Test(int id, string name): id(id), name(name) {};
		Test(const Test &other): id(other.id), name(other.name) {};

		void print() const { cout << name << ": " << id << endl; }; // mark const, as set/iterator requires it
		bool operator< (const Test& other) const { // mark const, as set/iterator requires it
			if (id < other.id) {
				return (true);
			} else {
				if (id == other.id) {
					if (name < other.name) {
						return (true);
					} else {
						return (false);
					}
				} else {
					return (false);
				}
			}
		};

	};

	set<Test> tests;

	tests.insert(Test(10, "Mike"));
	tests.insert(Test(20, "Sam"));
	tests.insert(Test(40, "Sue"));
	tests.insert(Test(30, "Vicky"));
	tests.insert(Test(20, "Sam")); // Won't be added, because we have it already

	for (set<Test>::iterator it = tests.begin(); it != tests.end(); ++it) {
		it->print();
	}

}

void test_stl_stack_queue(void) {

	// stacks and queues: LIFO and FIFO collections
	// stack: LIFO = Last In First Out - iteration doesn't make sense! Image a pile of objects: we put them and take them
	// queue: FIFO = First In First Out - similar to stack, but another order

	class Test {
	private:
		string name;
	public:
		Test(): name("") {};
		Test(string name): name(name) {};
		Test(const Test& other): name(other.name) {};
		~Test() {}; // check how many times it's called...
		void print() const { cout << name << endl; }; // const is necessary
	};

	cout << "-> stack" << endl;

	// LIFO stack
	stack<Test> testStack;

	// Add items: push()
	testStack.push(Test("Mike"));
	testStack.push(Test("Jon"));
	testStack.push(Test("Sue"));

	// How to use it
	// - do not iterate, it doesn't make sense iterating! The only way to iterate is to pop() elements
	// - metaphore: pile of dishes: last dish on pile is first to be washed
	// - get last object with top() and remove it with pop() 
	Test test_1 = testStack.top(); // Last object we added returned: Sue
	test_1.print(); // Sue

	testStack.pop(); // Last object removed (Sue), no return

	Test test_2 = testStack.top(); // Last object we added: Jon
	test_2.print(); // Jon

	cout << "-" << endl;

	// iteration by pop()-ing
	while (testStack.size() > 0) {
		Test test = testStack.top();
		test.print();
		testStack.pop();
	}

	cout << "-> queue" << endl;

	// FIFO queue	
	queue<Test> testQueue;

	// Add items: push()
	testQueue.push(Test("Mike"));
	testQueue.push(Test("Jon"));
	testQueue.push(Test("Sue"));

	// How to use it
	// - do not iterate, it doesn't make sense iterating! The only way to iterate is to pop() elements
	// - metaphore: queue at supermarket: first person to enter is first to exit
	// - get last object with front() and remove it with pop() 
	// - we have method front() instead of top()

	// we have access to front and back of queue
	testQueue.front().print(); // Mike
	testQueue.back().print(); // Sue

	cout << "-" << endl;

	// iteration by pop()-ing
	while (testQueue.size() > 0) {
		Test test = testQueue.front();
		test.print();
		testQueue.pop();
	}


}

void test_stl_sort(void) {

	// sort (vector)
	// - sort(vector.begin(), vector.end(), <comparison_function>)
	// - it is really std::sort
	// - it is in std
	// - applied to vectors
	// - contained type must have operator< implemented
	// - you can pass a comparison function to sort()
	// - it changes the vector on which it is applied by sorting its elements in the especified range

	cout << "-> std::sort(vector)" << endl;

	class Test {
	private:
		string name;
		int id;
	public:
		Test(): id(0), name("") {};
		Test(int id, string name): id(id), name(name) {};
		Test(const Test& other): id(other.id), name(other.name) {};
		~Test() {}; // check how many times it's called...
		bool operator<(const Test& other) const {
			// we implement alpha-numerical ordering
			if (name < other.name) {
				return (true);
			} else if (name == other.name) {
				if (id < other.id) {
					return (true);
				}
			}
			return (false);
		}
		void print() const { cout << name << ": " << id << endl; }; // const is necessary
	};

	vector<Test> tests;

	// Add items: push()
	tests.push_back(Test(10, "Mike"));
	tests.push_back(Test(20, "Jon"));
	tests.push_back(Test(5, "Sue"));
	tests.push_back(Test(30, "Mary"));
	tests.push_back(Test(3, "Clark"));
	tests.push_back(Test(50, "Mike")); // 2nd Mike, but with different id

	// std::sort
	sort(tests.begin(), tests.end()); // begin() and end() provided

	for (vector<Test>::iterator it = tests.begin(); it != tests.end(); ++it) {
		it->print();
	}

}

void test_stl_deque(void) {

	// deque: double ended queue: similar to a vector, but we can additionally push_front() and pop_front()
	// - similar interface as vector
	// - BUT: internally values stored in several chunks or arrays
	// - therefore: they can be more efficient than vectors when reallocation is performed (e.g., with frequent push_backs)
	// - they are not like lists: not all cells are linked to each other, but bigger chunks of values

	cout << "--> deque" << endl;

	deque<int> numbers;

	numbers.push_back(1);
	numbers.push_back(2);
	numbers.push_back(3);
	numbers.push_back(4);
	numbers.push_front(0);
	numbers.push_front(-1);
	numbers.push_front(-2);
	numbers.push_front(-3);

	for (deque<int>::iterator it = numbers.begin(); it != numbers.end(); ++it) {
		cout << *it << endl; // -3, -2, -1, ..., 2, 3, 4
	}

	cout << "-" << endl;

	numbers.pop_back(); // 4
	numbers.pop_front(); // -3

	for (deque<int>::iterator it = numbers.begin(); it != numbers.end(); ++it) {
		cout << *it << endl; // -2, -1, ..., 2, 3
	}

}

void test_stl_container_nesting(void) {
	
	// container nesting: combining different container types 
	// - we can create any kind of complex data structures using different containers
	// - BUT: sometimes it is more useful to create our own type used in the chosen container

	cout << "-> combined containers" << endl;

	// Example: a map of vectors

	map <string, vector<int> > scores; // Students with scores

	scores["Mike"].push_back(10);
	scores["Vicky"].push_back(20);
	scores["Mike"].push_back(30);

	for (	map<string, vector<int> >::iterator it = scores.begin();
			it != scores.end();
			++it	) {

		string name = it->first;
		vector<int> my_scores = it->second;

		cout << name << ": " << flush;

		for (vector<int>::iterator jt = my_scores.begin(); jt != my_scores.end(); ++jt) {
			cout << *jt << " " << flush;
		}

		cout << endl;

	}

}

