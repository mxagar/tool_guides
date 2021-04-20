# Ordered Data Structures in C++

Notes made when following the course offered in Coursera by University of Illinois at Urbana-Champaign.
Prof. Wade Fagen-Ulmschneider. This course is 2/3 in the 'Accelerated Computer Science Fundamentals Specialization'.

## Week 1: Linear Structures

### 1.1 Arrays

- An array stores data sequentially: elements are stored one after the other, increasing the memory addresses (thus, also indices) one unit at a time
- Limitations of arrays
    - All elements of the same type
        - The size of the type is known (in bytes); that makes possible to calculate the offset to any given index. Thus, we don't need to navigate through all elements ($O(n)$), but we have direct access to each element ($O(1)$)
    - Arrays have a fixed capacity
        - Capacity: maximum number of elements
        - Elements are stored sequentially in a memory block
        - If we exceed the capacity, we need to allocate a new larger chunk of memory and we need to copy the data over from the previous block to the new
        - However, the `std::vector` does that automatically; therefore, for the user, an `std::vector` is an array which dynamically gorws in size, eventhough internally resizing and memory allocation happpens if we exceed the initial capacity

Examples `array/ex2/main.cpp` and `array/ex3/main.cpp`:
```c++
// Create an array of 10 numbers
int values[10] = { 2, 3, 5, 7, 11, 13, 15, 17, 21, 23 };

// Print the size of each type `int`
std::cout << sizeof(int) << std::endl; // 4 (bytes)

// Compute the offset from the beginning of the array to [2]
// &[0] points to the start or index 0
// &[2] points to the start or index 2
// between indices 0 and 2 there are 2 cells -> 2 x 4 bytes
int offset = (long)&(values[2]) - (long)&(values[0]);
std::cout << offset << std::endl; // 8 = 2 x 4 bytes

// Similarly with any other structures
Cube cubes[3] = { Cube(11), Cube(42), Cube(400) };
std::cout << sizeof(Cube) << std::endl; // 8 bytes
int offset = (long)&(cubes[2]) - (long)&(cubes[0]); // 2 x 8 bytes
```

Example `array/ex4/main.cpp`: `std::vector`:
```c++
// Examine capacity, push_back = append, size
std::cout << "Initial Capacity: " << cubes.capacity() << std::endl; // 3
cubes.push_back( Cube(800) );
std::cout << "Size after adding: " << cubes.size() << std::endl; // 3
std::cout << "Capacity after adding: " << cubes.capacity() << std::endl; // 3x2 = 6, it is doubled!

// Compute the offset from the beginning of the array to [2] and [3]
std::cout << "Size of cube element: " << sizeof(cubes[0]) << std::endl; // 8 bytes
int offset1 = (long)&(cubes[2]) - (long)&(cubes[0]); // 2 x 8
int offset2 = (long)&(cubes[3]) - (long)&(cubes[0]); // 3 x 8 -> they are sequantially stored!
```

### 1.2 Lists = Linked Memory

- Linked memory lists store data elements together with a link/pointer to the location in memory of the next list node. So basically we have
    - **List nodes**: element (data) + pointer to next node
    - **Linked list** itself: set of list nodes; a head pointer (e.g., `head_`) marks the memory location where the list begins, `nullptr` marks the end of the list (the pointer to the next element of the last element is `nullptr`)

A **list node** is defined with data + pointer to the next node, as shown in `linked-memory/List.h`:
```c++
template <typename T>
class ListNode {
public:
    const T & data;
    ListNode *next;
    ListNode(const T & data) : data(data), next(nullptr) { }
};
```

A **linked list** is as shown in `linked-memory/List.h`, here simplified:
```c++
template <typename T>
class List {
  public:
    const T & operator[](unsigned index); // access elements with []
    void insertAtFront(const T & data);

  private:
    class ListNode {
      public:
        const T & data;
        ListNode *next;
        ListNode(const T & data) : data(data), next(nullptr) { }
    };

    ListNode *head_;  // head pointer    
};
```

- Differences wrt. arrays:
    - Now, if want the element in `[4]` we need to visit all previous elements before it, in chain. So here we have $O(n)$ complexity.
    - Now we can insert an element between other two elements. Usually, new elements are inserted at the front, since that requires basically creating a new node and changing the head pointer.
    - In a list, the capacity is bounded to the available (heap) memory of the system; we don't need to resize as we needed with the array
    - Both arrays and lists contain elements of the same type
    - Summary: lists are more flexible than arrays, but with runtime disadvantages


How to access/return a list element, as shown in `linked-memory/List.hpp`:
```c++
template <typename T>
const T & List<T>::operator[](unsigned index) {
    // We step starting in head pointer index times 
    ListNode *thru = head_;
    while (index > 0 && thru->next != nullptr) {
        thru = thru->next;
        index--;
    }  
    return thru->data;
}
```

How to insert a new list element, as shown in `linked-memory/List.hpp`:
```c++
template <typename T>
void List<T>::insertAtFront(const T & data) {
    // Create a new ListNode on the heap
    // bacause the node needs to live beyond the scope of this function
    ListNode *node = new ListNode(data);
    // Set the new node’s next pointer point the current
    // head of the List = the current first node
    node->next = head_;
    // Set the List’s head pointer to be the new node
    head_ = node;
}
```
