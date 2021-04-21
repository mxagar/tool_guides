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

### 1.3 Run Time Analysis: Access, Insert

- Run time analysis compare speed of an algorithm as the size of the data grows
    - Big-O notation: the most dominant term when counting the number of operations
- Access of elements in arrays and lists
    - Arrays: access to index `i`
        - `offset = sizeof(element) x i`
        - $O(1)$: one operation, no matter the size; that is called constant time
    - Lists: access to index `i`
        - We need to traverse `i` one by one
        - $O(n)$: n operations; that is called linear time
- Resizing of arrays (if capacity = size, ie., array full)
    - Everytime we exceed capacity, we need to allocate a new memory block and copy all the data there
    - If we keep on extending the array later in time, we're going to copy the data several times!
    - Strategy 1: we append the initial the capacity `c` is exceeded (eg., `c = 2`)
        - The number of copies required is quadratic (= polynomial) in the number of elements: $O(n^2)$
        - First time `r = 1`, we copy `c = 2 = r`
        - Then `r = 2`, we copy `2c = r*c`
        - Then `r = 3`, `3c = r*c`
        - If `r = n/2`, being `n` the final total number of elements
        - The total number of copies is `sum(c, 2c, ...) = c*sum(1, 2, ..., r) = c*r(r+1)/2 = r^r + r`
        - Since `r = n/2` -> $O(n^2)$ !!
    - Strategy 2: we double the size/capacity when we hit the limit
        - The number of copies required is linear in the number of elements: $O(n)$
        - First time we do `c = 2` copies, then `2c`, `2*2c`, `2*2*2c`, ...
        - Total copies = `sum(2, 4, 6, ..., 2^r) = 2(2^r - 1)`
        - We know: `c^r = n` -> `r = log2(n)`
        - Therefore: `2(2^log2(n) - 1) = 2(n-1)` -> $O(n)$ < $O(n^2)$ !!
        - **Thus, doubling the capacity every time we hit the limit leads to a linear effort of copies**
        - However, the average insertion time is $O(1)$, because 
            - We very rarely copy the entire array when inserting a new element
            - For every copying effort of $O(n)$ we have inserted `n` elements without copy
            - Therefore, the average insertion effort is $O(n)/n = O(1)*$ !!
            - That is called **amortized running time**
        
### 1.4 Array and List Operations: Find, Insert After

- Summary of complexities so far
    - **Access** index `i`
        - Array: $O(1)$: direct access via offset
        - List: $O(n)$: must traverse all nodes until `i`
    - **Insert** at the front/back
        - Array: $O(n)$ (if copy due to capacity exceeded), $O(1)*$ (amortized)
        - List: $O(1)$: new node at the head
- What about **finding** data?
    - In both, in the data is *unsorted*, we need to traverse all elements until finding the one: $O(n)$
    - However, we can improve that for the arrays if the data is *sorted*
        - Say we want to find the cube with length 17
        - We start in the middle of the container
        - Has the element a larger length? If so, we continue in the first part, otherwise in the second
        - This way, we check only half of the branch each time: `log2(n)` checks performed: $O(log2(n))$
        - This is called **binary serach**
    - For the lists, no binary search is possible: all lists have $O(n)$ find complexity
- Last operation: **insert after**
    - Given an index `i`, insert element fater it
    - With an array we need to move all data after `i`; worst case, we need to move all `n` elements -> $O(n)$
    - With a list, we just need to change the pointer of element `i` -> $O(1)$

- Conclusions
    - Arrays and linked memory lists are the basis for other data structures built on top of them
    - Depending on what we want to achieve (which operations is critical), we choose the array or the list as basis
    - Similarly, we consider to sort or not

### 1.5 Queue (Data Structure)



### 1.6 Stack (Data Structure)

