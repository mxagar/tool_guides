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

- **FIFO** array/list: First in, First Out; it mimicks waiting in a line
- Abstract Data Type (ADT) description of a Queue, all $O(1)$:
    - `create`: create an emty queue
    - `push`: add data to the back of the queue
    - `pop`: remove data from the top of the queue
    - `empty`: return true if queue is empty
- A queue can be implemented with an undelying array or list:
    - Array implementation
        - We need to keep track of an index of where we should be removing from the queue
        - Adding elements is pushing to the array
        - The first elements of the array have the priority to be popped
            - Although be might not pop, but just modify the front index
        - All operations are $O(1)$: `create`, `push`, `pop`, `empty`
            - But: `push` and `pop` are $O(1)$ in amortized runtime, assuming capacity is doubled when exceeded
    - List implementation
        - We insert upfront at $O(1)$
        - We store a new `tail_` pointer which is pointing to the last element
        - For the queue implementation we build a doubly linked list: elements have **two pointers: the one of the next element and the one of the previous element**
        - Every time we `push`, we add before the `head_` by using the forward pointer
        - Every time we `pop`, we remove and update the `tail_` by using the backward pointer
        - All operations are $O(1)$: `create`, `push`, `pop`, `empty`


There is a `std::queue<type>` in the STL. Example from `queue/main.cpp`:
```c++
#include <queue>
...
std::queue<std::string> q;
q.push( "Orange" );
q.push( "Blue" );
q.push( "Illinois" );
// Print the front of the queue out and pop it off:
std::cout << "First pop(): " << q.front() << std::endl; // Orange
q.pop(); // Orange removed
q.push( "Illini" );
std::cout << "Second pop(): " << q.front() << std::endl; // Blue
```

### 1.6 Stack (Data Structure)

- **LIFO** array/list: Last In First Out; it mimicks a pile of dishes/papers
- Abstract Data Type (ADT) description of a Sueue, all $O(1)$:
    - `create`: create an emty queue
    - `push`: add data to the top of the stack
    - `pop`: remove data from the top of the stack
    - `empty`: return true if stack is empty
- A stack can be built with any type of collection, array/list:
    - Array-based
        - We insert bacwards from n to 0; when we arrive 0, the array is expanded
        - A counters/index is kept for the last inserted element position
        - If we pop, the element on the front (index previous to the insert conter) is removed
    - List-based: it's even easier
        - We insert at the front with `push`,
        - We remove from the frot with `pop`, and update the `head_` pointer
        - Nothing new/additional is required
    - All operations are $O(1)$: `create`, `push`, `pop`, `empty`
        - But for the array-based version, `push` and `pop` are $O(1)$ in amortized runtime, assuming capacity is doubled when exceeded

There is a `std::stack<type>` in the STL. Example from `stack/main.cpp`:
```c++
#include <stack>
...
std::stack<std::string> s;
s.push( "Orange" );
s.push( "Blue" );
s.push( "Illinois" );
// Print the front of the stack out and pop it off:
std::cout << "First pop(): " << s.top() << std::endl; // Illinois
s.pop(); // Illinois removed
s.push( "Illini" );
std::cout << "Second pop(): " << s.top() << std::endl; // Illini
```

### Week 1 Assignment: Linked Lists and Merge Sort

See `week_1_assignment/README.md`.

#### `const` correctness

```c++
// if a function name is continued by const
// it implies no data is changed within it
bool LinkedList<T>::empty() const { return !head_; }

// here, we deliver a link/reference to an internal value
// note that we deliver the link to the internal variable and we can change it!
T& LinkedList<T>::front() { return head_->data; }

// here, we pass a link/reference to a function
// with const, we force it to remein unchanged
void function(const T& data);
// without const, we could change data inside
void function(T& data);

// in case we want to pass to another function a LInkedList as const reference: const LinkedList<T>& myList
// we need to overload LinkedList's (public) member functions
// with a const function + const return version
const T& LinkedList<T>::front() const { return head_->data; }

// Always use const after the function name if it does not change anything!
```

#### Insertion and Merge Sort

Insertion Sort: `O(n^2)`

```pseudocode
a <- {a0, a1, a2, ...} // a[0] == a0
i <- 1
while i < length(a)
    x <- a[i]
    j <- i - 1
    while j >= 0 and a[j] > x
        a[j+1] <- a[j]
        j <- j - 1
    end
    a[j+1] <- x
    i <- i + 1
end
```

Merge Sort, `O(n*log(n))`: efficient sorting algorithm, from Wikipedia:

1. Divide the unsorted list into n sublists, each containing one element (a list of one element is considered sorted).
2. Repeatedly merge sublists to produce new sorted sublists until there is only one sublist remaining. This will be the sorted list.

## Week 2: Binary (Search) Trees

Until now we have studied flat data structures consisting of lists or sequences of elements.
In contrast, hierarchical data structures like trees allow having elements with relationships to each other.

### 2.1 Tree Terminology

- Elements are **nodes** (often depicted as circles); the **root node** is the first node
- Nodes are connected by **edges**: arrows that connect parant nodes with children nodes
    - The root node has no parents (ie., incomming edges)
- Nodes that have no children are called leaf nodes; they could be at any level
- Nodes contain often the data
- Edges are often not labelled and contain no data
- Ancestry terms apply to trees: siblings, ancestors, granchildren, grandparent
- **Three conditions** must be true for a tree: **rooted, directed and acyclic**
    - Must have a root
    - Must have directed edges
    - Must not have a cycle

### 2.2 Binary Trees

A binary tree is a tree where **every node has at most two children**. One child is the **left child** and the other is the **right child**.

Properties of a binary tree:

- **Height** (`h`) of a binary tree: number of edges in the *longest path from the root to the leaf*
- A binary tree is **full** iff every node has either zero or two children.
- A binary tree is **perfect** iff all interior nodes have two children and all leaves are at the same level.
- A binary tree is **complete** iff the tree is perfect until the last level (before the leaves) and all leaf nodes on the last level are pushed to the left; that means that there could be missing some leaves on the right until a node from which we have leafs to the left. Note that complete trees are not necessarily full, or vice-versa.

Definition of a binary tree in `binary-tree/BinaryTree.h`:
```c++
template <typename T>
class BinaryTree {
  public:
    // ...
  private:
    class TreeNode {
      public:
        T & data;
        TreeNode* left;
        TreeNode* right;
        TreeNode(T & data) : data(data), left(nullptr), right(nullptr) { }
    };
    TreeNode *root_;
};
```

### 2.3 Tree Traversals

A traversal consists in visiting all the nodes of the tree and accessing their values.
It is different to a search: the search does not have to visit all nodes, just the necessary ones until the sought one is found.
A tree traversal can be done in many ways, depending what we'd like to prioritize.
In general, we want to have different ways in which the data in the nodes is accessed, aka. shouted.

Summary of basic traversals for BTs:
- `preOrder`: depth first, shouting/displaying current node first, then the children
- `inOrder`: depth first, shouting/displaying current node between the children
- `postOrder`: depth first, shouting/displaying current node last, after the children
- `levelOrder`: breadth first, each level completely one after the other

Example: `preOrder` traversal: starting with the root node, traverse all nodes by
1. shouting (displaying or accessing/using) the value of the **current** node and the
2. shouting the one on the **left** and
3. shouting the one on the **right**.

```c++
// preOrder traversal: shout current, go left, go right
template<class T>
void BinaryTree<T> preOrder(TreeNode* current) {
    if (current != nullptr) {
        shout(current); // shout = display node's stored value
        preOrder(current->left); // recursive call
        preOrder(current->right);
    }
}
```

Example: `inOrder` traversal: go left, shout current, go right:
```c++
// inOrder traversal: go left, shout current, go right
template<class T>
void BinaryTree<T> inOrder(TreeNode* current) {
    if (current != nullptr) {
        inOrder(current->left); // recursive call
        shout(current); // shout = display node's stored value
        inOrder(current->right);
    }
}
```

Example: `postOrder` traversal: go left, go right, shout current:
```c++
// postOrder traversal: go left, go right, shout current
template<class T>
void BinaryTree<T> postOrder(TreeNode* current) {
    if (current != nullptr) {
        inOrder(current->left); // recursive call
        inOrder(current->right);
        shout(current); // shout = display node's stored value
    }
}
```

### 2.4 Binary Search Trees (BST)

A Binary Search Tree (BST) is an ordered binary tree capable of being used as a search structure. A binary tree is a BST if for every node in the tree:

- nodes in the **left** are **less** than iteself
- nodes in the **right** are **greater** than iteself

If we apply recursively this definition, we can store data that can be searched very quickly, for instance a `Dictionary` which associates `keys` with `values`:

- email/username: profile
- phone number: record
- url: webpage
- address: home

Note that the nodes contain the `key` values as well as a pointer to the `value`; we omit the `value` for our purposes.

`Dictionary` Abstract Data Type (ADT):

- `find()`: given a `key`, find and return its `value`
- `insert()`: given a `key:value` pair, insert is properly into the dictionary
- `remove()`: remove a `key`
- `empty()`: is the dictionary empty?

Dictionary structure `bst/Dictionary.h`:
```c++
template <typename K, typename D>
class Dictionary {
  public:
    Dictionary() : head_(nullptr) { }
    const D& find(const K& key);
    void insert(const K& key, const D& data);
    const D& remove(const K& key);
    bool empty() const;

  private:
    class TreeNode {
      public:
        const K& key;
        const D& data;
        TreeNode* left;
        TreeNode* right;
        // Note nodes have nullptr value for left/right
        TreeNode(const K& key, const D& data)
          : key(key), data(data), left(nullptr), right(nullptr) { }
    };
    TreeNode *head_;
    ///...
}
```

Intuition of the `find()` function:

- We have a value `v` we want to find.
- We start with the root: is `v` the root, smaller, greater? Answer determines: finish, go left, go right.
- Process repeats with current node until either: we find the node or we reach a leaf without finding it.
- Worst-case: visiting the longest path, i.e., if `h` is the height: `O(h)`. Note that the very worst case for `h` is th esituation in which our tree is a linked list, i.e., we have only left/right nodes; in that case `h` = `n` -> `O(n)`.

Implementation of `find()` in `bst/Dictionary.hpp`:
```c++
// Public function we call from outside
// It calls our helper function _find
template <typename K, typename D>
const D& Dictionary<K, D>::find(const K& key) {
  TreeNode*& node = _find(key, head_);
  if (node == nullptr) { throw std::runtime_error("error: key not found"); }
  return node->data;
}
...
// Helper function _find
// A node alias is returned
template <typename K, typename D>
typename Dictionary<K, D>::TreeNode*& Dictionary<K, D>::_find(const K& key, TreeNode*& cur) const {
  // In case leaf reached
  if (cur == nullptr) { return cur; }
  // If key found
  else if (key == cur->key) { return cur; }
  // Recursion left/right
  else if (key < cur->key) { return _find(key, cur->left); }
  else { return _find(key, cur->right); }
}
```

Intuition of the `insert()` function:

- We basically `find()` the `key` we want to insert in th etree until we find the leaf.
- We insert the `key` left or right to the detected leaf.

Implementation of `insert()` in `bst/Dictionary.hpp`:
```c++
template <typename K, typename D>
void Dictionary<K, D>::insert(const K& key, const D& data) {
  // _find 
  // if leaf reached, its left/right node pointer is returned
  TreeNode *& node = _find(key, head_);
  // new node is created on the heap and its pointer stored in the returned/aliased pointer 
  node = new TreeNode(key, data);
}
```

Intuition of the `remove()` function:

- We basically `find()` the `key` we want to insert in th etree until we find the node. The node can be
    1. leaf (= no children) or
    2. somewhere in a middle level with one child or
    3. two children or the root node
- The cases (1) and (2) are straightforward: we remove the node (case 1) and re-link the nodes as in a linked list if necessary (case 2).
- The case (3) is more complicated because we need to find a best new node. **The best cadidate for that is the node which has the closest `key` to the node we are removing**. That is called the **In-Order Predecessor** or IOP:
    - That IOP node is the previous node to the one we are eliminating in an in-order traversal, i.e., a traversal of nodes with ascending `keys`.
    - It turns out that **the IOP node is always the rightmost node of the nodes's left sub-tree**, so
        - We go to the left tree
        - We take always the right node until we reach our leaf
        - We swap the IOP and the node we are removing
        - We delete the new leaf as in case 1

Implementation of `remove()` in `bst/Dictionary.hpp`:
```c++
// Public interface
// Note that we return the data, an alias to it??
template <typename K, typename D>
const D& Dictionary<K, D>::remove(const K& key) {
  TreeNode*& node = _find(key, head_);
  return _remove(node);
}
// Helper
template <typename K, typename D>
const D& Dictionary<K, D>::_remove(TreeNode*& node) {
  // Zero child remove
  if (node->left == nullptr && node->right == nullptr) {
    const D& data = node->data;
    delete node;
    node = nullptr;
    return data;
  }
  // One-child (left) remove
  else if (node->left != nullptr && node->right == nullptr) {
    const D& data = node->data;
    TreeNode* temp = node;
    node = node->left; // node pointer is replaced by left child's
    delete temp; // node is deleted from heap
    return data;
  }
  // One-child (right) remove
  else if (node->left == nullptr && node->right != nullptr) {
    // This case is symmetric to the previous case.
    const D& data = node->data;
    TreeNode* temp = node;
    node = node->right; // node pointer is replaced by right child's
    delete temp; // node is deleted from heap
    return data;
  }
  // Two-child remove
  else {
    TreeNode*& iop = _iop_of(node);
    TreeNode*& moved_node = _swap_nodes(node, iop);
    return _remove(moved_node);
  }
}
```

### 2.5 BST Analysis

