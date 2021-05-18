# Ordered Data Structures: Week 1 Assignment: Linked Lists and Merge Sort

Notes extracted from the instructions and while completing the assignment.
## General instructions

- Compile: `make clean`, `make` and `make test`
- Run: `./main`, `./test`
- Templated C++ code: code in header files, specific version generated at compilation, depending on how templates are used in code
- File to edit: `LinkedListExercises.h`
- Files to read:
    - `LinkedList.h`: `LinkedList` class
    - `main.cpp`: `main`
    - `tests/week1_test.cpp`: configuration of the `test` program
- For submission: `make zip`

## `LinkedList` class

- Doubly-linked list; can be used as doubled-ended queue/stack: push/pop both ends, constant time access to front/back.
- List contains nodes; **nodes are created in the heap**, each node contains `next` and `prev` pointers and also the data stored.
- Class is similar to `std::list`, but not as through: missing `iterator`, memory layer not as abstracted, etc.

## New concepts: `const` and Sorting

See `../README.md`.

- `const` correctness
- Insertion and Merge Sort

## Additional Notes

Structure of the class `LinkedList`:

    LinkedList<T>
        public:
            Node
                public:
                    Node* next;
                    Node* prev;
                    T data;
        private:
            Node* head_;
            Node* tail_;
            int size_;

        public:
            splitHalves() // [1,2,3,4,5] -> [[1,2,3],[4,5]]
            front()
            back()
            explode() // [1,2,3] -> [[1],[2],[3]]
            getHeadPtr()
            size()
            empty()
            popFront()
            popBack()
            pushBack()
            pushFront()
            insertOrdered()
            insertionSort()
            isSorted()
            merge()
            mergeSortRecursive()
            mergeSortIterative()
            mergeSort() // wrapper calling any of the above
            print()
            operators =, ==, !=, <<, ...
            assertCorrectSize()
            assertPrevLinks()

## Exercise 1

In `LinkedListExercise.h`, implement

`void LinkedList<T>::insertOrdered(const T& newData)`

which inserts a new item in a previously sorted `LinkedList` before the earliest
item in the list that is greater.

Steps:
- Create a new node on the heap
- Traverse nodes to find the next greater
- Insert in found place (could be also at front=head/back=tail)
- Update pointers (prev/next & head/tail if necessary)
- Update `size_`

Notes:
- No `push`, `pop`
- Tips: check if list empty, insert at begining, reached end?
- Handle the `nullptr` correctly: do no de-reference it!

## Exercise 2

In `LinkedListExercise.h`, implement

`LinkedList<T> LinkedList<T>::merge(const LinkedList<T>& other) const`

which combines (without modifying) two sorted lists (`*this` and `other`) into a new one, returned.

```c++
LinkedList<int> leftList; // fill in
LinkedList<int> rightList; // fill in
LinkedList<int> mergedList = leftList.merge(rightList);
```

Notes:
- It should run `O(n)`, so we cannot append and then sort
- Assume extreme cases too: empty lists, different lengths, repeating elements
- We can make copies inside, if needed
- We can/should use the member functions, eg., `push`
- Update all pointers and `size_`
- Do not call any sorting function
- Do not call `insertOrdered`
- Since both lists are ordered, a single traversal pass down should be enough