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
- List contains nodes; nodes are created in the heap, each node contains `next` and `prev` pointers and also the data stored.
- Class is similar to `std::list`, but not as through: missing `iterator`, memory layer not as abstracted, etc.

## New concepts: `const` and Sorting

See `../README.md`.

- `const` correctness
- Insertion and Merge Sort

## Notes

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

            splitHalves()
            front()
            back()
            explode()
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
            print()
            operators =, ...
## Exercise 1


## Exercise 2

