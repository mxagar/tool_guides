/**
 * C++ puzzle program.  Try to figure out the result before running!
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include <iostream>

using std::cout;
using std::endl;

int main() {
  // Pointer in the stack
  int *x;
  int size = 3;
  // Content in the heap
  // Arrays
  x = new int[size]; // we allocate a sequence of ints of size size = 3

  for (int i = 0; i < size; i++) {
    x[i] = i + 3;
  }

  delete[] x; // since x points to a sequence, all sequence must be de-allocated
}
