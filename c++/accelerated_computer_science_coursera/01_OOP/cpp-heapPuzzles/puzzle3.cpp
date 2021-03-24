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
  // Since we use new, some variables live in the heap!
  // Pointers in the stack
  int *p, *q;
  // p points to a memory part in the heap
  p = new int;
  q = p; // q contains the same address as p
  *q = 8; // fill in heap memory
  cout << *p << endl; // 8

  q = new int; // we create a new memory part in the heap
  *q = 9; // fill in latter heap memory
  cout << *p << endl; // 8
  cout << *q << endl; // 9

  return 0;
}
