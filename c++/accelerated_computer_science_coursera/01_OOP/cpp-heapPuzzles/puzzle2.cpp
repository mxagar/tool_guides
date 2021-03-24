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
  // Since we have the new keyword
  // the value pointed by new is in the heap!
  
  // STACK: x = 0x1...: it contains an address to heap memory (small)
  // HEAP: *x = 0: it contains an int value, still uninitialized
  int *x = new int;
  
  // & is used for referencing variables: aliases/links to memory parts
  // y is equivalent to the content pointed by x
  // &y is the the address contained by x!
  int &y = *x; // y is linking to the content pointed by x
  y = 4; // content pointed by x is filled

  cout << &x << endl; // 0xF...: address in the stack (large)
  cout << x << endl; // 0x1...: address in the heap (small)
  cout << *x << endl; // 4: value in the heap assigned through alias y

  cout << &y << endl; // 0x1... the address contained in x
  cout << y << endl; // 4: value pointed by x in the heap
  // cout << *y << endl; // it does not make sense: an int is not an address pointing somewhere...

  return 0;
}
