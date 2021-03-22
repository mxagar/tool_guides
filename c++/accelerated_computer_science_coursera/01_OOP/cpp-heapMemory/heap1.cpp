/**
 * C++ program allocating and free'ing heap memory.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include "Cube.h"
using uiuc::Cube;

int main() {
  // p is in the stack and contains an address
  // to an int in the heap
  int *p = new int;
  // c is in the stack and contains an address
  // to a Cube in the heap
  Cube *c = new Cube;

  // De-reference (go to memory) and fill in values
  *p = 42;
  (*c).setLength(4);
  // It is better to use the arrow operator,
  // which is equivalent to the previous (*c).
  // -> works when we wnat to access the object memory of pointers
  // that contain addresses of classes
  c->setLength(4); 

  // delete destroys the heap memory allocated for the actual variables
  // but the pointer in the stack still remains
  // and it is pointing to a non-allocated address!
  // Since C++11 we have nullptr, which refers to memory address 0x0
  // nullptr is equivalent to the C NULL
  // Address 0x0 is reserved and never used by the system
  // so we understand if var == nullptr, var contains a non-initialized address
  delete c;  c = nullptr;
  delete p;  p = nullptr;
  return 0;
}
