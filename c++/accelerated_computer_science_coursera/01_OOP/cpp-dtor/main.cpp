/**
 * C++ program invoking Cube's destructor several times.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include "Cube.h"
using uiuc::Cube;

double cube_on_stack() {
  Cube c(3); // Construtor called, object memory allocated on function stack
  return c.getVolume(); // We leave the function stack, destructor called
}

void cube_on_heap() {
  Cube * c1 = new Cube(10); // Constructor called, object memory allocated in heap
  Cube * c2 = new Cube; // Constructor called, object memory allocated in heap
  delete c1; // We delete one object from te heap
  // BUT: one object instance remeins in the heap when we leave the function!
}

int main() {
  cube_on_stack(); // create 1, destroy 1
  cube_on_heap(); // create 2, destroy 1
  cube_on_stack(); // create 1, destory 1
  return 0;
}