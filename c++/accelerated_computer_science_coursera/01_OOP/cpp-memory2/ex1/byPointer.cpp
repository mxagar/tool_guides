/**
 * C++ program copying a Cube currency pointer.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include "../Cube.h"
using uiuc::Cube;

int main() {
  // Create a 1,000-valued cube
  Cube c(10);

  // Transfer the cube
  Cube * myCube = &c;

  // Effect: we have 2 variables: the pointer and the cube itself
  

  return 0;
}
