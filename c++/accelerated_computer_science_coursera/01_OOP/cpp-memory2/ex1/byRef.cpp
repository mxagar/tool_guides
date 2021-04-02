/**
 * C++ program aliasing a Cube class by reference.
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
  Cube & myCube = c;

  // Effect: the cube is constructud once!
  // We have only 1000 USD altogether!
  // We have two variables owning the same cube!

  return 0;
}
