/**
 * C++ program using the Cube's one arugment constructor.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include "Cube.h"
#include <iostream>

int main() {
  uiuc::Cube c;  // !!! ERROR: we don't have a param-less or default constructor!
  std::cout << "Volume: " << c.getVolume() << std::endl;
  return 0;
}