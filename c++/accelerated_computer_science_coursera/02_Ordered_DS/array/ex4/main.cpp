/**
 * Calculating the memory seperation of elements in a std::vector.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include <iostream>
#include <vector>
#include "../Cube.h"

using uiuc::Cube;

int main() {
  std::vector<Cube> cubes{ Cube(11), Cube(42), Cube(400) };

  // Examine capacity, push_back = append, size
  std::cout << "Initial Capacity: " << cubes.capacity() << std::endl; // 3
  cubes.push_back( Cube(800) );
  std::cout << "Size after adding: " << cubes.size() << std::endl; // 3
  std::cout << "Capacity after adding: " << cubes.capacity() << std::endl; // 3x2 = 6, it is doubled!

  // Compute the offset from the beginning of the array to [2] and [3]
  std::cout << "Size of cube element: " << sizeof(cubes[0]) << std::endl; // 8 bytes
  int offset1 = (long)&(cubes[2]) - (long)&(cubes[0]); // 2 x 8
  int offset2 = (long)&(cubes[3]) - (long)&(cubes[0]); // 3 x 8 -> they are sequantially stored!
  std::cout << "Memory separation 1: " << offset1 << std::endl;
  std::cout << "Memory separation 2: " << offset2 << std::endl;

  // Find a specific `target` cube in the array:
  Cube target = Cube(400);
  for (unsigned i = 0; i < cubes.size(); i++) {
    if (target == cubes[i]) {
      std::cout << "Found target at [" << i << "]" << std::endl;
    }
  }

  return 0;
}
