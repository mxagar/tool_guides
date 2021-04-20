/**
 * Calculating the memory seperation of elements in an array.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include <iostream>

int main() {
  // Create an array of 10 number
  int values[10] = { 2, 3, 5, 7, 11, 13, 15, 17, 21, 23 };
  
  // Print the size of each type `int`
  std::cout << sizeof(int) << std::endl; // 4 bytes

  // Compute the offset from the beginning of the array to [2]
  // &[0] points to the start or index 0
  // &[2] points to the start or index 2
  // between indices 0 and 2 there are 2 cells -> 2 x 4 bytes
  int offset = (long)&(values[2]) - (long)&(values[0]);
  std::cout << offset << std::endl;

  return 0;
}
