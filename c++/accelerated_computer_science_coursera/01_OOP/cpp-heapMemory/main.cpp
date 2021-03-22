/**
 * C++ program printing various memory values using heap-allocated memory.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include <iostream>

int main() {
  // numPtr is in the stack memory and contains an address
  // to a heap memory part which contains an it
  // the int value is not initialized yet
  int *numPtr = new int;

  std::cout << "*numPtr: " << *numPtr << std::endl; // the int in the heap is not initialized; any value can be displayed here
  std::cout << " numPtr: " <<  numPtr << std::endl; // it contains an address to the heap, low address value
  std::cout << "&numPtr: " << &numPtr << std::endl; // the address of numPtr in the stack, large address value

  // we change the value of the int from the heap
  *numPtr = 42;
  std::cout << "*numPtr assigned 42." << std::endl;

  std::cout << "*numPtr: " << *numPtr << std::endl;
  std::cout << " numPtr: " <<  numPtr << std::endl;
  std::cout << "&numPtr: " << &numPtr << std::endl;

  return 0;
}