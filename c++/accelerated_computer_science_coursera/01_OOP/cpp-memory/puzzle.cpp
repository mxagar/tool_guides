/**
 * C++ code showing improper use of memory (returning a reference to a stack variable).
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

double someOtherFunction();  // Forward decl

#include <iostream>
#include "Cube.h"
using uiuc::Cube;

Cube *CreateUnitCube() {
  // We allocate memory in the stack of CreateUnitCube
  // for Cube cube
  Cube cube;
  cube.setLength(15);
  // We return the address of cube
  // BUT: after exiting CreateUnitCube()
  // the stack memory that contains cube is destroyed!
  return &cube;
}

int main() {
  // main() is added to the stack
  // CreateCube() is added to the stack
  // and executed -> see the comments in it
  // the returned address is wrong
  // because after exiting CreateUnitCube()
  // we destroy it!
  Cube *c = CreateUnitCube();
  // Now, the stack allocated and destroyed for CreateUnitCube()
  // is re-allocated for someOtherFunction()!
  someOtherFunction();
  double a = c->getSurfaceArea();
  std::cout << "Surface Area: " << a << std::endl;
  double v = c->getVolume();
  std::cout << "Volume: " << v << std::endl;
  return 0;
}


// Some other function that does something that uses some stack memory.
// In this code, we calculate the total volume of cubes of length from 0..99.
double someOtherFunction() {
  Cube cubes[100];
  double totalVolume = 0;

  for (int i = 0; i < 100; i++) {
    cubes[i].setLength(i);
    totalVolume += cubes[i].getVolume();
  }

  // We return totalVolume by value, not by reference
  // therefore, a new stack location is assigned outside from
  // the stack of someOtherFunction()
  // which is destroyed after exiting the function
  return totalVolume;
}