/**
 * Simple C++ class for representing a Cube (with constructors).
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#include "Cube.h"
#include <iostream>

namespace uiuc {  
  Cube::Cube(double length) {
    // Cube c1(100);
    length_ = length;
    std::cout << "Created $" << getVolume() << std::endl;
  }

  Cube::Cube(const Cube & obj) {
    // obj passed as reference: no new Cube created when passing it
    // Cube c2 = c1;
    length_ = obj.length_;
    std::cout << "Created $" << getVolume() << " via copy" << std::endl;
  }

  Cube & Cube::operator=(const Cube & obj) {
    // We assign/copy to the Cube the values from obj
    // Cube c2;
    // c2 = c1;
    std::cout << "Transformed $" << getVolume() << "-> $" << obj.getVolume() << std::endl;
    length_ = obj.length_;
    return *this;
  }

  double Cube::getVolume() const {
    return length_ * length_ * length_;
  }

  double Cube::getSurfaceArea() const {
    return 6 * length_ * length_;
  }

  void Cube::setLength(double length) {
    length_ = length;
  }
}
