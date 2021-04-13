/**
 * A `Cube` class inheriting from a `Shape`
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#pragma once

#include "Shape.h"
#include "HSLAPixel.h"

namespace uiuc {
  class Cube : public Shape {
    // Since we inherit from Shape,
    // we get all public methods and variables from it
    // Public methods from Shape should give access
    // to private variables, eg: getWidth() -> width_
    public:
      Cube(double width, uiuc::HSLAPixel color);
      double getVolume() const;

    private:
      uiuc::HSLAPixel color_;
  };
}