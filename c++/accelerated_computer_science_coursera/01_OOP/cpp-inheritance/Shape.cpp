#include "Shape.h"

// From the default constructor
// We call the custom constructor below by using Shape(1)
Shape::Shape() : Shape(1) {
  // Nothing.
}

// We have an initialized list
// with which we initialize the member varieble width_
// Initializer list: MyClass::MyClass(double var) : var_(var)
// With the initializer list we avoid writing redundant lines of code
// and initialize everything in one place in the code!
Shape::Shape(double width) : width_(width) {
  // Nothing.
}

double Shape::getWidth() const {
  return width_;
}
