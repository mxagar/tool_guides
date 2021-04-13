/**
 * Generic `Shape` class.
 * 
 * @author
 *   Wade Fagen-Ulmschneider <waf@illinois.edu>
 */

#pragma once

class Shape {
  public:
    Shape();
    Shape(double width);
    // All derived classes will have this method
    double getWidth() const;

  private:
    // All derived classes will have this variable
    double width_;
};
