#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

// Dummy classes to be wrapped in python

class Shape
{
public:
	virtual double area() const { return 0.0; }
};

class Circle : public Shape
{
	double _r;
public:
	Circle(double r) : _r(r) {}
	double radius() const { return _r; }
	void setRadius(double r) { _r = r; }
	double area() const { return 3.1415927 * _r * _r; }
};


PYBIND11_MODULE(dummylib, m)
{
	/**
	Print an object
	*/
	m.def("print", [](const py::object& o) {
		std::string s = py::str(o).cast<std::string>();
		printf("   --> DummyLib says: '%s'\n", s.c_str());
	});

	/**
	Multiply elements in a vector of doubles by a factor
	@param factor the factor to multiply
	@param all if this is true (default) all elements will be multiplied, ese, only even items
	*/
	m.def("mult", [](const std::vector<double>& a, double factor, bool all) {

		std::vector<double> b = a;
		int step = all ? 1 : 2;
		for (size_t i = 0; i < a.size(); i += step)
		{
			b[i] = factor * a[i];
		}
		return b;
	},
	py::arg("array"), py::arg("factor") = 2.0, py::arg("all") = true);


	/**
	* Class Shape
	*/
	py::class_<Shape, std::shared_ptr<Shape>>(m, "Shape")

		.def(py::init<>())

		/**
		* Returns the area of the shape
		*/
		.def("area", [](std::shared_ptr<Shape> self) { return self->area(); })
		;

	/**
	* Class Circle, derived from Shape
	*/
	py::class_<Circle, std::shared_ptr<Circle>, Shape>(m, "Circle")

		/**
		* Constructor
		*/
		.def(py::init<double>())

		/**
		* Property get/set radius
		*/
		.def_property("radius",
			[](std::shared_ptr<Circle> self) { return self->radius(); },
			[](std::shared_ptr<Circle> self, double r) { self->setRadius(r); })
		;

}

