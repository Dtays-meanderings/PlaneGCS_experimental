#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "GCS.h"
#include "Geo.h"

namespace py = pybind11;

class Parameter {
private:
    double value;
public:
    Parameter(double val) : value(val) {}
    double getValue() const { return value; }
    void setValue(double val) { value = val; }
    double* get_ptr() { return &value; }
};

PYBIND11_MODULE(planegcs, m) {
    m.doc() = "PlaneGCS 2D Geometric Constraint Solver Python Bindings";

    // Bind Parameter class
    py::class_<Parameter>(m, "Parameter")
        .def(py::init<double>(), py::arg("value") = 0.0)
        .def_property("value", &Parameter::getValue, &Parameter::setValue)
        .def("__repr__", [](const Parameter &p) {
            return "<planegcs.Parameter value=" + std::to_string(p.getValue()) + ">";
        });

    // Bind Point class
    py::class_<GCS::Point>(m, "Point")
        .def(py::init([](Parameter &x, Parameter &y) {
            return GCS::Point(x.get_ptr(), y.get_ptr());
        }), py::keep_alive<1, 2>(), py::keep_alive<1, 3>())
        .def_property("x", 
            [](const GCS::Point &self) { return *self.x; },
            [](GCS::Point &self, double val) { *self.x = val; })
        .def_property("y", 
            [](const GCS::Point &self) { return *self.y; },
            [](GCS::Point &self, double val) { *self.y = val; });

    // Bind Curve class
    py::class_<GCS::Curve>(m, "Curve");

    // Bind Line class
    py::class_<GCS::Line, GCS::Curve>(m, "Line")
        .def(py::init([](const GCS::Point &p1, const GCS::Point &p2) {
            auto l = new GCS::Line();
            l->p1 = p1;
            l->p2 = p2;
            return l;
        }))
        .def_readwrite("p1", &GCS::Line::p1)
        .def_readwrite("p2", &GCS::Line::p2);

    // Bind Circle class
    py::class_<GCS::Circle, GCS::Curve>(m, "Circle")
        .def(py::init([](const GCS::Point &center, Parameter &rad) {
            auto c = new GCS::Circle();
            c->center = center;
            c->rad = rad.get_ptr();
            return c;
        }), py::keep_alive<1, 3>())
        .def_readwrite("center", &GCS::Circle::center)
        .def_property("rad",
            [](const GCS::Circle &self) { return *self.rad; },
            [](GCS::Circle &self, double val) { *self.rad = val; });

    // Bind SolveStatus enum
    py::enum_<GCS::SolveStatus>(m, "SolveStatus")
        .value("Success", GCS::SolveStatus::Success)
        .value("Converged", GCS::SolveStatus::Converged)
        .value("Failed", GCS::SolveStatus::Failed)
        .value("SuccessfulSolutionInvalid", GCS::SolveStatus::SuccessfulSolutionInvalid)
        .export_values();

    // Bind Algorithm enum
    py::enum_<GCS::Algorithm>(m, "Algorithm")
        .value("BFGS", GCS::Algorithm::BFGS)
        .value("LevenbergMarquardt", GCS::Algorithm::LevenbergMarquardt)
        .value("DogLeg", GCS::Algorithm::DogLeg)
        .export_values();

    // Bind System class
    py::class_<GCS::System>(m, "System")
        .def(py::init<>())
        .def("clear", &GCS::System::clear)
        .def("clearByTag", &GCS::System::clearByTag, py::arg("tagId"))
        
        // Basic Constraints
        .def("addConstraintEqual", [](GCS::System &self, Parameter &p1, Parameter &p2, int tagId, bool driving) {
            return self.addConstraintEqual(p1.get_ptr(), p2.get_ptr(), tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("tagId") = 0, py::arg("driving") = true)
        
        .def("addConstraintDifference", [](GCS::System &self, Parameter &p1, Parameter &p2, Parameter &diff, int tagId, bool driving) {
            return self.addConstraintDifference(p1.get_ptr(), p2.get_ptr(), diff.get_ptr(), tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("difference"), py::arg("tagId") = 0, py::arg("driving") = true)
        
        .def("addConstraintP2PDistance", [](GCS::System &self, GCS::Point &p1, GCS::Point &p2, Parameter &dist, int tagId, bool driving) {
            return self.addConstraintP2PDistance(p1, p2, dist.get_ptr(), tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("distance"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintP2PAngle", [](GCS::System &self, GCS::Point &p1, GCS::Point &p2, Parameter &angle, double incrAngle, int tagId, bool driving) {
            return self.addConstraintP2PAngle(p1, p2, angle.get_ptr(), incrAngle, tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("angle"), py::arg("incrAngle") = 0.0, py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintHorizontal", [](GCS::System &self, GCS::Point &p1, GCS::Point &p2, int tagId, bool driving) {
            return self.addConstraintHorizontal(p1, p2, tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintVertical", [](GCS::System &self, GCS::Point &p1, GCS::Point &p2, int tagId, bool driving) {
            return self.addConstraintVertical(p1, p2, tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintP2LDistance", [](GCS::System &self, GCS::Point &p, GCS::Line &l, Parameter &dist, int tagId, bool driving) {
            return self.addConstraintP2LDistance(p, l, dist.get_ptr(), tagId, driving);
        }, py::arg("p"), py::arg("l"), py::arg("distance"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintPointOnLine", [](GCS::System &self, GCS::Point &p, GCS::Line &l, int tagId, bool driving) {
            return self.addConstraintPointOnLine(p, l, tagId, driving);
        }, py::arg("p"), py::arg("l"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintPointOnCircle", [](GCS::System &self, GCS::Point &p, GCS::Circle &c, int tagId, bool driving) {
            return self.addConstraintPointOnCircle(p, c, tagId, driving);
        }, py::arg("p"), py::arg("c"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintParallel", [](GCS::System &self, GCS::Line &l1, GCS::Line &l2, int tagId, bool driving) {
            return self.addConstraintParallel(l1, l2, tagId, driving);
        }, py::arg("l1"), py::arg("l2"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintPerpendicular", [](GCS::System &self, GCS::Line &l1, GCS::Line &l2, int tagId, bool driving) {
            return self.addConstraintPerpendicular(l1, l2, tagId, driving);
        }, py::arg("l1"), py::arg("l2"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintTangent", [](GCS::System &self, GCS::Line &l, GCS::Circle &c, int tagId, bool driving) {
            return self.addConstraintTangent(l, c, tagId, driving);
        }, py::arg("l"), py::arg("c"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintL2LAngle", [](GCS::System &self, GCS::Line &l1, GCS::Line &l2, Parameter &angle, int tagId, bool driving) {
            return self.addConstraintL2LAngle(l1, l2, angle.get_ptr(), tagId, driving);
        }, py::arg("l1"), py::arg("l2"), py::arg("angle"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintEqualRadius", [](GCS::System &self, GCS::Circle &c1, GCS::Circle &c2, int tagId, bool driving) {
            return self.addConstraintEqualRadius(c1, c2, tagId, driving);
        }, py::arg("c1"), py::arg("c2"), py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintEqualLength", [](GCS::System &self, GCS::Line &l1, GCS::Line &l2, Parameter *length, int tagId, bool driving) {
            return self.addConstraintEqualLength(l1, l2, length ? length->get_ptr() : nullptr, tagId, driving);
        }, py::arg("l1"), py::arg("l2"), py::arg("length") = nullptr, py::arg("tagId") = 0, py::arg("driving") = true)

        .def("addConstraintP2PSymmetric", [](GCS::System &self, GCS::Point &p1, GCS::Point &p2, GCS::Line &l, int tagId, bool driving) {
            return self.addConstraintP2PSymmetric(p1, p2, l, tagId, driving);
        }, py::arg("p1"), py::arg("p2"), py::arg("l"), py::arg("tagId") = 0, py::arg("driving") = true)

        // Solving management
        .def("declareUnknowns", [](GCS::System &self, std::vector<Parameter*> &params) {
            GCS::VEC_pD raw_params;
            raw_params.reserve(params.size());
            for (auto p : params) {
                raw_params.push_back(p->get_ptr());
            }
            self.declareUnknowns(raw_params);
        })
        .def("initSolution", &GCS::System::initSolution, py::arg("alg") = GCS::Algorithm::DogLeg)
        .def("solve", [](GCS::System &self, bool isFine, GCS::Algorithm alg, bool isRedundantsolving) {
            return self.solve(isFine, alg, isRedundantsolving);
        }, py::arg("isFine") = true, py::arg("alg") = GCS::Algorithm::DogLeg, py::arg("isRedundantsolving") = false)
        .def("applySolution", &GCS::System::applySolution);
}
