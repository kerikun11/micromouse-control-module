/**
 * @file ctrl.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this files defines a python module implemented in C++
 * @date 2020-06-11
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#include <ctrl/accel_designer.h>
#include <ctrl/slalom.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

PYBIND11_MODULE(ctrl, m) {
  namespace py = pybind11;
  using namespace ctrl;

  m.doc() = "MicroMouse Control Module";

  py::class_<AccelCurve>(m, "AccelCurve")
      .def(py::init<>())
      .def(py::init<float, float, float, float>())
      .def("reset", &AccelCurve::reset)
      .def("j", &AccelCurve::j)
      .def("a", &AccelCurve::a)
      .def("v", &AccelCurve::v)
      .def("x", &AccelCurve::x)
      .def("t_end", &AccelCurve::t_end)
      .def("v_end", &AccelCurve::v_end)
      .def("x_end", &AccelCurve::x_end)
      .def("t_0", &AccelCurve::t_0)
      .def("t_1", &AccelCurve::t_1)
      .def("t_2", &AccelCurve::t_2)
      .def("t_3", &AccelCurve::t_3)
      .def("getTimeStamp", &AccelCurve::getTimeStamp)
      .def("__str__",
           [](const AccelCurve &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  py::class_<AccelDesigner>(m, "AccelDesigner")
      .def(py::init<>())
      .def(py::init<float, float, float, float, float, float, float, float>(),
           py::arg("j_max"), py::arg("a_max"), py::arg("v_max"),
           py::arg("v_start"), py::arg("v_target"), py::arg("dist"),
           py::arg("x_start") = float(0), py::arg("t_start") = float(0))
      .def("reset", &AccelDesigner::reset, //
           py::arg("j_max"), py::arg("a_max"), py::arg("v_max"),
           py::arg("v_start"), py::arg("v_target"), py::arg("dist"),
           py::arg("x_start") = float(0), py::arg("t_start") = float(0))
      .def("j", &AccelDesigner::j)
      .def("a", &AccelDesigner::a)
      .def("v", &AccelDesigner::v)
      .def("x", &AccelDesigner::x)
      .def("t_end", &AccelDesigner::t_end)
      .def("v_end", &AccelDesigner::v_end)
      .def("x_end", &AccelDesigner::x_end)
      .def("t_0", &AccelDesigner::t_0)
      .def("t_1", &AccelDesigner::t_1)
      .def("t_2", &AccelDesigner::t_2)
      .def("t_3", &AccelDesigner::t_3)
      .def("getTimeStamp", &AccelDesigner::getTimeStamp)
      .def("__str__",
           [](const AccelDesigner &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  py::class_<Pose>(m, "Pose")
      .def(py::init<>())
      .def(py::init<float, float, float>())
      .def_readwrite("x", &Pose::x)
      .def_readwrite("y", &Pose::y)
      .def_readwrite("th", &Pose::th)
      .def("clear", &Pose::clear)
      .def("mirror_x", &Pose::mirror_x)
      .def("rotate", &Pose::rotate)
      .def("homogeneous", &Pose::homogeneous)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def("__str__",
           [](const Pose &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  py::class_<slalom::Shape>(m, "Shape")
      .def(py::init<Pose, float, float, float, float, float>(),
           py::arg("total"), py::arg("y_curve_end"), py::arg("x_adv") = 0,
           py::arg("dddth_max") = slalom::dddth_max_default,
           py::arg("ddth_max") = slalom::ddth_max_default,
           py::arg("dth_max") = slalom::dth_max_default)
      .def(py::init<Pose, Pose, float, float, float, float, float, float>())
      .def_readwrite("total", &slalom::Shape::total)
      .def_readwrite("curve", &slalom::Shape::curve)
      .def_readwrite("straight_prev", &slalom::Shape::straight_prev)
      .def_readwrite("straight_post", &slalom::Shape::straight_post)
      .def_readwrite("v_ref", &slalom::Shape::v_ref)
      .def_readwrite("dddth_max", &slalom::Shape::dddth_max)
      .def_readwrite("ddth_max", &slalom::Shape::ddth_max)
      .def_readwrite("dth_max", &slalom::Shape::dth_max)
      .def_static("integrate", &slalom::Shape::integrate)
      .def("__str__",
           [](const slalom::Shape &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  py::class_<State>(m, "State")
      .def(py::init<>())
      .def_readwrite("q", &State::q)
      .def_readwrite("dq", &State::dq)
      .def_readwrite("ddq", &State::ddq)
      .def_readwrite("dddq", &State::dddq)
      //
      ;

  py::class_<slalom::Trajectory>(m, "Trajectory")
      .def(py::init<slalom::Shape &, bool>(), py::arg("shape"),
           py::arg("mirror_x") = false)
      .def("reset", &slalom::Trajectory::reset)
      .def("update", &slalom::Trajectory::update, py::arg("state"),
           py::arg("t"), py::arg("Ts"), py::arg("k_slip") = 0e0f)
      .def("getVelocity", &slalom::Trajectory::getVelocity)
      .def("getTimeCurve", &slalom::Trajectory::getTimeCurve)
      .def("getShape", &slalom::Trajectory::getShape)
      .def("getAccelDesigner", &slalom::Trajectory::getAccelDesigner)
      //
      ;
}
