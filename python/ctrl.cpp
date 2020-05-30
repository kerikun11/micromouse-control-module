#include <accel_designer.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

BOOST_PYTHON_MODULE(ctrl) {
  using namespace boost::python;
  using namespace ctrl;

  class_<AccelCurve>("AccelCurve")
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
      //
      ;

  class_<AccelDesigner>("AccelDesigner")
      .def("reset", &AccelDesigner::reset)
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
      //
      ;

  class_<std::vector<float>>("vf").def(
      vector_indexing_suite<std::vector<float>>());
}
