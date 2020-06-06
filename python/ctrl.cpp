#include <ctrl/accel_designer.h>
#include <ctrl/slalom.h>

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
      .def(self_ns::str(self))
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
      .def(self_ns::str(self))
      //
      ;
  class_<std::vector<float>>("vf").def(
      vector_indexing_suite<std::vector<float>>());

  class_<Pose>("Pose", init<float, float, float>())
      .def_readwrite("x", &Pose::x)
      .def_readwrite("y", &Pose::y)
      .def_readwrite("th", &Pose::th)
      .def("clear", &Pose::clear)
      .def("homogeneous", &Pose::homogeneous)
      .def("rotate", &Pose::rotate)
      .def("mirror_x", &Pose::mirror_x)
      .def(self_ns::str(self))
      //
      ;

  class_<slalom::Shape>(
      "Shape",
      //   init<Pose, Pose, float, float, float, float, float, float>())
      init<Pose, float, float, float, float, float>())
      .def_readwrite("total", &slalom::Shape::total)
      .def_readwrite("curve", &slalom::Shape::curve)
      .def_readwrite("straight_prev", &slalom::Shape::straight_prev)
      .def_readwrite("straight_post", &slalom::Shape::straight_post)
      .def_readwrite("v_ref", &slalom::Shape::v_ref)
      .def_readwrite("dddth_max", &slalom::Shape::dddth_max)
      .def_readwrite("ddth_max", &slalom::Shape::ddth_max)
      .def_readwrite("dth_max", &slalom::Shape::dth_max)
      .def("integrate", &slalom::Shape::integrate)
      .def(self_ns::str(self))
      //
      ;

  class_<State>("State")
      .def_readwrite("q", &State::q)
      .def_readwrite("dq", &State::dq)
      .def_readwrite("ddq", &State::ddq)
      .def_readwrite("dddq", &State::dddq)
      //
      ;

  class_<slalom::Trajectory>("Trajectory", init<slalom::Shape &, bool>())
      .def("reset", &slalom::Trajectory::reset)
      .def(
          "update",
          +[](const slalom::Trajectory &obj, const State &s, float t, float Ts,
              float k_slip) {
            auto state = s;
            obj.update(state, t, Ts, k_slip);
            return state;
          })
      .def("getVelocity", &slalom::Trajectory::getVelocity)
      .def("getTimeCurve", &slalom::Trajectory::getTimeCurve)
      .def("getShape", &slalom::Trajectory::getShape,
           return_value_policy<copy_const_reference>())
      .def("getAccelDesigner", &slalom::Trajectory::getAccelDesigner,
           return_value_policy<copy_const_reference>())
      //
      ;
}
