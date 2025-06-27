#include "python.hpp"

BOOST_PYTHON_MODULE(crocoddyl_plotter_pywrap) {
  namespace bp = boost::python;

  bp::import("mim_solvers");
  bp::import("crocoddyl");

  crocoddyl_plotter::exposeCallbacks();
}