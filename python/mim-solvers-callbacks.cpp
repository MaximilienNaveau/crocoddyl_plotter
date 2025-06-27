///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl_plotter/mim_solvers_callbacks.hpp"
#include "python.hpp"

namespace bp = boost::python;
namespace crocoddyl_plotter {

void exposeCallbacks() {
//   bp::register_ptr_to_python<std::shared_ptr<CallbackAbstract> >();

  bp::class_<CrocoddylPlotterServer, boost::noncopyable>(
      "CrocoddylPlotterServer", "Callback function for sending data over network.",
      bp::init<std::string>("Initialize the callback.", bp::arg("url")="0.0.0.0:1234"))
      .def("send", &CrocoddylPlotterServer::send)
      ;
}

}  // namespace mim_solvers
