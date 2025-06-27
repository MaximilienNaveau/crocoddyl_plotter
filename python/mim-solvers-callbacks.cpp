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

  bp::class_<MimSolversPlotter, bp::bases<mim_solvers::CallbackAbstract>, boost::noncopyable>(
      "MimSolversPlotter", "Callback function for sending data over network.",
      bp::init<>("Initialize the callback."))
      .def("send", &MimSolversPlotter::send)
      ;
}

}  // namespace mim_solvers
