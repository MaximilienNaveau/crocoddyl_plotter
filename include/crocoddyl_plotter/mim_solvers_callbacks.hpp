#ifndef CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_
#define CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_

#include <crocoddyl/core/solver-base.hpp>

namespace crocoddyl_plotter {

class CrocoddylPlotterImpl;

class CrocoddylPlotterServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CrocoddylPlotterServer(std::string url = "0.0.0.0:1234");
  ~CrocoddylPlotterServer();

  bool send(const std::shared_ptr<crocoddyl::ShootingProblem>& problem, int iteration);

 private:
  std::unique_ptr<CrocoddylPlotterImpl> impl_;
};

}  // namespace crocoddyl_plotter

#endif  // CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_
