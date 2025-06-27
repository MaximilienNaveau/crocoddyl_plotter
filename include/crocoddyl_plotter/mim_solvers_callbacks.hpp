#ifndef CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_
#define CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_

#include <mim_solvers/utils/callbacks.hpp>

namespace crocoddyl_plotter {

class MimSolversPlotterImpl;

class MimSolversPlotter : public mim_solvers::CallbackAbstract {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MimSolversPlotter(std::string url = "0.0.0.0:1234");
  ~MimSolversPlotter() override;

  bool send(const std::shared_ptr<crocoddyl::ShootingProblem>& problem, int iteration);

  void operator()(crocoddyl::SolverAbstract& solver) override;
  void operator()(crocoddyl::SolverAbstract& solver,
                  std::string solver_type) override;

 private:
  std::unique_ptr<MimSolversPlotterImpl> impl_;
};

}  // namespace crocoddyl_plotter

#endif  // CROCODDYL_PLOTTER_MIM_SOLVERS_CALLBACKS_HPP_
