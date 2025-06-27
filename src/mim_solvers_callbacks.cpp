#include "crocoddyl_plotter/mim_solvers_callbacks.hpp"

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "crocoddyl.grpc.pb.h"
#include <grpcpp/grpcpp.h>

#include <crocoddyl/core/integ-action-base.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>

namespace crocoddyl_plotter {
typedef double Scalar;
typedef crocoddyl::ActionModelAbstractTpl<Scalar> ActionModelAbstract;
typedef crocoddyl::ActionDataAbstractTpl<Scalar> ActionDataAbstract;
typedef crocoddyl::IntegratedActionModelAbstractTpl<Scalar>
    IntegratedActionModelAbstract;
typedef crocoddyl::IntegratedActionDataAbstractTpl<Scalar>
    IntegratedActionDataAbstract;

typedef crocoddyl::IntegratedActionModelEulerTpl<Scalar>
    IntegratedActionModelEuler;
typedef crocoddyl::IntegratedActionDataEulerTpl<Scalar>
    IntegratedActionDataEuler;

typedef crocoddyl::DifferentialActionModelAbstractTpl<Scalar>
    DifferentialActionModelAbstract;
typedef crocoddyl::DifferentialActionDataAbstractTpl<Scalar>
    DifferentialActionDataAbstract;

typedef crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<Scalar>
    DifferentialActionModelFreeFwdDynamics;
typedef crocoddyl::DifferentialActionDataFreeFwdDynamicsTpl<Scalar>
    DifferentialActionDataFreeFwdDynamics;

typedef crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
typedef crocoddyl::CostDataSumTpl<Scalar> CostDataSum;

typedef ActionDataAbstract::VectorXs VectorXs;
typedef ActionDataAbstract::MatrixXs MatrixXs;
typedef Eigen::Map<VectorXs> VectorMapXs;
typedef Eigen::Map<MatrixXs> MatrixMapXs;

class MimSolversPlotterImpl final : public CrocoddylPlotter::Service {
public:
  // CrocoddylPlotter service_;
  std::unique_ptr<grpc::Server> server_;
  std::thread serverThread_;
  grpc::ServerWriter<OCPd> *writer_ = NULL;

  int iteration_ = 0;
  OCPd nodes_;
  struct OCPConverter {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum {
      Unknown,
      FreeFwdDynamics,
    } differentialModelType = Unknown;

    std::vector<Scalar> cost;
    std::vector<VectorXs> Lx;
    std::vector<VectorXs> Lu;

    void init(const std::shared_ptr<ActionModelAbstract> &am,
              const std::shared_ptr<ActionDataAbstract> &ad) {
      auto iam = std::dynamic_pointer_cast<IntegratedActionModelEuler>(am);
      auto iad = std::dynamic_pointer_cast<IntegratedActionDataEuler>(ad);
      if (!iam || !iad)
        throw std::runtime_error("Not an integrated action.");

      auto dam =
          std::dynamic_pointer_cast<DifferentialActionModelFreeFwdDynamics>(
              iam->get_differential());
      auto dad =
          std::dynamic_pointer_cast<DifferentialActionDataFreeFwdDynamics>(
              iad->differential);
      if (dam) {
        if (!dad)
          throw std::runtime_error("Model and data not matching.");
        differentialModelType = FreeFwdDynamics;
      }
      // Add support to other differential actions here.

      if (differentialModelType == Unknown) {
        throw std::runtime_error("Unsupported differential action model");
      }

      auto &cm = get_costs(am);
      auto &cd = get_costs(ad);

      const auto &cmc = cm->get_costs();
      const auto &cdc = cd->costs;

      cost.resize(cmc.size());
      Lx.resize(cmc.size());
      Lu.resize(cmc.size());
      int i = 0;
      for (auto const &it : cdc) {
        Lx[i].resizeLike(it.second->Lx);
        Lu[i].resizeLike(it.second->Lu);
        ++i;
      }
    }

    const std::shared_ptr<CostModelSum> &
    get_costs(const std::shared_ptr<ActionModelAbstract> &am) {
      auto dam = std::static_pointer_cast<IntegratedActionModelAbstract>(am)
                     ->get_differential();
      switch (differentialModelType) {
      case FreeFwdDynamics:
        return std::static_pointer_cast<DifferentialActionModelFreeFwdDynamics>(
                   dam)
            ->get_costs();
      default:
        abort();
      }
    }

    const std::shared_ptr<CostDataSum> &
    get_costs(const std::shared_ptr<ActionDataAbstract> &ad) {
      auto dad =
          std::static_pointer_cast<IntegratedActionDataEuler>(ad)->differential;
      switch (differentialModelType) {
      case FreeFwdDynamics:
        return std::static_pointer_cast<DifferentialActionDataFreeFwdDynamics>(
                   dad)
            ->costs;
      default:
        abort();
      }
    }

    void read(const std::shared_ptr<ActionModelAbstract> &am,
              const std::shared_ptr<ActionDataAbstract> &ad) {
      auto &cd = get_costs(ad);

      const auto &cdc = cd->costs;

      int i = 0;
      for (auto const &it : cdc) {
        cost[i] = it.second->cost;
        Lx[i].swap(it.second->Lx);
        Lu[i].swap(it.second->Lu);
        ++i;
      }
    }

    void write(Noded &node) {
      auto *cs = node.mutable_costs();

      cs->Reserve(cost.size());
      if (cs->size() != cost.size()) {
        // TODO handle non 0 case in assert below.
        assert(cs->size() == 0);
        for (int i = 0; i < cost.size(); ++i)
          cs->Add();
      }

      for (int i = 0; i < cost.size(); ++i) {
        auto &c = cs->at(i);
        c.mutable_lx()->Resize(Lx[i].size(), Scalar(0));
        c.mutable_lu()->Resize(Lu[i].size(), Scalar(0));
        c.set_cost(cost[i]);
        VectorMapXs(c.mutable_lx()->mutable_data(), Lx[i].size()) = Lx[i];
        VectorMapXs(c.mutable_lu()->mutable_data(), Lu[i].size()) = Lu[i];
        // c.mutable_lx()->Assign(Lx[i].data(), Lx[i].data() + Lx[i].size());
        // c.mutable_lu()->Assign(Lu[i].data(), Lu[i].data() + Lu[i].size());
      }
    }
  };
  std::vector<OCPConverter> nodeConverters_;

  // dataSent_ is not required as is. It depends on the desired behavior.
  bool dataSent_ = true;
  bool dataReadyToSend_ = false;
  std::mutex dataReadyMutex_;
  std::condition_variable dataReadyCondition_;

  void start() {
    if (server_)
      throw std::runtime_error("A server is already running");
    serverThread_ = std::thread(&MimSolversPlotterImpl::runServer, this);
  }

  void shutdown() {
    if (server_) {
      server_->Shutdown();
      serverThread_.join();
    }
  }

  bool send(const std::shared_ptr<crocoddyl::ShootingProblem> &problem);

  grpc::Status GetOCPData(grpc::ServerContext *context,
                          const google::protobuf::Empty *request,
                          grpc::ServerWriter<OCPd> *writer) override {
    std::unique_lock lk(dataReadyMutex_);
    while (true) {
      dataReadyCondition_.wait(lk, [&] { return dataReadyToSend_; });
      nodes_.set_iteration(iteration_);
      for (int i = 0; i < nodeConverters_.size(); ++i) {
        nodeConverters_[i].write(nodes_.mutable_nodes()->at(i));
      }
      dataSent_ = true;
      dataReadyToSend_ = false;
      lk.unlock();
      std::cout << "GetOCPData ready to send" << std::endl;
      writer->Write(nodes_);
      std::cout << "GetOCPData sent one data" << std::endl;
    }
    // if (writer_ != NULL) {
    //   return grpc::Status(grpc::StatusCode::UNAVAILABLE,
    //                       "GetOCPData can only be called once.");
    // }
    // writer_ = writer;
    // TODO wait for something to tell when it is done
    return grpc::Status::OK;
  }

private:
  void runServer() {
    std::string server_address("0.0.0.0:1234");

    grpc::EnableDefaultHealthCheckService(true);
    grpc::ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(this);
    // Finally assemble the server.
    server_ = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server_->Wait();
  }
};

MimSolversPlotter::MimSolversPlotter()
    : mim_solvers::CallbackAbstract(), impl_(new MimSolversPlotterImpl) {
  impl_->start();
}

MimSolversPlotter::~MimSolversPlotter() { impl_->shutdown(); }

bool MimSolversPlotter::send(
    const std::shared_ptr<crocoddyl::ShootingProblem> &problem) {
  return impl_->send(problem);
}

void MimSolversPlotter::operator()(crocoddyl::SolverAbstract &solver) {
  send(solver.get_problem());
}

void MimSolversPlotter::operator()(crocoddyl::SolverAbstract &solver,
                                   std::string) {
  (*this)(solver);
}

bool MimSolversPlotterImpl::send(
    const std::shared_ptr<crocoddyl::ShootingProblem> &problem, int iteration) {
  std::cout << "start" << std::endl;
  std::unique_lock lk(dataReadyMutex_, std::defer_lock);
  // If data not sent, don't override the current data.
  if (!dataSent_)
    return false;
  std::cout << "data sent" << std::endl;
  // If mutex is locked, we are sending data. Don't wait.
  if (!lk.try_lock())
    return false;
  std::cout << "locked" << std::endl;
  iteration_ = iteration;
  const std::size_t T = problem->get_T();
  const std::size_t N = T + 1;

  const auto &rms = problem->get_runningModels();
  const auto &rds = problem->get_runningDatas();
  const auto &tm = problem->get_terminalModel();
  const auto &td = problem->get_terminalData();

  if (nodes_.nodes_size() != N || nodeConverters_.size() != N) {
    // Initialization

    nodes_.mutable_nodes()->Reserve(N);
    // TODO handle non 0 case in assert below.
    assert(nodes_.nodes_size() == 0);
    if (nodes_.nodes_size() != N) {
      for (int i = 0; i < N; ++i)
        nodes_.mutable_nodes()->Add();
    }

    nodeConverters_.resize(N);

    for (std::size_t i = 0; i < T; ++i) {
      nodeConverters_[i].init(rms[i], rds[i]);
    }
    nodeConverters_[T].init(tm, td);

    std::cout << "initialized " << N << ' ' << nodes_.nodes_size() << std::endl;
  }

  for (std::size_t i = 0; i < T; ++i) {
    nodeConverters_[i].read(rms[i], rds[i]);
  }
  nodeConverters_[T].read(tm, td);

  std::cout << "swapped" << std::endl;
  dataReadyToSend_ = true;
  lk.unlock();
  dataReadyCondition_.notify_one();
  return true;
}

} // namespace crocoddyl_plotter