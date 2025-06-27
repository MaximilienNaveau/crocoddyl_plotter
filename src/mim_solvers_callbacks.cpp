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

class CrocoddylPlotterImpl final : public CrocoddylPlotter::Service {
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

    std::vector<Scalar> weights;
    std::vector<Scalar> cost;
    std::vector<VectorXs> Lx;
    std::vector<VectorXs> Lu;
    std::vector<std::string> names;

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
      weights.resize(cmc.size());
      Lx.resize(cmc.size());
      Lu.resize(cmc.size());
      names.resize(cmc.size());
      int i = 0;
      for (auto const &it : cdc) {
        Lx[i].resizeLike(it.second->Lx);
        Lu[i].resizeLike(it.second->Lu);
        Lx[i].setZero();
        Lu[i].setZero();
        names[i] = it.first;
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
      auto &cm = get_costs(am);
      auto &cd = get_costs(ad);

      const auto &cmc = cm->get_costs();
      const auto &cdc = cd->costs;

      int i = 0;
      auto it_m = cmc.cbegin();
      for (auto const &it_d : cdc) {
        cost[i] = it_d.second->cost;
        weights[i] = it_m->second->weight;
        Lx[i].swap(it_d.second->Lx);
        Lu[i].swap(it_d.second->Lu);
        ++i;
        ++it_m;
      }
    }

    void write(Noded &node, bool writeNames) {
      auto *cs = node.mutable_costs();

      cs->Reserve(cost.size());
      if (cs->size() != cost.size()) {
        // TODO handle non 0 case in assert below.
        assert(cs->size() == 0);
        for (int i = 0; i < cost.size(); ++i)
          cs->Add();
      }

      for (int i = 0; i < cost.size(); ++i) {
        const Scalar& w = weights[i];
        auto &c = cs->at(i);
        c.mutable_lx()->Resize(Lx[i].size(), Scalar(0));
        c.mutable_lu()->Resize(Lu[i].size(), Scalar(0));
        c.set_cost(w*cost[i]);
        VectorMapXs(c.mutable_lx()->mutable_data(), Lx[i].size()) = w*Lx[i];
        VectorMapXs(c.mutable_lu()->mutable_data(), Lu[i].size()) = w*Lu[i];
        if (writeNames)
          c.set_name(names[i]);
        else
          c.clear_name();
        // c.mutable_lx()->Assign(Lx[i].data(), Lx[i].data() + Lx[i].size());
        // c.mutable_lu()->Assign(Lu[i].data(), Lu[i].data() + Lu[i].size());
      }
    }
  };
  std::vector<OCPConverter> nodeConverters_;

  // dataSent_ is not required as is. It depends on the desired behavior.
  bool dataSent_ = true;
  bool dataReadyToSend_ = false;
  bool shutdownRequested_ = false;
  std::mutex dataReadyMutex_;
  std::condition_variable dataReadyCondition_;

  void start(std::string url) {
    if (server_)
      throw std::runtime_error("A server is already running");
    serverThread_ = std::thread(&CrocoddylPlotterImpl::runServer, this, url);
  }

  void shutdown() {
    if (server_) {
      std::cout << "Server shutting down..." << std::flush;
      shutdownRequested_ = true;
      dataReadyCondition_.notify_one();
      server_->Shutdown();
      serverThread_.join();
      std::cout << " done." << std::endl;
    }
  }

  bool send(const std::shared_ptr<crocoddyl::ShootingProblem> &problem,
            int iteration);

  grpc::Status GetOCPData(grpc::ServerContext *context,
                          const google::protobuf::Empty *request,
                          grpc::ServerWriter<OCPd> *writer) override {
    bool firstMessage = true;
    std::unique_lock lk(dataReadyMutex_, std::defer_lock);

    while (true) {
      lk.lock();
      // std::cout << "GetOCPData waits to sent data" << std::endl;
      while (!(shutdownRequested_ || dataReadyToSend_))
        dataReadyCondition_.wait(lk);
      // lk, [&] { return shutdownRequested_ || dataReadyToSend_; });
      if (shutdownRequested_ || context->IsCancelled()) {
        std::cout << "GetOCPData stopping." << std::endl;
        lk.unlock();
        break;
      }

      nodes_.set_iteration(iteration_);
      for (int i = 0; i < nodeConverters_.size(); ++i) {
        nodeConverters_[i].write(nodes_.mutable_nodes()->at(i), firstMessage);
      }
      firstMessage = false;
      dataSent_ = true;
      dataReadyToSend_ = false;
      // std::cout << "Unlocking" << std::endl;
      lk.unlock();
      // std::cout << "GetOCPData ready to send" << std::endl;
      writer->Write(nodes_);
      // std::cout << "GetOCPData sent one data" << std::endl;
    }
    return grpc::Status::OK;
  }

private:
  void runServer(std::string const server_address) {

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

CrocoddylPlotterServer::CrocoddylPlotterServer(std::string url)
    : impl_(new CrocoddylPlotterImpl) {
  impl_->start(url);
}

CrocoddylPlotterServer::~CrocoddylPlotterServer() { impl_->shutdown(); }

bool CrocoddylPlotterServer::send(
    const std::shared_ptr<crocoddyl::ShootingProblem> &problem, int iteration) {
  return impl_->send(problem, iteration);
}

bool CrocoddylPlotterImpl::send(
    const std::shared_ptr<crocoddyl::ShootingProblem> &problem, int iteration) {
  std::unique_lock lk(dataReadyMutex_, std::defer_lock);
  // If data not sent, don't override the current data.
  // If mutex is locked, we are sending data. Don't wait.
  if (!lk.try_lock())
    return false;
  if (!dataSent_) {
    lk.unlock();
    return false;
  }
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
  }

  for (std::size_t i = 0; i < T; ++i) {
    nodeConverters_[i].read(rms[i], rds[i]);
  }
  nodeConverters_[T].read(tm, td);

  dataSent_ = false;
  dataReadyToSend_ = true;
  lk.unlock();
  dataReadyCondition_.notify_one();
  return true;
}

} // namespace crocoddyl_plotter