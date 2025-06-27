#include "grpc-client.h"

GRPCClient::GRPCClient(std::string url, QObject *parent)
    : QThread(parent), url_(url) {}

void GRPCClient::connect() {
  auto channel = grpc::CreateChannel(url_, grpc::InsecureChannelCredentials());
  stub_ = CrocoddylPlotter::NewStub(channel);
}

void GRPCClient::run() {
  grpc::ClientContext context;
  OCPd ocpData;
  bool first = true;

  std::unique_ptr<grpc::ClientReader<OCPd>> reader(
      stub_->GetOCPData(&context, google::protobuf::Empty()));
  while (reader->Read(&ocpData)) {
    if (first) {
      initializeCostNames(ocpData);
      emit initialize(costNames_, costValues_.size(), jacobianXUIndices_);
      first = false;
    }
    double infnorm = setData(ocpData);
    emit dataUpdated(ocpData.iteration(), costValues_, infnorm);

    if (requestStop_) {
      context.TryCancel();
      break;
    }

    //   std::cout << "Found feature called " << feature.name() << " at "
    //             << feature.location().latitude() / kCoordFactor_ << ", "
    //             << feature.location().longitude() / kCoordFactor_ <<
    //             std::endl;
  }
  grpc::Status status = reader->Finish();
  if (status.ok()) {
    qDebug() << "RPC call GetOCPData succeeded.";
  } else {
    qDebug() << "RPC call GetOCPData failed.";
  }
}

void GRPCClient::initializeCostNames(OCPd const &data) {
  costIndices_.resize(data.nodes_size());
  // costValues_.resize(data.nodes_size());
  jacobianXUIndices_.resize(data.nodes_size() * 2);

  int nvar = 0;

  // QCPColorMapData* costJacobian_;
  for (int i = 0; i < data.nodes_size(); ++i) {

    Noded const &node = data.nodes(i);
    costIndices_[i].resize(node.costs_size(), -1);
    // costValues_[i].resize(node.costs_size(), 0.0);
    int nx = 0, nu = 0;
    for (int c = 0; c < node.costs_size(); c++) {
      Costd const &cost = node.costs(c);
      if (cost.name().size() == 0)
        throw std::runtime_error(
            "Initialization message should have cost name");
      QString name(QString::fromStdString(cost.name()));
      int index = costNames_.indexOf(name);
      if (index == -1) {
        index = costNames_.size();
        costNames_.append(name);
        costValues_.append(QVector<double>(data.nodes_size(), 0.0));
      }
      costIndices_[i][c] = index;

      nx = cost.lx_size();
      nu = cost.lu_size();
    }
    jacobianXUIndices_[2 * i] = nx;
    jacobianXUIndices_[2 * i + 1] = nu;
    nvar += nx + nu;
  }

  costJacobian_->setSize(nvar, costNames_.size());
  costJacobian_->setRange(QCPRange(0, nvar - 1),
                          QCPRange(-0.5, costNames_.size()-0.5));
}

double GRPCClient::setData(OCPd const &data) {
  int ivar = 0;
  double infnorm = 0.0;

  for (int i = 0; i < data.nodes_size(); ++i) {
    Noded const &node = data.nodes(i);
    int nx = 0, nu = 0;
    for (int c = 0; c < node.costs_size(); c++) {
      Costd const &cost = node.costs(c);
      int cv = costIndices_[i][c];
      costValues_[cv][i] = cost.cost();
      for (int ix = 0; ix < cost.lx_size(); ++ix) {
        infnorm = std::max(std::abs(cost.lx(ix)), infnorm);
        costJacobian_->setData(ivar++, cv, cost.lx(ix));
      }
      for (int iu = 0; iu < cost.lu_size(); ++iu) {
        infnorm = std::max(std::abs(cost.lu(iu)), infnorm);
        costJacobian_->setData(ivar++, cv, cost.lu(iu));
      }
    }
  }

  return infnorm;
  // costJacobian_->setSize(nvar, costNames_.size());
  // costJacobian_->setRange(QCPRange(0, nvar-1), QCPRange(0,
  // costNames_.size()));
}