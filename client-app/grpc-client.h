#include <QObject>
#include <QThread>
#include "QCustomPlot/qcustomplot.h"

#include "crocoddyl.grpc.pb.h"
#include <grpcpp/grpcpp.h>

using namespace crocoddyl_plotter;

class GRPCClient : public QThread {
  Q_OBJECT
public:
  explicit GRPCClient(std::string url, QObject *parent = nullptr);
  void setColorMapData(QCPColorMapData* data) {
    costJacobian_ = data;
  }

public slots:
  void connect();
  void requestStop() {
    requestStop_ = true;
  }

protected:
  void run() override;

signals:
  void initialize(const QStringList costNames, int nNodes, QVector<int> variableIndices);
  void dataUpdated(int iteration, QList<QVector<double>> costValues, double infiniteNorm);

private:
  void initializeCostNames(OCPd const &data);
  double setData(OCPd const &data);

  QStringList costNames_;
  QList<QVector<int>> costIndices_;
  QList<QVector<double>> costValues_;

  QCPColorMapData *costJacobian_ = NULL;
  QVector<int> jacobianXUIndices_;

  bool requestStop_ = false;
  std::string url_;
  std::unique_ptr<CrocoddylPlotter::Stub> stub_;
};