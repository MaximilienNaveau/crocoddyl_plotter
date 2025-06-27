#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "QCustomPlot/qcustomplot.h"

class GRPCClient;

class MainWindow : public QMainWindow
{
  Q_OBJECT
  
public:
  explicit MainWindow(QString url, QWidget *parent = 0);
  virtual ~MainWindow();
  
  void setupBarChart();
  void setupColorMap();
  
// private slots:
//   void realtimeDataSlot();

public slots:
  void connectGRPCClient(QString url);
  void disconnectGRPCClient();
  void initializePlots(const QStringList costNames, int nNodes, QVector<int> variableIndices);
  void updatePlots(int iteration, QList<QVector<double>> costValues, double infiniteNorm);

private:
  QLabel* header = NULL;

  QCustomPlot* barPlot = NULL;
  QCPBarsGroup *barsGroup = NULL;

  QCustomPlot* colorPlot = NULL;
  QCPColorMap* colorMap = NULL;

  GRPCClient* grpcClient = NULL;
};

#endif // MAINWINDOW_H
