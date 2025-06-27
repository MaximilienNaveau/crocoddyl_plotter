#include "mainwindow.h"

#include <QDebug>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>
#include <QVBoxLayout>

#include "grpc-client.h"

MainWindow::MainWindow(QString url, QWidget *parent)
    : QMainWindow(parent), barPlot(new QCustomPlot),
      colorPlot(new QCustomPlot), header(new QLabel) {
  QWidget *central = new QWidget;
  QVBoxLayout *layout = new QVBoxLayout(central);
  layout->addWidget(header);
  layout->addWidget(barPlot, 1);
  layout->addWidget(colorPlot, 2);

  setCentralWidget(central);

  setGeometry(400, 250, 542, 390);

  setupBarChart();
  setupColorMap();

  setWindowTitle("QCustomPlot");
  statusBar()->clearMessage();
  // currentDemoIndex = demoIndex;
  // barPlot->replot();
  // colorPlot->replot();

  connectGRPCClient(url);
}

void MainWindow::setupBarChart() {
  // set dark background gradient:
  // QLinearGradient gradient(0, 0, 0, 400);
  // gradient.setColorAt(0, QColor(90, 90, 90));
  // gradient.setColorAt(0.38, QColor(105, 105, 105));
  // gradient.setColorAt(1, QColor(70, 70, 70));
  // barPlot->setBackground(QBrush(gradient));

  /*
  // create empty bar chart objects:
  QCPBars *regen = new QCPBars(barPlot->xAxis, barPlot->yAxis);
  QCPBars *nuclear = new QCPBars(barPlot->xAxis, barPlot->yAxis);
  QCPBars *fossil = new QCPBars(barPlot->xAxis, barPlot->yAxis);
  regen->setAntialiased(false); // gives more crisp, pixel aligned bar borders
  nuclear->setAntialiased(false);
  fossil->setAntialiased(false);
  regen->setStackingGap(1);
  nuclear->setStackingGap(1);
  fossil->setStackingGap(1);
  // set names and colors:
  fossil->setName("Fossil fuels");
  fossil->setPen(QPen(QColor(111, 9, 176).lighter(170)));
  fossil->setBrush(QColor(111, 9, 176));
  nuclear->setName("Nuclear");
  nuclear->setPen(QPen(QColor(250, 170, 20).lighter(150)));
  nuclear->setBrush(QColor(250, 170, 20));
  regen->setName("Regenerative");
  regen->setPen(QPen(QColor(0, 168, 140).lighter(130)));
  regen->setBrush(QColor(0, 168, 140));
  // stack bars on top of each other:
  nuclear->moveAbove(fossil);
  regen->moveAbove(nuclear);

  // prepare x axis with country labels:
  QVector<double> ticks;
  QVector<QString> labels;
  ticks << 1 << 2 << 3 << 4 << 5 << 6 << 7;
  labels << "USA" << "Japan" << "Germany" << "France" << "UK" << "Italy" <<
  "Canada"; QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
  textTicker->addTicks(ticks, labels);
  barPlot->xAxis->setTicker(textTicker);
  barPlot->xAxis->setTickLabelRotation(60);
  barPlot->xAxis->setSubTicks(false);
  barPlot->xAxis->setTickLength(0, 4);
  barPlot->xAxis->setRange(0, 8);
  barPlot->xAxis->setBasePen(QPen(Qt::white));
  barPlot->xAxis->setTickPen(QPen(Qt::white));
  barPlot->xAxis->grid()->setVisible(true);
  barPlot->xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
  barPlot->xAxis->setTickLabelColor(Qt::white);
  barPlot->xAxis->setLabelColor(Qt::white);

  // prepare y axis:
  barPlot->yAxis->setRange(0, 12.1);
  barPlot->yAxis->setPadding(5); // a bit more space to the left border
  barPlot->yAxis->setBasePen(QPen(Qt::white));
  barPlot->yAxis->setTickPen(QPen(Qt::white));
  barPlot->yAxis->setSubTickPen(QPen(Qt::white));
  barPlot->yAxis->grid()->setSubGridVisible(true);
  barPlot->yAxis->setTickLabelColor(Qt::white);
  barPlot->yAxis->setLabelColor(Qt::white);
  barPlot->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
  barPlot->yAxis->grid()->setSubGridPen(QPen(QColor(130, 130, 130), 0,
  Qt::DotLine));

  // Add data:
  QVector<double> fossilData, nuclearData, regenData;
  fossilData  << 0.86*10.5 << 0.83*5.5 << 0.84*5.5 << 0.52*5.8 << 0.89*5.2 <<
  0.90*4.2 << 0.67*11.2; nuclearData << 0.08*10.5 << 0.12*5.5 << 0.12*5.5 <<
  0.40*5.8 << 0.09*5.2 << 0.00*4.2 << 0.07*11.2; regenData   << 0.06*10.5 <<
  0.05*5.5 << 0.04*5.5 << 0.06*5.8 << 0.02*5.2 << 0.07*4.2 << 0.25*11.2;
  fossil->setData(ticks, fossilData);
  nuclear->setData(ticks, nuclearData);
  regen->setData(ticks, regenData);
  */
  barPlot->xAxis->setLabel("OCP node");
  barPlot->yAxis->setLabel("Cost value");

  // setup legend:
  barPlot->legend->setVisible(true);
  barPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop |
                                                               Qt::AlignRight);
  barPlot->legend->setBrush(QColor(255, 255, 255, 100));
  barPlot->legend->setBorderPen(Qt::NoPen);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  barPlot->legend->setFont(legendFont);
  // barPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

void MainWindow::setupColorMap() {
  // configure axis rect:
  colorPlot->setInteractions(
      QCP::iRangeDrag | QCP::iRangeZoom); // this will also allow rescaling the
                                          // color scale by dragging/zooming
  colorPlot->axisRect()->setupFullAxesBox(true);
  colorPlot->xAxis->setLabel("OCP variables");
  colorPlot->yAxis->setLabel("Cost");

  // set up the QCPColorMap:
  colorMap = new QCPColorMap(colorPlot->xAxis, colorPlot->yAxis);
  colorMap->setInterpolate(false);

  // add a color scale:
  QCPColorScale *colorScale = new QCPColorScale(colorPlot);
  colorPlot->plotLayout()->addElement(1, 0, colorScale);
  colorScale->setType(QCPAxis::atBottom);
  colorMap->setColorScale(colorScale);
  colorScale->axis()->setLabel("Jacobian value");

  // set the color gradient of the color map to one of the presets:
  QCPColorGradient gradient;
  gradient.setColorStops({
      {0.0, QColor(0, 0, 255, 200)},
      {0.5, QColor(255, 255, 255, 200)},
      {1.0, QColor(255, 0, 0, 200)},
  });
  colorMap->setGradient(gradient);
  // colorMap->setGradient(QCPColorGradient::gpPolar);
  // colorMap->gradient().setColorStopAt(0.5, QColor("white"));
  // we could have also created a QCPColorGradient instance and added own colors
  // to the gradient, see the documentation of QCPColorGradient for what's
  // possible.

  // rescale the data dimension (color) such that all data points lie in the
  // span visualized by the color gradient:
  // colorMap->rescaleDataRange();

  // make sure the axis rect and color scale synchronize their bottom and top
  // margins (so they line up):
  // QCPMarginGroup *marginGroup = new QCPMarginGroup(colorPlot);
  // colorPlot->axisRect()->setMarginGroup(QCP::msBottom | QCP::msTop,
  //                                       marginGroup);
  // colorScale->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup);

  // rescale the key (x) and value (y) axes so the whole color map is visible:
  // colorPlot->rescaleAxes();
}

void MainWindow::disconnectGRPCClient() {
  if (grpcClient != NULL) {
    grpcClient->requestStop();
    grpcClient->wait();
    grpcClient->deleteLater();
    grpcClient = NULL;
  }
}

void MainWindow::connectGRPCClient(QString url) {
  disconnectGRPCClient();
  qDebug() << "Connecting to" << url;
  grpcClient = new GRPCClient(url.toStdString(), this);
  grpcClient->setColorMapData(colorMap->data());
  grpcClient->connect();

  connect(grpcClient, &GRPCClient::initialize, this,
          &MainWindow::initializePlots);
  connect(grpcClient, &GRPCClient::dataUpdated, this, &MainWindow::updatePlots);

  grpcClient->start();
}

void MainWindow::initializePlots(const QStringList costNames, int nNodes, QVector<int> variableIndices) {
  qDebug() << costNames;
  // QList<QCPBars*> bars;
  QVector<double> keys(nNodes);
  QVector<double> zeros(nNodes, 0.0);
  for (int i = 0; i < nNodes; ++i)
    keys[i] = i;
  barsGroup = new QCPBarsGroup(barPlot);

  int k = 0;
  const float ratio = 360.0 / costNames.size();
  for (QString const &name : costNames) {
    QCPBars *bars = new QCPBars(barPlot->xAxis, barPlot->yAxis);
    bars->setName(name);
    bars->setData(keys, zeros);
    bars->setWidth(0.9 / costNames.size());
    QColor color(QColor::fromHsv(int(k * ratio), 200, 128));
    bars->setPen(QPen(color.lighter(170)));
    bars->setBrush(color);
    // bars.append(bars);
    bars->setBarsGroup(barsGroup);
    // barsGroup->append(bars);
    k += 1;
  }

  barPlot->xAxis->setRange(0, nNodes);

  colorPlot->rescaleAxes();

  QSharedPointer<QCPAxisTickerText> xTextTicker(new QCPAxisTickerText);
  int ivar = 0;
  for (int i = 0; i < variableIndices.size() / 2; ++i) {
    xTextTicker->addTick(ivar, QString("x%1").arg(i));
    ivar += variableIndices[2*i];
    xTextTicker->addTick(ivar, QString("u%1").arg(i));
    ivar += variableIndices[2*i+1];
  }
  colorPlot->xAxis->setTicker(xTextTicker);

  QSharedPointer<QCPAxisTickerText> yTextTicker(new QCPAxisTickerText);
  yTextTicker->addTicks(keys, costNames);
  colorPlot->yAxis->setTicker(yTextTicker);
  colorPlot->yAxis->setTickLabelRotation(60);
  colorPlot->yAxis->setSubTicks(false);
  colorPlot->yAxis->grid()->setPen(QPen(QColor("gray"), 1, Qt::SolidLine));
  colorPlot->yAxis->grid()->setVisible(true);
  // colorPlot->yAxis->setTickLength(0, 4);
  // colorPlot->yAxis->setRange(0, 8);
  // colorPlot->yAxis->setBasePen(QPen(Qt::white));
  // colorPlot->yAxis->setTickPen(QPen(Qt::white));
  // colorPlot->yAxis->grid()->setVisible(true);
  // colorPlot->yAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
  // colorPlot->yAxis->setTickLabelColor(Qt::white);
  // colorPlot->yAxis->setLabelColor(Qt::white);
  
}

void MainWindow::updatePlots(int iteration, QList<QVector<double>> costValues,
                             double infiniteNorm) {
  header->setText(QString("Iteration: %1").arg(iteration));
  QVector<double> keys(costValues[0].size());
  for (int i = 0; i < keys.size(); ++i)
    keys[i] = i;
  for (int i = 0; i < costValues.size(); ++i) {
    QCPBars *bars = barsGroup->bars(i);
    bars->setData(keys, costValues[i]);
  }
  barPlot->yAxis->rescale(true);
  barPlot->replot();

  colorMap->setDataRange(QCPRange(-infiniteNorm, infiniteNorm));
  colorPlot->replot();
}

/*
void MainWindow::realtimeDataSlot()
{
  static QTime time(QTime::currentTime());
  // calculate two new data points:
  double key = time.elapsed()/1000.0; // time elapsed since start of demo, in
seconds static double lastPointKey = 0; if (key-lastPointKey > 0.002) // at most
add point every 2 ms
  {
    // add data to lines:
    ui->customPlot->graph(0)->addData(key,
qSin(key)+qrand()/(double)RAND_MAX*1*qSin(key/0.3843));
    ui->customPlot->graph(1)->addData(key,
qCos(key)+qrand()/(double)RAND_MAX*0.5*qSin(key/0.4364));
    // rescale value (vertical) axis to fit the current data:
    //ui->customPlot->graph(0)->rescaleValueAxis();
    //ui->customPlot->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->customPlot->xAxis->setRange(key, 8, Qt::AlignRight);
  ui->customPlot->replot();

  // calculate frames per second:
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key-lastFpsKey > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
      QString("%1 FPS, Total Data points: %2")
      .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
      .arg(ui->customPlot->graph(0)->data()->size()+ui->customPlot->graph(1)->data()->size())
      , 0);
      lastFpsKey = key;
      frameCount = 0;
    }
  }
  */

MainWindow::~MainWindow() { disconnectGRPCClient(); }
