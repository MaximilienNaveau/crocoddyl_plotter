#include "mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>

int main(int argc, char *argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif
  QApplication a(argc, argv);
  QString defaultUrl("localhost:1234");

  QCommandLineParser parser;
  parser.setApplicationDescription("Crocoddyl plotter");
  parser.addHelpOption();
  parser.addPositionalArgument("url",
                               "URL of the server. Defaults to " + defaultUrl);
  parser.process(a);
  const QStringList args = parser.positionalArguments();
  QString url(args.size() > 0 ? url = args[0] : defaultUrl);

  MainWindow w(url);
  w.show();

  return a.exec();
}
