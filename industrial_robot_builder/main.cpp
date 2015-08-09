#include "industrial_robot_builder.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  IndustrialRobotBuilder w;
  w.show();

  return a.exec();
}
