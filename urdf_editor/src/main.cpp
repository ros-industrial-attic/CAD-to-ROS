#include "urdf_editor/urdf_editor.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "urdf_builder");
  }

  QApplication a(argc, argv);
  URDFEditor w;
  w.show();

  return a.exec();
}
