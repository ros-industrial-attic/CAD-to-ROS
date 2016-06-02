
#include <urdf_editor/urdf_editor.h>

#include <QApplication>

#include <ros/ros.h>


int main(int argc, char *argv[])
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "urdf_editor");
  }

  QApplication a(argc, argv);
  URDFEditor w;
  w.show();

  return a.exec();
}
