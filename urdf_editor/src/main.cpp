#include "include/urdf_editor/urdf_editor.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  URDFEditor w;
  w.show();

  return a.exec();
}
