#ifndef MYRVIZ_H
#define MYRVIZ_H

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

namespace urdf_editor
{

  class MyRviz: public QWidget
  {
    Q_OBJECT
  public:
    MyRviz(QWidget *parent = 0);
    virtual ~MyRviz();

  private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    rviz::Display *display_;
  };

}

#endif // MYRVIZ_H
