#ifndef INDUSTRIAL_ROBOT_BUILDER_H
#define INDUSTRIAL_ROBOT_BUILDER_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QTreeWidgetItem>
#include <urdf_parser/urdf_parser.h>

namespace Ui {
class IndustrialRobotBuilder;
}

class IndustrialRobotBuilder : public QMainWindow
{
  Q_OBJECT

public:
  explicit IndustrialRobotBuilder(QWidget *parent = 0);
  ~IndustrialRobotBuilder();

void addLink();

void addJoint(QTreeWidgetItem *parent);

bool isJoint(QTreeWidgetItem *itm);

QString getValidName(QString prefix, QList<QString> &current_names);

private slots:
void on_robotTreeWidget_customContextMenuRequested(const QPoint &pos);

void on_robotTreeWidget_itemClicked(QTreeWidgetItem *item, int column);

private:
  Ui::IndustrialRobotBuilder *ui;
  QTreeWidgetItem *root_;
  QTreeWidgetItem *links_;
  QTreeWidgetItem *joints_;
  QList<QString> link_names;
  QList<QString> joint_names;
  QWidget *link_page_;
  QWidget *joint_page_;
  boost::shared_ptr<urdf::ModelInterface> model_;


  class QtTreePropertyBrowser *property_editor_;
};

#endif // INDUSTRIAL_ROBOT_BUILDER_H
