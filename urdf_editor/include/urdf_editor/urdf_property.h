#ifndef __URDF_PROPERTY_H__
#define __URDF_PROPERTY_H__

#include <QtCore>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMenu>

#include <qttreepropertybrowser.h>
#include <urdf_editor/urdf_property_tree.h>
#include <urdf_editor/my_rviz.h>

#include <urdf_editor/urdf_types.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/qt_types.h>


namespace urdf_editor
{
  class URDFTransformer;

  class URDFProperty : public QObject
  {
    Q_OBJECT
  public:
    URDFProperty(URDFPropertyTree *tree_widget, QWidget *browser_parent, QWidget *rviz_parent);
    ~URDFProperty();

    bool loadURDF(QString file_path);

    bool saveURDF(QString file_path);

    bool clear();

    bool unsavedChanges;

    void requestCollisionVisualizationEnabled(bool b);
    void requestVisualizationEnabled(bool b);

  private slots:
    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    void on_propertyWidget_customContextMenuRequested(const QPoint &pos);
    void on_propertyWidget_jointParentLinkChanged(JointProperty *property, QString current_name, QString new_name);
    void on_propertyWidget_jointOriginChanged(JointProperty *property);
    void on_propertyWidget_jointAxisChanged(JointProperty *property);

    void on_propertyWidget_linkValueChanged(LinkProperty *property);

    void on_unsavedChanges();
    void on_unsavedChanges(JointProperty *property);

  private:
    bool redrawRobotModel();

    boost::shared_ptr<QtTreePropertyBrowser> property_editor_;
    URDFPropertyTree *tree_widget_;
    QWidget *browser_parent_;
    urdf_editor::MyRviz *rviz_widget_;
    boost::shared_ptr<URDFTransformer> tf_transformer_;
    bool loading_;
  };
}

#endif // __URDF_PROPERTY_H__
