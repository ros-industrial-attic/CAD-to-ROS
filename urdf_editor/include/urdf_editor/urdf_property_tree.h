#ifndef __URDF_PROPERTY_TREE_H__
#define __URDF_PROPERTY_TREE_H__

#include <urdf_editor/link_property.h>
#include <urdf_editor/joint_property.h>
#include <urdf_editor/urdf_property_tree_link_item.h>
#include <urdf_editor/urdf_property_tree_joint_item.h>

namespace urdf_editor
{

  class URDFPropertyTree: public QTreeWidget
  {

    Q_OBJECT

  public:
    enum ItemType {LinkRoot = 1001, JointRoot = 1002, Link = 1003, Joint = 1004};

    URDFPropertyTree(QWidget *parent);
    virtual ~URDFPropertyTree();

    bool loadRobotModel(urdf::ModelInterfaceSharedPtr model);

    urdf::ModelInterfaceSharedPtr getRobotModel();

    QTreeWidgetItem *getSelectedItem();

    bool isJoint(QTreeWidgetItem *item);

    bool isLink(QTreeWidgetItem *item);

    bool isJointRoot(QTreeWidgetItem *item);

    bool isLinkRoot(QTreeWidgetItem *item);

    URDFPropertyTreeLinkItem *asLinkTreeItem(QTreeWidgetItem *item);

    URDFPropertyTreeJointItem *asJointTreeItem(QTreeWidgetItem *item);

    void clear();

  private slots:
    void on_itemPressed(QTreeWidgetItem *item, int column);
    void on_addActionTriggered();
    void on_removeActionTriggered();
    void on_expandActionTriggered();
    void on_collapseActionTriggered();
    void on_contextMenuRequested(const QPoint &pos);

    // Joint SLOTS
    void on_jointNameChanged(JointProperty *property, QString current_name, QString new_name);
    void on_jointParentLinkChanged(JointProperty *property, QString current_name, QString new_name);

    // Link SLOTS
    void on_linkNameChanged(LinkProperty *property, QString current_name, QString new_name);

  signals:
    void valueChanged();

    // Link SIGNALS
    void linkAddition(LinkProperty *property);
    void linkDeletion();
    void linkValueChanged(LinkProperty *property);

    // Joint SIGNALS
    void jointAddition(JointProperty *property);
    void jointDeletion();
    void jointNameChanged(JointProperty *property, QString current_name, QString new_name);
    void jointParentLinkChanged(JointProperty *property, QString current_name, QString new_name);
    void jointOriginChanged(JointProperty *property);
    void jointAxisChanged(JointProperty *property);
    void jointCalibrationChanged(JointProperty *property);
    void jointDynamicsChanged(JointProperty *property);
    void jointLimitsChanged(JointProperty *property);
    void jointMimicChanged(JointProperty *property);
    void jointSafetyChanged(JointProperty *property);
    void jointValueChanged(JointProperty *property);

  private:
    virtual void dropEvent(QDropEvent * event);
    void initialize();
    void createActions();
    void createMenus();

    bool populateFromRobotModel();
    void addItemRecursively(urdf::LinkSharedPtr link, QTreeWidgetItem* parent);
    void addItemRecursively(urdf::JointSharedPtr joint, QTreeWidgetItem* parent);

    urdf::LinkSharedPtr addModelLink();
    URDFPropertyTreeLinkItem* addLinkTreeItem(QTreeWidgetItem* parent, urdf::LinkSharedPtr link);
    void removeModelLink(urdf::LinkSharedPtr link);
    void removeLinkTreeItem(QTreeWidgetItem *item);

    urdf::JointSharedPtr addModelJoint(QString child_link_name);
    URDFPropertyTreeJointItem* addJointTreeItem(QTreeWidgetItem* parent, urdf::JointSharedPtr joint);
    void removeModelJoint(urdf::JointSharedPtr joint);
    void removeJointTreeItem(QTreeWidgetItem *item);

    QString getValidName(QString prefix, QStringList &current_names, unsigned int &counter);

    /*! Set expanded value for item and all its children (recursively) */
    void setExpandedRecursive(QTreeWidgetItem *item, bool expanded);

    /**
     * @brief This moves all of a QTreeWidgetItem's children to another QTreeWidgetItem.
     * @param parent as a *QTreeWidgetItem
     * @param new_parent as a *QTreeWidgetItem
     */
    void moveTreeChildren(QTreeWidgetItem *parent, QTreeWidgetItem *new_parent);

    void addMapping(URDFPropertyTreeLinkItem *item);
    void addMapping(URDFPropertyTreeJointItem *item);
    void removeMapping(URDFPropertyTreeLinkItem *item);
    void removeMapping(URDFPropertyTreeJointItem *item);


    urdf::ModelInterfaceSharedPtr model_;

    QTreeWidgetItem *root_;
    QTreeWidgetItem *link_root_;
    QTreeWidgetItem *joint_root_;
    QTreeWidgetItem *drag_item_;

    QStringList joint_names_;
    QStringList link_names_;
    unsigned int joint_counter_;
    unsigned int link_counter_;
    QMap<QString, URDFPropertyTreeLinkItem*> links_;
    QMap<QString, URDFPropertyTreeJointItem*> joints_;

    QMenu *context_menu_;
    QAction *add_action_;
    QAction *remove_action_;
    QAction *expand_action_;
    QAction *collapse_action_;
  };
}

#endif // __URDF_PROPERTY_TREE_H__
