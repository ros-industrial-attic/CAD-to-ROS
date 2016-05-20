#include <QApplication>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QDropEvent>
#include <QAction>
#include <QMenu>
#include <QMap>
#include <QMessageBox>

#include <urdf_editor/urdf_property_tree.h>
#include <urdf/model.h>
#include <ros/ros.h>

namespace urdf_editor
{
  const QString ACTION_ADD_TEXT = "Add";
  const QString ACTION_REMOVE_TEXT = "Remove";
  const QString ACTION_EXPANDALL_TEXT = "Expand All";
  const QString ACTION_COLLAPSEALL_TEXT = "Collapse All";

  URDFPropertyTree::URDFPropertyTree(QWidget *parent) :
    drag_item_(NULL),
    model_(new urdf::ModelInterface()),
    joint_counter_(0),
    link_counter_(0)
  {
    setSelectionMode(QAbstractItemView::SingleSelection);
    setDragEnabled(true);
    viewport()->setAcceptDrops(true);
    setDropIndicatorShown(true);
    setDragDropMode(QAbstractItemView::InternalMove);
    setColumnCount(1);

    initialize();
    createActions();
    createMenus();

    connect(this, SIGNAL(itemPressed(QTreeWidgetItem*,int)),
            this, SLOT(on_itemPressed(QTreeWidgetItem*,int)));

    connect(this, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(on_contextMenuRequested(QPoint)));
  }

  URDFPropertyTree::~URDFPropertyTree()
  {
    delete context_menu_;
  }

  bool URDFPropertyTree::loadRobotModel(urdf::ModelInterfaceSharedPtr model)
  {
    clear();

    if (!model)
      return false;

    model_ = model;
    link_counter_ = model_->links_.size();
    joint_counter_= model_->joints_.size();

    return populateFromRobotModel();
  }

  urdf::ModelInterfaceSharedPtr URDFPropertyTree::getRobotModel()
  {
    return model_;
  }

  QTreeWidgetItem * URDFPropertyTree::getSelectedItem()
  {
    return selectedItems()[0];
  }

  bool URDFPropertyTree::isJoint(QTreeWidgetItem *item)
  {
    return (item->type() == URDFPropertyTree::Joint ? true : false);
  }

  bool URDFPropertyTree::isLink(QTreeWidgetItem *item)
  {
    return (item->type() == URDFPropertyTree::Link ? true : false);
  }

  bool URDFPropertyTree::isJointRoot(QTreeWidgetItem *item)
  {
    return (item->type() == URDFPropertyTree::JointRoot ? true : false);
  }

  bool URDFPropertyTree::isLinkRoot(QTreeWidgetItem *item)
  {
    return (item->type() == URDFPropertyTree::LinkRoot ? true : false);
  }

  URDFPropertyTreeLinkItem *URDFPropertyTree::asLinkTreeItem(QTreeWidgetItem *item)
  {
    if (isLink(item))
    {
      return dynamic_cast<URDFPropertyTreeLinkItem *>(item);
    }
    else
    {
      ROS_ERROR("Tried to convert a QTreeWidgetItem to a URDFPropertyTreeLinkItem that is not a URDFPropertyTreeLinkItem.");
      return NULL;
    }
  }

  URDFPropertyTreeJointItem *URDFPropertyTree::asJointTreeItem(QTreeWidgetItem *item)
  {
    if (isJoint(item))
    {
      return dynamic_cast<URDFPropertyTreeJointItem *>(item);
    }
    else
    {
      ROS_ERROR("Tried to convert a QTreeWidgetItem to a URDFPropertyTreeJointItem that is not a URDFPropertyTreeJointItem.");
      return NULL;
    }
  }

  void URDFPropertyTree::clear()
  {
    // Clear Links from tree
    // Based on QT documentation you must first remove all items.
    foreach(QTreeWidgetItem *item, links_)
      item->parent()->removeChild(item);

    // Also QT documentation states that a removeChild does not
    // delete the item, it just makes it safe to delete.
    foreach(QTreeWidgetItem *item, links_)
      delete item;


    // Clear Joints from tree
    foreach(QTreeWidgetItem *item, joints_)
      item->parent()->removeChild(item);

    foreach(QTreeWidgetItem *item, joints_)
      delete item;

    link_names_.clear();
    joint_names_.clear();
    link_counter_ = 0;
    joint_counter_ = 0;
    links_.clear();
    joints_.clear();
    model_->clear();
  }

  void URDFPropertyTree::initialize()
  {
    root_ = new QTreeWidgetItem(this);
    root_->setText(0, "RobotModel");
    root_->setExpanded(true);
    root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    addTopLevelItem(root_);

    link_root_ = new QTreeWidgetItem(URDFPropertyTree::LinkRoot);
    link_root_->setText(0, "Links");
    link_root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    root_->addChild(link_root_);

    joint_root_ = new QTreeWidgetItem(URDFPropertyTree::JointRoot);
    joint_root_->setText(0,"Joints");
    joint_root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    root_->addChild(joint_root_);
  }

  void URDFPropertyTree::createMenus()
  {
    context_menu_ = new QMenu(this);
    context_menu_->addAction(add_action_);
    context_menu_->addAction(remove_action_);
    context_menu_->addSeparator();
    context_menu_->addAction(expand_action_);
    context_menu_->addAction(collapse_action_);
  }

  void URDFPropertyTree::createActions()
  {
    add_action_ = new QAction(ACTION_ADD_TEXT, this);
    add_action_->setStatusTip(tr("Add new link to selected link."));
    connect(add_action_,SIGNAL(triggered()), this, SLOT(on_addActionTriggered()));

    remove_action_ = new QAction(ACTION_REMOVE_TEXT, this);
    remove_action_->setStatusTip(tr("Remove selected link."));
    connect(remove_action_,SIGNAL(triggered()), this, SLOT(on_removeActionTriggered()));

    collapse_action_ = new QAction(ACTION_COLLAPSEALL_TEXT, this);
    collapse_action_->setStatusTip(tr("Collapse selected item and all its children."));
    connect(collapse_action_,SIGNAL(triggered()), this, SLOT(on_collapseActionTriggered()));

    expand_action_ = new QAction(ACTION_EXPANDALL_TEXT, this);
    expand_action_->setStatusTip(tr("Expand selected item and all its children."));
    connect(expand_action_,SIGNAL(triggered()), this, SLOT(on_expandActionTriggered()));
  }

  bool URDFPropertyTree::populateFromRobotModel()
  {
    // add all links to the tree, starting with the root
    urdf::LinkSharedPtr rlink;
    model_->getLink(model_->getRoot()->name, rlink);
    addItemRecursively(rlink, link_root_);

    // add all joints, starting with those that have the root as parent
    std::vector<urdf::JointSharedPtr>& child_joints = rlink->child_joints;
    std::vector<urdf::JointSharedPtr>::iterator joint_it;

    for (joint_it = child_joints.begin(); joint_it != child_joints.end(); ++joint_it)
      addItemRecursively(*joint_it, joint_root_);

    return true;
  }

  void URDFPropertyTree::addItemRecursively(urdf::LinkSharedPtr link, QTreeWidgetItem *parent)
  {
    // first add the tree item
    QTreeWidgetItem* item = addLinkTreeItem(parent, link);

    // now do child links
    std::vector<urdf::LinkSharedPtr >& child_links = link->child_links;
    std::vector<urdf::LinkSharedPtr >::iterator link_it;

    for (link_it = child_links.begin(); link_it != child_links.end(); ++link_it)
      addItemRecursively(*link_it, item);  // recursive
  }

  void URDFPropertyTree::addItemRecursively(urdf::JointSharedPtr joint, QTreeWidgetItem *parent)
  {
        // see which joints are the children of the child_link
    urdf::LinkSharedPtr child_link;
    model_->getLink(joint->child_link_name, child_link);

    if (child_link)
    {
      // Check joint type, if set to UNKNOWN change it to FIXED since
      // UNKNOWN causes parsing error.
      if (joint->type == urdf::Joint::UNKNOWN)
        joint->type = urdf::Joint::FIXED;

      // first add the tree item
      URDFPropertyTreeJointItem *item = addJointTreeItem(parent, joint);

      // assign joint to link
      links_[QString::fromStdString(joint->child_link_name)]->assignJoint(item);

      std::vector<urdf::JointSharedPtr >& child_joints = child_link->child_joints;
      std::vector<urdf::JointSharedPtr >::iterator joint_it;

      for (joint_it = child_joints.begin(); joint_it != child_joints.end(); ++joint_it)
        addItemRecursively(*joint_it, item);  // recursive
    }
    else
    {
      qDebug() << QString("Can't find Link object for child_link '%s' of '%s'").arg(
        joint->child_link_name.c_str(), joint->name.c_str());
    }
  }

  urdf::LinkSharedPtr URDFPropertyTree::addModelLink()
  {
    // add link to urdf model
    QString name = getValidName("link_", link_names_, link_counter_);
    urdf::LinkSharedPtr new_link(new urdf::Link());
    new_link->name = name.toStdString();
    model_->links_.insert(std::make_pair(name.toStdString(), new_link));

    // If this is the first link added set it as the root link.
    if (model_->links_.size() == 1)
      model_->root_link_ = new_link;

    return new_link;
  }

  URDFPropertyTreeLinkItem* URDFPropertyTree::addLinkTreeItem(QTreeWidgetItem *parent, urdf::LinkSharedPtr link)
  {
    URDFPropertyTreeLinkItem* item = new URDFPropertyTreeLinkItem(link, link_names_);
    connect(item, SIGNAL(linkNameChanged(LinkProperty*, QString, QString)),
            this, SLOT(on_linkNameChanged(LinkProperty*, QString, QString)));
    connect(item, SIGNAL(valueChanged(LinkProperty*)), this, SIGNAL(linkValueChanged(LinkProperty*)));
    connect(item, SIGNAL(valueChanged(LinkProperty*)), this, SIGNAL(valueChanged()));

    // Map link name to tree widget item
    addMapping(item);
    link_names_.append(item->getName());

    parent->addChild(item);

    emit linkAddition(item->getProperty().get());

    return item;
  }

  void URDFPropertyTree::removeModelLink(urdf::LinkSharedPtr link)
  {
    std::map<std::string, urdf::LinkSharedPtr>::iterator it = model_->links_.find(link->name);
    if (it != model_->links_.end())
      model_->links_.erase(it);
    else
      ROS_ERROR("Tried to remove model link (%s) which does not exist.", link->name.c_str());
  }

  void URDFPropertyTree::removeLinkTreeItem(QTreeWidgetItem *item)
  {
    URDFPropertyTreeLinkItem *link = asLinkTreeItem(item);
    // Remove mapping between link name and tree widget item.
    removeMapping(link);
    link_names_.removeOne(link->getName());

    QTreeWidgetItem *parent = link->parent();
    moveTreeChildren(item, parent);
    parent->removeChild(item);
  }

  urdf::JointSharedPtr URDFPropertyTree::addModelJoint(QString child_link_name)
  {
    // add joint to urdf model
    QString name = getValidName("joint_", joint_names_, joint_counter_);
    urdf::JointSharedPtr new_joint(new urdf::Joint());
    new_joint->name = name.toStdString();
    new_joint->child_link_name = child_link_name.toStdString();
    new_joint->type = urdf::Joint::FIXED;
    model_->joints_.insert(std::make_pair(name.toStdString(), new_joint));

    return new_joint;
  }

  URDFPropertyTreeJointItem* URDFPropertyTree::addJointTreeItem(QTreeWidgetItem *parent, urdf::JointSharedPtr joint)
  {
    if (isJointRoot(parent))
      joint->parent_link_name = model_->getRoot()->name;
    else
      joint->parent_link_name = asJointTreeItem(parent)->getChildLinkName().toStdString();

    URDFPropertyTreeJointItem* item = new URDFPropertyTreeJointItem(joint, link_names_, joint_names_);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    connect(item, SIGNAL(jointNameChanged(JointProperty*, QString, QString)),
            this, SLOT(on_jointNameChanged(JointProperty*, QString, QString)));
    connect(item, SIGNAL(parentLinkChanged(JointProperty*, QString, QString)),
            this, SLOT(on_jointParentLinkChanged(JointProperty*, QString, QString)));

    // Pass the signal through
    connect(item, SIGNAL(typeChanged(JointProperty*)), this, SIGNAL(jointTypeChanged(JointProperty*)));
    connect(item, SIGNAL(originChanged(JointProperty*)), this, SIGNAL(jointOriginChanged(JointProperty*)));
    connect(item, SIGNAL(axisChanged(JointProperty*)), this, SIGNAL(jointAxisChanged(JointProperty*)));
    connect(item, SIGNAL(calibrationChanged(JointProperty*)), this, SIGNAL(jointCalibrationChanged(JointProperty*)));
    connect(item, SIGNAL(dynamicsChanged(JointProperty*)), this, SIGNAL(jointDynamicsChanged(JointProperty*)));
    connect(item, SIGNAL(limitsChanged(JointProperty*)), this, SIGNAL(jointLimitsChanged(JointProperty*)));
    connect(item, SIGNAL(mimicChanged(JointProperty*)), this, SIGNAL(jointMimicChanged(JointProperty*)));
    connect(item, SIGNAL(safetyChanged(JointProperty*)), this, SIGNAL(jointSafetyChanged(JointProperty*)));
    connect(item, SIGNAL(valueChanged(JointProperty*)), this, SIGNAL(jointValueChanged(JointProperty*)));
    connect(item, SIGNAL(valueChanged(JointProperty*)), this, SIGNAL(valueChanged()));

    // Map link name to tree widget item
    addMapping(item);
    joint_names_.append(item->getName());

    parent->addChild(item);

    emit jointAddition(item->getProperty().get());

    return item;
  }

  void URDFPropertyTree::removeModelJoint(urdf::JointSharedPtr joint)
  {
    std::map<std::string, urdf::JointSharedPtr>::iterator it = model_->joints_.find(joint->name);
    if (it != model_->joints_.end())
      model_->joints_.erase(it);
    else
      ROS_ERROR("Tried to remove model joint (%s) which does not exist.", joint->name.c_str());
  }

  void URDFPropertyTree::removeJointTreeItem(QTreeWidgetItem *item)
  {
    URDFPropertyTreeJointItem* joint = asJointTreeItem(item);
    QString newParentName = joint->getParentLinkName();

    // Remove mapping between joint name and tree widget item.
    removeMapping(joint);
    joint_names_.removeOne(joint->getName());

    // If it is trying to set the parent to a link that does not exist,
    // set the parent to the first link in the chain. This should only
    // occur if the user deletes the first link.
    if (!link_names_.contains(newParentName))
      newParentName = QString::fromStdString(model_->getRoot()->name);

    QTreeWidgetItem *parent = item->parent();

    for (int i=0; i < item->childCount(); i++)
    {
      asJointTreeItem(item->child(i))->setParentLinkName(newParentName);
    }

    parent->removeChild(item);
  }

  QString URDFPropertyTree::getValidName(QString prefix, QStringList &current_names, unsigned int &counter)
  {
    QString name;
    do
    {
      counter+=1;
      name = prefix + QString::number(counter);
    } while (current_names.contains(name));
    return name;
  }

  void URDFPropertyTree::setExpandedRecursive(QTreeWidgetItem *item, bool expanded)
  {
    item->setExpanded(expanded);
    for (int i=0; i < item->childCount(); i++)
      setExpandedRecursive(item->child(i), expanded);
  }

  void URDFPropertyTree::moveTreeChildren(QTreeWidgetItem *parent, QTreeWidgetItem *new_parent)
  {
    for (int i=0; i < parent->childCount(); i++)
      new_parent->addChild(parent->takeChild(i));
  }

  void URDFPropertyTree::on_itemPressed(QTreeWidgetItem *item, int column)
  {
    drag_item_ = item;
  }

  void URDFPropertyTree::on_addActionTriggered()
  {
    QTreeWidgetItem *sel = getSelectedItem();
    URDFPropertyTreeLinkItem *new_link = addLinkTreeItem(sel, addModelLink());
    sel->setExpanded(true);

    if (link_names_.count() > 2 && !isLinkRoot(sel->parent()))
    {
      URDFPropertyTreeLinkItem *sel_link = asLinkTreeItem(sel);
      URDFPropertyTreeJointItem *parent = sel_link->getAssignedJoint();

      urdf::JointSharedPtr joint = addModelJoint(new_link->getName());
      URDFPropertyTreeJointItem *new_joint = addJointTreeItem(parent, joint);
      parent->setExpanded(true);
      new_link->assignJoint(new_joint);
    }
    else if (link_names_.count() == 2 || isLinkRoot(sel->parent()))
    {
      urdf::JointSharedPtr joint = addModelJoint(new_link->getName());
      URDFPropertyTreeJointItem *new_joint = addJointTreeItem(joint_root_, joint);
      joint_root_->setExpanded(true);
      new_link->assignJoint(new_joint);
    }
  }

  void URDFPropertyTree::on_removeActionTriggered()
  {
    URDFPropertyTreeJointItem *joint;
    URDFPropertyTreeLinkItem  *sel = asLinkTreeItem(getSelectedItem());
    QTreeWidgetItem *parent = sel->parent();
    if (isLinkRoot(parent) && sel->childCount() > 1)
    {
      QMessageBox::information(this, "Invalid Operation", "The root link can only be removed if it has a single child.");
      return;
    }

    if (sel->hasAssignedJoint())
    {
      joint = sel->getAssignedJoint();
    }
    else
    {
      // This handles the removal of the root link.
      URDFPropertyTreeLinkItem *new_root_link = asLinkTreeItem(sel->child(0));
      joint = new_root_link->getAssignedJoint();
      new_root_link->unassignJoint();
      model_->root_link_ = new_root_link->getData();
    }

    removeModelLink(sel->getData());
    removeLinkTreeItem(sel);

    removeModelJoint(joint->getData());
    removeJointTreeItem(joint);

    delete sel;
    delete joint;

    emit linkDeletion();
    emit jointDeletion();
  }

  void URDFPropertyTree::on_expandActionTriggered()
  {
    setExpandedRecursive(getSelectedItem(), true);
  }

  void URDFPropertyTree::on_collapseActionTriggered()
  {
    setExpandedRecursive(getSelectedItem(), false);
  }

  void URDFPropertyTree::on_contextMenuRequested(const QPoint &pos)
  {
    if (selectedItems().isEmpty())
      return;

    QTreeWidgetItem *sel = getSelectedItem();

    if (isLinkRoot(sel) && link_names_.isEmpty() || isLink(sel))
    {
      add_action_->setEnabled(true);
      remove_action_->setEnabled(isLink(sel));
      expand_action_->setEnabled(true);
      collapse_action_->setEnabled(true);
    }
    else if (isJointRoot(sel) || isJoint(sel) || (isLinkRoot(sel) && !link_names_.isEmpty()))
    {
      add_action_->setEnabled(false);
      remove_action_->setEnabled(false);
      expand_action_->setEnabled(true);
      collapse_action_->setEnabled(true);
    }

    context_menu_->exec(mapToGlobal(pos));
  }

  void URDFPropertyTree::on_jointNameChanged(JointProperty *property, QString current_name, QString new_name)
  {
    URDFPropertyTreeJointItem *item = joints_[current_name];
    joints_.remove(current_name);
    addMapping(item);

    int idx = joint_names_.indexOf(current_name);
    joint_names_.replace(idx, new_name);

    emit jointNameChanged(property, current_name, new_name);
  }

  void URDFPropertyTree::on_jointParentLinkChanged(JointProperty *property, QString current_name, QString new_name)
  {
    URDFPropertyTreeJointItem *item = joints_[property->getName()];
    URDFPropertyTreeLinkItem *parent_link = links_[property->getParent()];
    URDFPropertyTreeLinkItem *child_link = links_[property->getChild()];
    QTreeWidgetItem *newParent;

    // if a parent joint does not exist (in the case of the first link),
    // set it to the root joint QTreeWidgetItem otherwise get the assigned joint.
    if (parent_link->hasAssignedJoint())
      newParent = parent_link->getAssignedJoint();
    else
      newParent = joint_root_;

    QTreeWidgetItem *take = item->parent()->takeChild(item->parent()->indexOfChild(item));
    newParent->addChild(take);

    // move link QTreeWidgetItem
    take = child_link->parent()->takeChild(child_link->parent()->indexOfChild(child_link));
    parent_link->addChild(take);

    emit jointParentLinkChanged(property, current_name, new_name);
  }

  void URDFPropertyTree::on_linkNameChanged(LinkProperty *property, QString current_name, QString new_name)
  {
    URDFPropertyTreeLinkItem *item = links_[current_name];
    links_.remove(current_name);
    addMapping(item);

    int idx = link_names_.indexOf(current_name);
    link_names_.replace(idx, new_name);
  }

  void URDFPropertyTree::dropEvent(QDropEvent * event)
  {

    QTreeWidget *tree = (QTreeWidget*)event->source();
    QTreeWidgetItem *item = tree->itemAt(event->pos());

    QModelIndex droppedIndex = indexAt( event->pos() );
    if( !droppedIndex.isValid() )
      return;

    QTreeWidgetItem* drop_item;
    DropIndicatorPosition dp = dropIndicatorPosition();
    // send the appropriate item and drop indicator signal
    if (dp == QAbstractItemView::OnItem || dp == QAbstractItemView::BelowItem)
    {
      drop_item = item;
    }
    else if (dp == QAbstractItemView::AboveItem)
    {
      drop_item = item->parent();
    }

    if (isLink(drag_item_) && isLink(drop_item))
      asLinkTreeItem(drag_item_)->getAssignedJoint()->setParentLinkName(asLinkTreeItem(drop_item)->getName());

    drop_item->setExpanded(true);
  }

  void URDFPropertyTree::addMapping(URDFPropertyTreeLinkItem *item)
  {
    links_.insert(item->getName(), item);
  }

  void URDFPropertyTree::addMapping(URDFPropertyTreeJointItem *item)
  {
    joints_.insert(item->getName(), item);
  }

  void URDFPropertyTree::removeMapping(URDFPropertyTreeLinkItem *item)
  {
    links_.remove(item->getName());
  }

  void URDFPropertyTree::removeMapping(URDFPropertyTreeJointItem *item)
  {
    joints_.remove(item->getName());
  }
}
