
#include <QVBoxLayout>

#include <urdf_parser/urdf_parser.h>

#include <urdf_editor/urdf_property.h>

#include <urdf_editor/link_collision_property.h>
#include <urdf_editor/link_inertial_property.h>
#include <urdf_editor/link_new_material_property.h>
#include <urdf_editor/link_visual_property.h>
#include <urdf_editor/link_property.h>

#include <urdf_editor/joint_property.h>
#include <exception>


const QString PROPERTY_NAME_TEXT = "Name";
const QString PROPERTY_COLLISION_TEXT = "Collision";
const QString PROPERTY_VISUAL_TEXT = "Visual";
const QString PROPERTY_INERTIAL_TEXT = "Inertial";
const QString PROPERTY_ORIGIN_TEXT = "Origin";
const QString PROPERTY_GEOMETRY_TEXT = "Geometry";
const QString PROPERTY_MATERIAL_TEXT = "Material";
const QString PROPERTY_AXIS_TEXT = "Axis";
const QString PROPERTY_CALIBRATION_TEXT = "Calibration";
const QString PROPERTY_DYNAMICS_TEXT = "Dynamics";
const QString PROPERTY_LIMIT_TEXT = "Limit";
const QString PROPERTY_MIMIC_TEXT = "Mimic";
const QString PROPERTY_SAFETY_TEXT = "Safety";

const QString ACTION_ADD_TEXT = "Add";
const QString ACTION_REMOVE_TEXT = "Remove";
const QString ACTION_EXPANDALL_TEXT = "Expand All";
const QString ACTION_COLLAPSEALL_TEXT = "Collapse All";

namespace urdf_editor
{
  URDFProperty::URDFProperty(URDFPropertyTree *tree_widget, QWidget *browser_parent, QWidget *rviz_parent)
  {
    tree_widget_ = tree_widget;
    browser_parent_ = browser_parent;
    model_.reset(new urdf::ModelInterface());

    root_ = new QTreeWidgetItem(tree_widget_);
    root_->setText(0, "RobotModel");
    root_->setExpanded(true);
    root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    tree_widget_->addTopLevelItem(root_);

    link_root_ = new QTreeWidgetItem();
    link_root_->setText(0, "Links");
    link_root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable );

    root_->addChild(link_root_);

    joint_root_ = new QTreeWidgetItem();
    joint_root_->setText(0,"Joints");
    joint_root_->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDropEnabled);
    root_->addChild(joint_root_);

    property_editor_.reset(new QtTreePropertyBrowser());
    property_editor_->setContextMenuPolicy(Qt::CustomContextMenu);
    property_editor_->setPropertiesWithoutValueMarked(false);
    property_editor_->setRootIsDecorated(true);
    property_editor_->setResizeMode(QtTreePropertyBrowser::ResizeToContents);

    QVBoxLayout *vlayout = new QVBoxLayout(browser_parent_);
    vlayout->setMargin(0);
    vlayout->addWidget(property_editor_.get());

    rviz_widget_ = new urdf_editor::MyRviz(rviz_parent);

    connect(tree_widget, SIGNAL(customContextMenuRequested(QPoint)),
              this, SLOT(on_treeWidget_customContextMenuRequested(QPoint)));

    connect(tree_widget, SIGNAL(itemClicked(QTreeWidgetItem*,int)),
              this, SLOT(on_treeWidget_itemClicked(QTreeWidgetItem*,int)));

    connect(tree_widget, SIGNAL(dragDropEvent(QTreeWidgetItem*,QTreeWidgetItem*)),
            this, SLOT(on_treeWidget_itemDragDrop(QTreeWidgetItem*,QTreeWidgetItem*)));

    connect(property_editor_.get(), SIGNAL(customContextMenuRequested(QPoint)),
              this, SLOT(on_propertyWidget_customContextMenuRequested(QPoint)));

    connect(this, SIGNAL(jointAddition()), SLOT(on_unsavedChanges()));

    connect(this, SIGNAL(jointDeletion()), SLOT(on_unsavedChanges()));

    connect(this, SIGNAL(linkAddition()), SLOT(on_unsavedChanges()));

    connect(this, SIGNAL(linkDeletion()), SLOT(on_unsavedChanges()));

    // No changes to be saved, yet
    unsavedChanges = false;
  }

  URDFProperty::~URDFProperty()
  {
  }

  bool URDFProperty::clear()
  {
    // Clear Links from tree
    while (link_root_->childCount() > 0)
    {
      link_root_->removeChild(link_root_->child(0));
    }

    // Clear Joints from tree
    while (joint_root_->childCount() > 0)
    {
      joint_root_->removeChild(joint_root_->child(0));
    }

    rviz_widget_->clear();
    model_->clear();
    property_editor_->clear();
    link_to_ltree_.clear();
    joint_child_to_ctree_.clear();
    ctree_to_joint_property_.clear();
    joint_property_to_ctree_.clear();
    ltree_to_link_property_.clear();
    link_property_to_ltree_.clear();
    link_names_.clear();
    joint_names_.clear();
    unsavedChanges = false;
  }

  bool URDFProperty::loadURDF(QString file_path)
  {
    model_ = urdf::parseURDFFile(file_path.toStdString());

    if (!model_)
      return false;

    if (!populateTreeWidget())
      return false;

    return rviz_widget_->loadRobot(model_);
  }

  bool URDFProperty::saveURDF(QString file_path)
  {
    TiXmlDocument* doc = urdf::exportURDF(model_);
    TiXmlDeclaration decl("1.0", "", "");
    doc->InsertBeforeChild(doc->RootElement(), decl);
    bool savedCorrectly = false;
    savedCorrectly = doc->SaveFile(file_path.toStdString());

    return savedCorrectly;
  }

  bool URDFProperty::populateTreeWidget()
  {
    // add all links to the tree, starting with the root
    urdf::LinkSharedPtr rlink;
    model_->getLink(model_->getRoot()->name, rlink);
    addToTreeWidget(rlink, link_root_);

    // add all joints, starting with those that have the root as parent
    std::vector<urdf::JointSharedPtr>& child_joints = rlink->child_joints;
    std::vector<urdf::JointSharedPtr>::iterator joint_it;

    for (joint_it = child_joints.begin(); joint_it != child_joints.end(); ++joint_it)
      addToTreeWidget(*joint_it, joint_root_);

    return true;
  }

  void URDFProperty::setAllChildrenExpandedValue(QTreeWidgetItem *parent, bool expanded)
  {
    QTreeWidgetItem *child;
    for (int i=0; i < parent->childCount(); i++)
    {
      child = parent->child(i);
      child->setExpanded(expanded);
      setAllChildrenExpandedValue(child, expanded);
    }
  }

  void URDFProperty::addToTreeWidget(urdf::LinkSharedPtr link, QTreeWidgetItem* parent)
  {
    // first add the tree item
    QTreeWidgetItem* item = addLinkTreeItem(parent, link);
    // now add the property
    LinkPropertySharedPtr lpptr = addLinkProperty(item, link);

    // now do child links
    std::vector<urdf::LinkSharedPtr >& child_links = link->child_links;
    std::vector<urdf::LinkSharedPtr >::iterator link_it;

    for (link_it = child_links.begin(); link_it != child_links.end(); ++link_it)
      addToTreeWidget(*link_it, item);  // recursive
  }

  void URDFProperty::addToTreeWidget(urdf::JointSharedPtr joint, QTreeWidgetItem* parent)
  {
    // first add the tree item
    QTreeWidgetItem* item = addJointTreeItem(parent, joint);

    // now add the property
    JointPropertySharedPtr jpptr =  addJointProperty(item, joint);

    // see which joints are the children of the child_link
    urdf::LinkSharedPtr child_link;
    model_->getLink(joint->child_link_name, child_link);

    if (child_link)
    {
      std::vector<urdf::JointSharedPtr >& child_joints = child_link->child_joints;
      std::vector<urdf::JointSharedPtr >::iterator joint_it;

      for (joint_it = child_joints.begin(); joint_it != child_joints.end(); ++joint_it)
        addToTreeWidget(*joint_it, item);  // recursive
    }
    else
    {
      qDebug() << QString("Can't find Link object for child_link '%s' of '%s'").arg(
        joint->child_link_name.c_str(), joint->name.c_str());
    }
  }

  urdf::LinkSharedPtr URDFProperty::addModelLink(QTreeWidgetItem* parent)
  {
    // add link to urdf model
    QString name = getValidName("link_", link_names_);
    urdf::LinkSharedPtr new_link(new urdf::Link());
    new_link->name = name.toStdString();
    model_->links_.insert(std::make_pair(name.toStdString(), new_link));

    // TODO: adding tree items and creating properties is not this methods responsibility
    // first add the tree item
    QTreeWidgetItem* item = addLinkTreeItem(parent, new_link);
    // now add the property
    addLinkProperty(item, new_link);

    emit linkAddition();
    return new_link;
  }

  QTreeWidgetItem* URDFProperty::addLinkTreeItem(QTreeWidgetItem* parent, urdf::LinkSharedPtr link)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);
    item->setText(0, QString::fromStdString(link->name));

    // add mapping between link and link QTreeWidgetItem
    mapLinkToLinkTreeItem(link, item);

    return item;
  }

  LinkPropertySharedPtr URDFProperty::addLinkProperty(QTreeWidgetItem* item, urdf::LinkSharedPtr link)
  {
    LinkPropertySharedPtr tree_link(new LinkProperty(link));
    QObject::connect(tree_link.get(), SIGNAL(linkNameChanged(LinkProperty *, const QVariant &)),
              this, SLOT(on_propertyWidget_linkNameChanged(LinkProperty*,QVariant)));
    QObject::connect(tree_link.get(), SIGNAL(valueChanged()),
              this, SLOT(on_propertyWidget_valueChanged()));

    // add mapping from treewidget item to link property
    mapLinkTreeItemToProperty(item, tree_link);
    // add mapping from link property to treewidget item
    mapLinkPropertyToTreeItem(tree_link.get(), item);

    link_names_.append(QString::fromStdString(link->name));

    return tree_link;
  }

  void URDFProperty::removeModelLink(QString link_name)
  {
    urdf::LinkSharedPtr link;
    model_->getLink(link_name.toStdString(), link);
    if (link)
    {
      // remove mapping from child link to joint QTreeWidgetItem
      joint_child_to_ctree_.remove(link);

      // remove mapping from link to link QTreeWidgetItem
      link_to_ltree_.remove(link);

      model_->links_.erase(model_->links_.find(link_name.toStdString()));
      link_names_.removeOne(link_name);
    }
  }

  void URDFProperty::removeLinkTreeItem(QTreeWidgetItem *item)
  {
    LinkPropertySharedPtr link_property = getLinkPropertyForTreeItem(item);

    moveTreeChildren(item, item->parent());

    // remove mapping from treewidget item to joint property
    ltree_to_link_property_.remove(item);
    // remove mapping from joint property to treewidget item
    link_property_to_ltree_.remove(link_property.get());

    item->parent()->removeChild(item);
  }

  urdf::JointSharedPtr URDFProperty::addModelJoint(QTreeWidgetItem *parent, QString child_link_name)
  {
    // add joint to urdf model
    QString name = getValidName("joint_", joint_names_);
    urdf::JointSharedPtr new_joint(new urdf::Joint());
    new_joint->name = name.toStdString();

    if (parent == joint_root_)
      new_joint->parent_link_name = model_->getRoot()->name;
    else
      new_joint->parent_link_name = getJointPropertyForTreeItem(parent)->getChildLinkName().toStdString();

    new_joint->child_link_name = child_link_name.toStdString();
    model_->joints_.insert(std::make_pair(name.toStdString(), new_joint));

    // TODO: adding tree items and creating properties is not this methods responsibility
    // first add the tree item
    QTreeWidgetItem* item = addJointTreeItem(parent, new_joint);
    // now add the property
    addJointProperty(item, new_joint);

    emit jointAddition();
    return new_joint;
  }

  QTreeWidgetItem* URDFProperty::addJointTreeItem(QTreeWidgetItem* parent, urdf::JointSharedPtr joint)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);
    item->setText(0, QString::fromStdString(joint->name));

    // add mapping between joint child link and joint QTreeWidgetItem
    urdf::LinkSharedPtr link;
    model_->getLink(joint->child_link_name, link);
    mapLinkToJointTreeItem(link, item);

    return item;
  }

  JointPropertySharedPtr URDFProperty::addJointProperty(QTreeWidgetItem *item, urdf::JointSharedPtr joint)
  {
    JointPropertySharedPtr tree_joint(new JointProperty(joint, link_names_, joint_names_));
    QObject::connect(tree_joint.get(), SIGNAL(jointNameChanged(JointProperty *, const QVariant &)),
              this, SLOT(on_propertyWidget_jointNameChanged(JointProperty*,QVariant)));
    QObject::connect(tree_joint.get(), SIGNAL(parentLinkChanged(JointProperty *, const QVariant &)),
              this, SLOT(on_propertyWidget_jointParentLinkChanged(JointProperty*,QVariant)));
    QObject::connect(tree_joint.get(), SIGNAL(valueChanged()),
              this, SLOT(on_propertyWidget_valueChanged()));

    // add mapping from treewidget item to joint property
    mapJointTreeItemToProperty(item, tree_joint);
    // add mapping from joint property to treewidget item
    mapJointPropertyToTreeItem(tree_joint.get(), item);

    joint_names_.append(QString::fromStdString(joint->name));

    return tree_joint;
  }

  void URDFProperty::removeModelJoint(QString joint_name)
  {
    std::map<std::string, urdf::JointSharedPtr>::iterator it = model_->joints_.find(joint_name.toStdString());
    if (it != model_->joints_.end())
    {
      model_->joints_.erase(it);
      joint_names_.removeOne(joint_name);
    }
  }

  void URDFProperty::removeJointTreeItem(QTreeWidgetItem *item)
  {
    JointPropertySharedPtr joint_property = getJointPropertyForTreeItem(item);
    QString newParentName = joint_property->getParentLinkName();

    // If it is trying to set the parent to a link that does not exits,
    // set the parent to the firts link in the chain. This should only
    // occur if the user delets the firts link
    if (!link_names_.contains(newParentName))
      newParentName = QString::fromStdString(model_->getRoot()->name);

    QTreeWidgetItem *parent = item->parent();

    for (int i=0; i < item->childCount(); i++)
    {
      getJointPropertyForTreeItem(item->child(i))->setParentLinkName(newParentName);
    }

    // remove mapping from treewidget item to joint property
    ctree_to_joint_property_.remove(item);
    // remove mapping from joint property to treewidget item
    joint_property_to_ctree_.remove(joint_property.get());

    parent->removeChild(item);
  }

  QString URDFProperty::getValidName(QString prefix, QList<QString> &current_names)
  {
    int i = 0;
    QString name;
    do
    {
      i+=1;
      name = prefix + QString::number(i);
    } while (current_names.contains(name));
    return name;
  }

  bool URDFProperty::isLink(QTreeWidgetItem *item)
  {
    return ltree_to_link_property_.contains(item);
  }

  bool URDFProperty::isJoint(QTreeWidgetItem *item)
  {
    return ctree_to_joint_property_.contains(item);
  }


  void URDFProperty::mapLinkToLinkTreeItem(urdf::LinkSharedPtr link, QTreeWidgetItem *item)
  {
    if (link_to_ltree_.contains(link))
      ROS_DEBUG("The Link(%s) to LinkTreeItem was remapped.", link->name.c_str());

    link_to_ltree_[link]=item;
  }

  QTreeWidgetItem * URDFProperty::getLinkTreeItemForLink(urdf::LinkSharedPtr link)
  {
    try
    {
      return link_to_ltree_[link];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested Link QTreeWidgetItem for Link(%s) is not mapped.", link->name.c_str());
      throw(e);
    }
  }

  void URDFProperty::mapLinkToJointTreeItem(urdf::LinkSharedPtr link, QTreeWidgetItem *item)
  {
    if (joint_child_to_ctree_.contains(link))
      ROS_DEBUG("The Link(%s) to joint QTreeWidgetItem was remapped.", link->name.c_str());

    joint_child_to_ctree_[link]=item;
  }

  QTreeWidgetItem * URDFProperty::getJointTreeItemForLink(urdf::LinkSharedPtr link)
  {
    try
    {
      return joint_child_to_ctree_[link];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested joint QTreeWidgetItem for Link(%s) is not mapped.", link->name.c_str());
      throw(e);
    }
  }

  void URDFProperty::mapJointTreeItemToProperty(QTreeWidgetItem *item, JointPropertySharedPtr property)
  {
    if (ctree_to_joint_property_.contains(item))
      ROS_DEBUG("The Joint(%s) QTreeWidgetItem to JointPropertySharedPtr was remapped.", item->text(0).toStdString().c_str());

    ctree_to_joint_property_[item]=property;
  }

  JointPropertySharedPtr URDFProperty::getJointPropertyForTreeItem(QTreeWidgetItem *item)
  {
    try
    {
      return ctree_to_joint_property_[item];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested JointPropertySharedPtr for Joint(%s) QTreeWidgetItem(%s) is not mapped.", item->text(0).toStdString().c_str());
      throw(e);
    }
  }

  void URDFProperty::mapJointPropertyToTreeItem(JointProperty *property, QTreeWidgetItem *item)
  {
    if (joint_property_to_ctree_.contains(property))
      ROS_DEBUG("The Joint(%s) JointProperty to QTreeWidgetItem was remapped.", property->getName().toStdString().c_str());

    joint_property_to_ctree_[property]=item;
  }

  QTreeWidgetItem * URDFProperty::getJointTreeItemForProperty(JointProperty *property)
  {
    try
    {
      return joint_property_to_ctree_[property];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested Joint QTreeWidgetItem for link(%s) JointProperty(%s) is not mapped.", property->getName().toStdString().c_str());
      throw(e);
    }
  }

  void URDFProperty::mapLinkTreeItemToProperty(QTreeWidgetItem *item, LinkPropertySharedPtr property)
  {
    if (ltree_to_link_property_.contains(item))
      ROS_DEBUG("The Link(%s) QTreeWidgetItem to LinkPropertySharedPtr was remapped.", item->text(0).toStdString().c_str());

    ltree_to_link_property_[item]=property;
  }

  LinkPropertySharedPtr URDFProperty::getLinkPropertyForTreeItem(QTreeWidgetItem *item)
  {
    try
    {
      return ltree_to_link_property_[item];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested LinkPropertySharedPtr for link(%s) QTreeWidgetItem(%s) is not mapped.", item->text(0).toStdString().c_str());
      throw(e);
    }
  }

  void URDFProperty::mapLinkPropertyToTreeItem(LinkProperty *property, QTreeWidgetItem *item)
  {
    if (link_property_to_ltree_.contains(property))
      ROS_DEBUG("The Link(%s) LinkProperty to link QTreeWidgetItem was remapped.", property->getName().toStdString().c_str());

    link_property_to_ltree_[property]=item;
  }

  QTreeWidgetItem * URDFProperty::getLinkTreeItemForProperty(LinkProperty *property)
  {
    try
    {
      return link_property_to_ltree_[property];
    }
    catch (std::exception& e)
    {
      ROS_ERROR("The requested link QTreeWidgetItem for link(%s) QTreeWidgetItem(%s) is not mapped.", property->getName().toStdString().c_str());
      throw(e);
    }
  }

  void URDFProperty::moveTreeChildren(QTreeWidgetItem *parent, QTreeWidgetItem *new_parent)
  {
    for (int i=0; i < parent->childCount(); i++)
      new_parent->addChild(parent->takeChild(i));
  }

  void URDFProperty::on_treeWidget_customContextMenuRequested(const QPoint &pos)
  {
      if (tree_widget_->selectedItems().isEmpty())
        return;

      QTreeWidgetItem *sel = tree_widget_->selectedItems()[0];

      QMenu *menu = new QMenu(tree_widget_);

      if ((sel == link_root_ && link_names_.isEmpty()) || isLink(sel))
      {
        menu->addAction(ACTION_ADD_TEXT);
        menu->addAction(ACTION_REMOVE_TEXT)->setEnabled(isLink(sel));
        menu->addSeparator();
        menu->addAction(ACTION_EXPANDALL_TEXT);
        menu->addAction(ACTION_COLLAPSEALL_TEXT);
      }
      else if (sel == joint_root_ || isJoint(sel) || (sel == link_root_ && !link_names_.isEmpty()))
      {
        menu->addAction(ACTION_EXPANDALL_TEXT);
        menu->addAction(ACTION_COLLAPSEALL_TEXT);
      }

      QAction *selected_item = menu->exec(tree_widget_->mapToGlobal(pos));
      if (selected_item)
      {
        if (selected_item->text() == ACTION_ADD_TEXT)
        {
          // we can only add to the link root item or to other links
          if (sel == link_root_ || isLink(sel))
          {
            urdf::LinkSharedPtr new_link = addModelLink(sel);
            urdf::LinkSharedPtr sel_link;
            model_->getLink(sel->text(0).toStdString(), sel_link);

            if (link_names_.count() > 2 && sel->parent() != link_root_)
            {
              QTreeWidgetItem *parent = getJointTreeItemForLink(sel_link);
              addModelJoint(parent, QString::fromStdString(new_link->name));
            }
            else if (link_names_.count() == 2 || sel->parent() == link_root_)
            {
              addModelJoint(joint_root_, QString::fromStdString(new_link->name));
            }
          }
        }
        else if (selected_item->text() == ACTION_REMOVE_TEXT)
        {
          if (isLink(sel))
          {
            // Also remove joint associated to removed link.
            urdf::LinkSharedPtr sel_link, child_link;
            model_->getLink(sel->text(0).toStdString(), sel_link);
            QTreeWidgetItem *link = getLinkTreeItemForLink(sel_link);

            // you can only remove the base link if it is the only link or has one child to prevent a malformed urdf.
            if ((link->parent() == link_root_ && link->childCount() <= 1) || link->parent() != link_root_)
            {
              QTreeWidgetItem *joint;
              if (link->parent() == link_root_)
              {
                if (link->childCount() == 1)
                {
                  model_->getLink(link->child(0)->text(0).toStdString(), child_link);
                  joint = getJointTreeItemForLink(child_link);

                  // remove mapping from child link to joint QTreeWidgetItem
                  joint_child_to_ctree_.remove(child_link);
                }
                else
                {
                  joint = NULL;
                }
              }
              else
              {
                joint = getJointTreeItemForLink(sel_link);
              }

              // order matters due to the removal of mappings
              removeLinkTreeItem(sel);
              removeModelLink(sel->text(0));

              // This is in case there is only one link and the user removes it. If there
              // is only one link there are no joints.
              if (joint)
              {
                // order matters due to the removal of mappings
                removeJointTreeItem(joint);
                removeModelJoint(joint->text(0));
              }

              // during the removal process the urdf is broken then fixed, so an update is required.
              on_propertyWidget_valueChanged();
            }
            else
            {
              // need to throw warning to user
            }
          }
        }
        else if (selected_item->text() == ACTION_EXPANDALL_TEXT)
        {
          sel->setExpanded(true);
          setAllChildrenExpandedValue(sel, true);
        }
        else if (selected_item->text() == ACTION_COLLAPSEALL_TEXT)
        {
          sel->setExpanded(false);
          setAllChildrenExpandedValue(sel, false);
        }
      }

      delete menu;
  }

  void URDFProperty::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
  {
    if (isLink(item))
    {
      getLinkPropertyForTreeItem(item)->loadProperty(property_editor_);
    }
    else if (isJoint(item))
    {
      //need to pass a list of available child links
      getJointPropertyForTreeItem(item)->loadProperty(property_editor_);
    }
    else
    {
      property_editor_->clear();
    }
  }

  void URDFProperty::on_treeWidget_itemDragDrop(QTreeWidgetItem *drag, QTreeWidgetItem *drop)
  {
    if (isLink(drag) && isLink(drop))
    {
      urdf::LinkSharedPtr link;
      model_->getLink(drag->text(0).toStdString(), link);
      QTreeWidgetItem *joint = getJointTreeItemForLink(link);
      JointPropertySharedPtr joint_property = getJointPropertyForTreeItem(joint);
      joint_property->setParentLinkName(drop->text(0));
    }
    else if (isJoint(drag) && isJoint(drop))
    {
      JointPropertySharedPtr drag_joint_property = getJointPropertyForTreeItem(drag);
      JointPropertySharedPtr drop_joint_property = getJointPropertyForTreeItem(drop);
      drag_joint_property->setParentLinkName(drop_joint_property->getChildLinkName());
    }
    else if (isJoint(drag) && drop == joint_root_)
    {
      JointPropertySharedPtr drag_joint_property = getJointPropertyForTreeItem(drag);
      drag_joint_property->setParentLinkName(link_root_->child(0)->text(0));
    }
  }

  void URDFProperty::on_propertyWidget_customContextMenuRequested(const QPoint &pos)
  {
    QtBrowserItem *selb = property_editor_->currentItem();
    QTreeWidgetItem *selt = tree_widget_->selectedItems()[0];

    
    if (!selb)
      return;

    if (ctree_to_joint_property_.contains(selt))
    {
       qDebug() << QString("The member ctree_to_joint_property_  contains the link %1").arg(selt->text(0));
       
       JointPropertySharedPtr activeJoint = getJointPropertyForTreeItem(selt);
       QMenu menu(property_editor_.get());
       
       
      // user right-clicked a 'Name' property entry: show 'Origin', Axis, Calibration, Dynamics, Limit, Mimic
      // and 'Safety Controller' options
      if (selb->property()->propertyName() == PROPERTY_NAME_TEXT && !selb->parent())
      {
        QAction *origin_action = menu.addAction(PROPERTY_ORIGIN_TEXT);
        QAction *axis_action = menu.addAction(PROPERTY_AXIS_TEXT);
        QAction *calibration_action = menu.addAction(PROPERTY_CALIBRATION_TEXT);  
        QAction *dynamics_action = menu.addAction(PROPERTY_DYNAMICS_TEXT);
        QAction *limit_action = menu.addAction(PROPERTY_LIMIT_TEXT);
        QAction *mimic_action = menu.addAction(PROPERTY_MIMIC_TEXT);
        QAction *safety_action = menu.addAction(PROPERTY_SAFETY_TEXT);
  
        // if this joint already has an 'origin' element, don't allow user to
        // add another
        if (activeJoint->hasOriginProperty())
        {
          origin_action->setDisabled(true);
        }
        
        // if this joint already has a 'axis element, don't allow user to
        // add another
        if (activeJoint->hasAxisProperty())
        {
          axis_action->setDisabled(true);
        }
        
         // if this joint already has a 'limit element, don't allow user to
        // add another
        if (activeJoint->hasLimitsProperty())
        {
          limit_action->setDisabled(true);
        }
        
        // if this joint already has a 'calibration element, don't allow user to
        // add another
        if (activeJoint->hasCalibrationProperty())
        {
          calibration_action->setDisabled(true);
        }
        
         // if this joint already has a 'dynamics element, don't allow user to
        // add another
        if (activeJoint->hasDynamicsProperty())
        {
          dynamics_action->setDisabled(true);
        }
  
        
        
         // if this joint already has a 'mimic element, don't allow user to
        // add another
        if (activeJoint->hasMimicProperty())
        {
          mimic_action->setDisabled(true);
        }
        
         // if this joint already has a 'safety element, don't allow user to
        // add another
        if (activeJoint->hasSafetyProperty())
        {
          safety_action->setDisabled(true);
        }
        
  
        QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
        // don't do anything if user didn't select something
        if (selected_item == NULL)
          return;
  
        if (selected_item == origin_action)
        {
          activeJoint->createOriginProperty();
          activeJoint->loadProperty(property_editor_);
        }
        else if (selected_item == axis_action)
        {
          activeJoint->createAxisProperty();
          activeJoint->loadProperty(property_editor_);
        }
        else if (selected_item == limit_action)
        {
          activeJoint->createLimitsProperty();
          activeJoint->loadProperty(property_editor_);
        }
        else if (selected_item == calibration_action)
        {
          activeJoint->createCalibrationProperty();
          activeJoint->loadProperty(property_editor_);
        }
        else if (selected_item == dynamics_action)
        {
          activeJoint->createDynamicsProperty();
          activeJoint->loadProperty(property_editor_);
        }
         else if (selected_item == mimic_action)
        {
          activeJoint->createMimicProperty();
          activeJoint->loadProperty(property_editor_);
        }
         else if (selected_item == safety_action)
        {
          activeJoint->createSafetyProperty();
          activeJoint->loadProperty(property_editor_);
        }
        else
        {
          // should never happen
          qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
          assert (1==0);
        }
      }
       
      return;
    }//joint
    
    if (ltree_to_link_property_.contains(selt))
    {
     
     
      LinkPropertySharedPtr activeLink = getLinkPropertyForTreeItem(selt);
      QMenu menu(property_editor_.get());
  
      // user right-clicked a 'Name' property entry: show 'Inertial', 'Visual'
      // and 'Collision' options
      if (selb->property()->propertyName() == PROPERTY_NAME_TEXT && !selb->parent())
      {
        QAction *inertial_action = menu.addAction(PROPERTY_INERTIAL_TEXT);
        QAction *visual_action = menu.addAction(PROPERTY_VISUAL_TEXT);
        QAction *collision_action = menu.addAction(PROPERTY_COLLISION_TEXT);
  
        // if this link already has an 'inertial' element, don't allow user to
        // add another
        if (activeLink->hasInertialProperty())
        {
          inertial_action->setDisabled(true);
        }
        
        // if this link already has a 'visual element, don't allow user to
        // add another
        if (activeLink->hasVisualProperty())
        {
          visual_action->setDisabled(true);
        }
        
        // if this link already has a 'collision element, don't allow user to
        // add another
        if (activeLink->hasCollisionProperty())
        {
          collision_action->setDisabled(true);
        }
  
  
        QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
        // don't do anything if user didn't select something
        if (selected_item == NULL)
          return;
  
        if (selected_item == inertial_action)
        {
          activeLink->createInertialProperty();
          activeLink->loadProperty(property_editor_);
        }
        else if (selected_item == visual_action)
        {
          activeLink->createVisualProperty();
          activeLink->loadProperty(property_editor_);
        }
        else if (selected_item == collision_action)
        {
          activeLink->createCollisionProperty();
          activeLink->loadProperty(property_editor_);
        }
        else
        {
          // should never happen
          qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
          assert (1==0);
        }
      }
  
      // user right-clicked an 'Interial' property entry: show 'Origin' option
      // only.
      if (selb->property()->propertyName() == PROPERTY_INERTIAL_TEXT)
      {
        LinkInertialPropertySharedPtr activeInertia = activeLink->getInertialProperty();
        QAction *origin = menu.addAction(PROPERTY_ORIGIN_TEXT);
  
        // if this link already has an 'origin' element, don't allow user to
        // add another
        if (activeInertia->hasOriginProperty())
        {
          origin->setDisabled(true);
        }
  
        QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
        // don't do anything if user didn't select something
        if (selected_item == NULL)
          return;
  
        if (selected_item == origin)
        {
          activeInertia->createOriginProperty();
          activeInertia->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else
        {
          // should never happen
          qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
          assert (1==0);
        }
      }
      //   user right-clicked on 'Visual' property entry: show  option
      // only.
      else if (selb->property()->propertyName() == PROPERTY_VISUAL_TEXT)
      {
        LinkVisualPropertySharedPtr activeVisual = activeLink->getVisualProperty();
        QAction *origin = menu.addAction(PROPERTY_ORIGIN_TEXT);
        QAction *geometry = menu.addAction(PROPERTY_GEOMETRY_TEXT);
        QAction *material = menu.addAction(PROPERTY_MATERIAL_TEXT);
        // if this link already has an 'origin' element, don't allow user to
        // add another
        if (activeVisual->hasOriginProperty())
        {
          origin->setDisabled(true);
        }
        if (activeVisual->hasGeometryProperty())
        {
          geometry->setDisabled(true);
        }
        if (activeVisual->hasMaterialProperty())
        {
          material->setDisabled(true);
        }
        
  
        QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
        // don't do anything if user didn't select something
        if (selected_item == NULL)
          return;
  
        if (selected_item == origin)
        {
          activeVisual->createOriginProperty();
          activeVisual->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else if (selected_item == geometry)
        {
          activeVisual->createGeometryProperty();
          activeVisual->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else if (selected_item == material)
        {
          activeVisual->createMaterialProperty();
          activeVisual->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else
        {
          // should never happen
          qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
          assert (1==0);
        }
      }
        // user right-clicked on 'Collsion' property entry: show  option
      // only.
      else if (selb->property()->propertyName() == PROPERTY_COLLISION_TEXT)
      {
        LinkCollisionPropertySharedPtr activeCollision = activeLink->getCollisionProperty();
        QAction *origin = menu.addAction(PROPERTY_ORIGIN_TEXT);
        QAction *geometry = menu.addAction(PROPERTY_GEOMETRY_TEXT);
        // if this link already has an 'origin' element, don't allow user to
        // add another
        if (activeCollision->hasOriginProperty())
        {
          origin->setDisabled(true);
        }
        
        if (activeCollision->hasGeometryProperty())
        {
          geometry->setDisabled(true);
        }
        
        
  
        QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
        // don't do anything if user didn't select something
        if (selected_item == NULL)
          return;
  
        if (selected_item == origin)
        {
          activeCollision->createOriginProperty();
          activeCollision->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else if (selected_item == geometry)
        {
          activeCollision->createGeometryProperty();
          activeCollision->loadData();
          activeLink->loadProperty(property_editor_);
        }
        else
        {
          // should never happen
          qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
          assert (1==0);
        }
      }
     return;
    }//Link

    
  }

  void URDFProperty::on_propertyWidget_linkNameChanged(LinkProperty *property, const QVariant &val)
  {
    QString orig_name = getLinkTreeItemForProperty(property)->text(0);
    int idx = link_names_.indexOf(orig_name);
    link_names_.replace(idx, val.toString());
    getLinkTreeItemForProperty(property)->setText(0, val.toString());
  }

  void URDFProperty::on_propertyWidget_jointNameChanged(JointProperty *property, const QVariant &val)
  {
    QString orig_name = getJointTreeItemForProperty(property)->text(0);
    int idx = joint_names_.indexOf(orig_name);
    joint_names_.replace(idx, val.toString());
    getJointTreeItemForProperty(property)->setText(0, val.toString());
  }

  void URDFProperty::on_propertyWidget_jointParentLinkChanged(JointProperty *property, const QVariant &val)
  {
    urdf::LinkSharedPtr parent_link, child_link;
    model_->getLink(property->getParentLinkName().toStdString(), parent_link);
    model_->getLink(property->getChildLinkName().toStdString(), child_link);

    // move joint QTreeWidgetItem
    QTreeWidgetItem *newParent = getJointTreeItemForLink(parent_link);

    // if a parent joint does not exist (in the case of the first link),
    // set it to the root joint QTreeWidgetItem
    if (!newParent)
      newParent = joint_root_;

    QTreeWidgetItem *move = getJointTreeItemForProperty(property);
    QTreeWidgetItem *take = move->parent()->takeChild(move->parent()->indexOfChild(move));
    newParent->addChild(take);

    // move link QTreeWidgetItem
    newParent = getLinkTreeItemForLink(parent_link);
    move = getLinkTreeItemForLink(child_link);
    take = move->parent()->takeChild(move->parent()->indexOfChild(move));
    newParent->addChild(take);
  }

  void URDFProperty::on_propertyWidget_valueChanged()
  {
    rviz_widget_->loadRobot(model_);
    unsavedChanges = true;
  }

  void URDFProperty::on_unsavedChanges()
  {
    unsavedChanges = true;
  }

}
