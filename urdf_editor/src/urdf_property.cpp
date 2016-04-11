#include "urdf_editor/urdf_property.h"
#include <QVBoxLayout>
#include <QMessageBox>

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

namespace urdf_editor
{
  URDFProperty::URDFProperty(QTreeWidget *tree_widget, QWidget *browser_parent, QWidget *rviz_parent)
  {
    tree_widget_ = tree_widget;
    browser_parent_ = browser_parent;
    model_.reset(new urdf::ModelInterface());

    root_ = new QTreeWidgetItem(tree_widget_);
    root_->setText(0, "RobotModel");
    root_->setExpanded(true);
    tree_widget_->addTopLevelItem(root_);

    link_root_ = new QTreeWidgetItem();
    link_root_->setText(0, "Links");
    root_->addChild(link_root_);

    joint_root_ = new QTreeWidgetItem();
    joint_root_->setText(0,"Joints");
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

    connect(property_editor_.get(), SIGNAL(customContextMenuRequested(QPoint)),
              this, SLOT(on_propertyWidget_customContextMenuRequested(QPoint)));

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
    joint_child_to_ctree_.clear();
    ctree_to_joint_property_.clear();
    joint_property_to_ctree_.clear();
    ltree_to_link_property_.clear();
    link_property_to_ltree_.clear();
    link_names_.clear();
    joint_names_.clear();
  }

  bool URDFProperty::loadURDF(QString file_path)
  {
    model_ = urdf::parseURDFFile(file_path.toStdString());
    if (model_)
      if (buildTree())
      {
        rviz_widget_->loadRobot(model_);
        return true;
      }
      else
      {
        return false;
      }
    else
    {
      return false;
    }
  }

  bool URDFProperty::saveURDF(QString file_path)
  {
    TiXmlDocument* doc = urdf::exportURDF(model_);
    TiXmlDeclaration decl("1.0", "", "");
    doc->InsertBeforeChild(doc->RootElement(), decl);
    bool savedCorrectly = false;
    savedCorrectly = doc->SaveFile(file_path.toStdString());

    QMessageBox msgBox;
    if (savedCorrectly)
    {
      msgBox.setWindowTitle("Success");
      msgBox.setText("The file was saved.");
    }
    else
    {
      msgBox.setWindowTitle("FAILURE");
      msgBox.setText("An error occurred during saving.");
    }
    msgBox.exec();

    return true;
  }

  bool URDFProperty::buildTree()
  {
    std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator link_it;
    for (link_it = model_->links_.begin(); link_it != model_->links_.end(); ++link_it)
    {
      addLinkProperty(link_it->second);
    }

    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator joint_it;
    std::string name;
    for (joint_it = model_->joints_.begin(); joint_it != model_->joints_.end(); ++joint_it)
    {
      name = joint_it->second->parent_link_name;
      if (name == model_->root_link_->name)
      {
        addJointProperty(joint_root_, joint_it->second);
      }
      else
      {
        addJointProperty(joint_child_to_ctree_[model_->links_.find(name)->second], joint_it->second);
      }

    }

    return true;
  }

  void URDFProperty::addLink()
  {
    QString name = getValidName("link_", link_names_);
    boost::shared_ptr<urdf::Link> new_link(new urdf::Link());
    new_link->name = name.toStdString();
    model_->links_.insert(std::make_pair(name.toStdString(), new_link));

    addLinkProperty(new_link);
  }

  void URDFProperty::addLinkProperty(boost::shared_ptr<urdf::Link> link)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem(link_root_);
    item->setText(0, QString::fromStdString(link->name));
    root_->addChild(item);

    LinkPropertyPtr tree_link(new LinkProperty(link));
    QObject::connect(tree_link.get(), SIGNAL(linkNameChanged(LinkProperty *, const QVariant &)),
              this, SLOT(on_propertyWidget_linkNameChanged(LinkProperty*,QVariant)));
    QObject::connect(tree_link.get(), SIGNAL(valueChanged()),
              this, SLOT(on_propertyWidget_valueChanged()));

    ltree_to_link_property_[item] = tree_link;
    link_property_to_ltree_[tree_link.get()] = item;

    link_names_.append(QString::fromStdString(link->name));
  }

  void URDFProperty::addJoint(QTreeWidgetItem *parent)
  {
    QString name = getValidName("joint_", joint_names_);
    boost::shared_ptr<urdf::Joint> new_joint(new urdf::Joint());
    new_joint->name = name.toStdString();
    model_->joints_.insert(std::make_pair(name.toStdString(), new_joint));

    addJointProperty(parent, new_joint);
  }

  void URDFProperty::addJointProperty(QTreeWidgetItem *parent, boost::shared_ptr<urdf::Joint> joint)
  {
    QString name = QString::fromStdString(joint->name);

    QTreeWidgetItem *item = new QTreeWidgetItem(parent);
    item->setText(0, name);
    root_->addChild(item);

    joint_child_to_ctree_[model_->links_.find(joint->child_link_name)->second] = item;

    JointPropertyPtr tree_joint(new JointProperty(joint, link_names_, joint_names_));
    QObject::connect(tree_joint.get(), SIGNAL(jointNameChanged(JointProperty *, const QVariant &)),
              this, SLOT(on_propertyWidget_jointNameChanged(JointProperty*,QVariant)));
    QObject::connect(tree_joint.get(), SIGNAL(valueChanged()),
              this, SLOT(on_propertyWidget_valueChanged()));

    ctree_to_joint_property_[item] = tree_joint;
    joint_property_to_ctree_[tree_joint.get()] = item;

    joint_names_.append(name);
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

  void URDFProperty::on_treeWidget_customContextMenuRequested(const QPoint &pos)
  {

      QTreeWidgetItem *sel = tree_widget_->selectedItems()[0];

      QMenu *menu = new QMenu(tree_widget_);
      menu->addAction("Add");
      menu->addAction("Remove");
      QAction *selected_item = menu->exec(tree_widget_->mapToGlobal(pos));
      if (selected_item)
      {
        if (selected_item->text() == "Add")
        {
          // we can only add to the link root item or to other links
          if (sel == link_root_ || isLink(sel))
          {
            addLink();
          }
          // or to the joint root item or to other links
          else if (sel == joint_root_ || isJoint(sel))
          {
            addJoint(sel);
          }
        }
        else
        {
          if (isLink(sel))
          {
            link_names_.removeOne(sel->text(0));
            link_root_->removeChild(sel);
          }
          else if (isJoint(sel))
          {
            joint_names_.removeOne(sel->text(0));
            sel->parent()->removeChild(sel);
          }
        }
      }

      delete menu;
  }

  void URDFProperty::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
  {
    if (isLink(item))
    {
      ltree_to_link_property_[item]->loadProperty(property_editor_);
    }
    else if (isJoint(item))
    {
      //need to pass a list of available child links
      ctree_to_joint_property_[item]->loadProperty(property_editor_);
    }
    else
    {
      property_editor_->clear();
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
       
       JointPropertyPtr activeJoint = ctree_to_joint_property_[selt];
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
     
     
      LinkPropertyPtr activeLink = ltree_to_link_property_[selt];
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
        LinkInertialPropertyPtr activeInertia = activeLink->getInertialProperty();
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
        LinkVisualPropertyPtr activeVisual = activeLink->getVisualProperty();
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
        LinkCollisionPropertyPtr activeCollision = activeLink->getCollisionProperty();
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
    QString orig_name = link_property_to_ltree_[property]->text(0);
    int idx = link_names_.indexOf(orig_name);
    link_names_.replace(idx, val.toString());
    link_property_to_ltree_[property]->setText(0, val.toString());
  }

  void URDFProperty::on_propertyWidget_jointNameChanged(JointProperty *property, const QVariant &val)
  {
    QString orig_name = joint_property_to_ctree_[property]->text(0);
    int idx = joint_names_.indexOf(orig_name);
    joint_names_.replace(idx, val.toString());
    joint_property_to_ctree_[property]->setText(0, val.toString());
  }

  void URDFProperty::on_propertyWidget_valueChanged()
  {
    rviz_widget_->loadRobot(model_);
  }

}
