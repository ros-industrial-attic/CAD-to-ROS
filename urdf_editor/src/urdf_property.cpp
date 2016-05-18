
#include <QVBoxLayout>

#include <urdf_parser/urdf_parser.h>

#include <urdf_editor/urdf_property.h>
#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/link_collision_property.h>
#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/link_inertial_property.h>
#include <urdf_editor/link_new_material_property.h>
#include <urdf_editor/link_visual_property.h>
#include <urdf_editor/link_property.h>
#include <urdf_editor/urdf_property_tree_link_item.h>
#include <urdf_editor/joint_property.h>
#include <urdf_editor/urdf_transforms.h>
#include <qmessagebox.h>

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
  URDFProperty::URDFProperty(URDFPropertyTree *tree_widget, QWidget *browser_parent, QWidget *rviz_parent)
  {
    tree_widget_ = tree_widget;
    browser_parent_ = browser_parent;

    property_editor_.reset(new QtTreePropertyBrowser());
    property_editor_->setContextMenuPolicy(Qt::CustomContextMenu);
    property_editor_->setPropertiesWithoutValueMarked(false);
    property_editor_->setRootIsDecorated(true);
    property_editor_->setResizeMode(QtTreePropertyBrowser::ResizeToContents);

    QVBoxLayout *vlayout = new QVBoxLayout(browser_parent_);
    vlayout->setMargin(0);
    vlayout->addWidget(property_editor_.get());

    rviz_widget_ = new urdf_editor::MyRviz(rviz_parent);

    tf_transformer_.reset(new URDFTransformer());

    connect(tree_widget_, SIGNAL(itemClicked(QTreeWidgetItem*,int)),
            this, SLOT(on_treeWidget_itemClicked(QTreeWidgetItem*,int)));

    connect(tree_widget_, SIGNAL(jointParentLinkChanged(JointProperty*,QString,QString)),
            this, SLOT(on_propertyWidget_jointParentLinkChanged(JointProperty*,QString,QString)));
    connect(tree_widget_, SIGNAL(jointOriginChanged(JointProperty*)),
            this, SLOT(on_propertyWidget_jointOriginChanged(JointProperty*)));
    connect(tree_widget_, SIGNAL(jointAxisChanged(JointProperty*)),
            this, SLOT(on_propertyWidget_jointAxisChanged(JointProperty*)));

    connect(tree_widget_, SIGNAL(linkValueChanged(LinkProperty*)),
            this, SLOT(on_propertyWidget_linkValueChanged(LinkProperty*)));

    connect(property_editor_.get(), SIGNAL(customContextMenuRequested(QPoint)),
              this, SLOT(on_propertyWidget_customContextMenuRequested(QPoint)));

    connect(tree_widget_, SIGNAL(jointAddition(JointProperty*)), SLOT(on_unsavedChanges(JointProperty*)));

    connect(tree_widget_, SIGNAL(jointDeletion()), SLOT(on_unsavedChanges()));

    connect(tree_widget_, SIGNAL(linkAddition(LinkProperty*)), SLOT(on_unsavedChanges()));

    connect(tree_widget_, SIGNAL(linkDeletion()), SLOT(on_unsavedChanges()));

    // No changes to be saved, yet
    unsavedChanges = false;
  }

  URDFProperty::~URDFProperty()
  {
  }

  bool URDFProperty::clear()
  {
    tree_widget_->clear();
    rviz_widget_->clear();
    property_editor_->clear();
    tf_transformer_->clear();
    unsavedChanges = false;
  }

  bool URDFProperty::loadURDF(QString file_path)
  {
    // TODO: refactor to single-exit method, or use exceptions and
    //       make sure to set 'loading_ = false'.
    loading_ = true;

    urdf::ModelInterfaceSharedPtr model;

    if (file_path.toStdString().find(".xacro") != std::string::npos)
    {
      std::string urdf_string;
      std::string cmd("rosrun xacro xacro.py ");
      cmd += file_path.toStdString();
      ROS_INFO( "Running '%s'...", cmd.c_str() );

      FILE* pipe = popen(cmd.c_str(), "r");
      if (!pipe)
      {
        ROS_ERROR("Error Loading Files: XACRO file or parser not found " );
        loading_ = false;
        return false;
      }
      char buffer[128] = {0};
      while (!feof(pipe))
      {
        if (fgets(buffer, sizeof(buffer), pipe) != NULL)
          urdf_string += buffer;
      }
      pclose(pipe);
      
      if (urdf_string.empty())
      {
        ROS_ERROR("Error Loading Files: Unable to parse XACRO file " );
        loading_ = false;
        return false;
      }
      model = urdf::parseURDF(urdf_string);
    }
    else
    {
      model = urdf::parseURDFFile(file_path.toStdString());
    }

    if (!tree_widget_->loadRobotModel(model))
    {
      loading_ = false;
      return false;
    }

    if (!rviz_widget_->loadRobot(model))
    {
      loading_ = false;
      return false;
    }

    rviz_widget_->updateBaseLink(model->getRoot()->name);

    loading_ = false;

    return true;
  }

  bool URDFProperty::saveURDF(QString file_path)
  {
    TiXmlDocument* doc = urdf::exportURDF(*tree_widget_->getRobotModel());
    TiXmlDeclaration decl("1.0", "", "");
    doc->InsertBeforeChild(doc->RootElement(), decl);
    bool savedCorrectly = false;
    savedCorrectly = doc->SaveFile(file_path.toStdString());

    return savedCorrectly;
  }

  bool URDFProperty::redrawRobotModel()
  {
    return rviz_widget_->loadRobot(tree_widget_->getRobotModel());
  }

  void URDFProperty::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
  {
    if (tree_widget_->isLink(item))
    {
      tree_widget_->asLinkTreeItem(item)->loadProperty(property_editor_);
    }
    else if (tree_widget_->isJoint(item))
    {
      //need to pass a list of available child links
      tree_widget_->asJointTreeItem(item)->loadProperty(property_editor_);
    }
    else
    {
      property_editor_->clear();
    }
  }

  void URDFProperty::on_propertyWidget_customContextMenuRequested(const QPoint &pos)
  {
    QtBrowserItem *selb = property_editor_->currentItem();
    QTreeWidgetItem *selt = tree_widget_->getSelectedItem();

    
    if (!selb)
      return;

    if (tree_widget_->isJoint(selt))
    {
       qDebug() << QString("The member ctree_to_joint_property_  contains the link %1").arg(selt->text(0));
       
       JointPropertySharedPtr activeJoint = tree_widget_->asJointTreeItem(selt)->getProperty();
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
    
    if (tree_widget_->isLink(selt))
    {
      LinkPropertySharedPtr activeLink = tree_widget_->asLinkTreeItem(selt)->getProperty();
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
        // user right-clicked on 'Collision' property entry: show  option
      // only.
      else if (selb->property()->propertyName() == PROPERTY_COLLISION_TEXT)
      {
        LinkCollisionPropertySharedPtr activeCollision = activeLink->getCollisionProperty();
        QAction *origin = menu.addAction(PROPERTY_ORIGIN_TEXT);
        QAction *geometry = menu.addAction(PROPERTY_GEOMETRY_TEXT);
        // if this link already has an 'origin' element, don't allow user to
        // add another
        if(activeCollision->hasOriginProperty())
        {
          origin->setDisabled(true);
        }
        
        if(activeCollision->hasGeometryProperty())
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

      else if (selb->property()->propertyName() == PROPERTY_GEOMETRY_TEXT)
      {
        if(selb->parent()->property()->propertyName().compare("Collision")==0)
        {
          LinkCollisionPropertySharedPtr activeCollision = activeLink->getCollisionProperty();
          LinkGeometryPropertySharedPtr geometry =  activeCollision->getGeometryProperty();

          QAction *generate_chull = menu.addAction(QString("Generate Convex"));

          QAction *selected_item = menu.exec(property_editor_->mapToGlobal(pos));
          // don't do anything if user didn't select something
          if (selected_item == NULL)
            return;

          if(selected_item == generate_chull)
          {
            std::string message;
            if(!geometry->generateConvexMesh(message))
            {
              QMessageBox::warning(NULL,"Convex Hull Failure",QString::fromStdString(message),QMessageBox::Ok);
            }
            else
            {
              QMessageBox msgBox;
              msgBox.setText("Convex Hull Generation Succeeded");
              msgBox.exec();
            }

          }
          else
          {
            // should never happen
            qDebug() << QString("The selected right click member %1 is not being handled!").arg(selected_item->text());
            assert (1==0);
          }
        }

      }

     return;
    }//Link 
  }

  void URDFProperty::on_propertyWidget_linkValueChanged(LinkProperty *property)
  {
    Q_UNUSED(property)
    redrawRobotModel();
    unsavedChanges = true;
  }


  void URDFProperty::on_unsavedChanges(JointProperty *property)
  {
    if (!loading_)
      unsavedChanges = true;

    tf_transformer_->updateLink(property);
  }

  void URDFProperty::on_unsavedChanges()
  {
    if (!loading_)
      unsavedChanges = true;
  }

  void URDFProperty::on_propertyWidget_jointParentLinkChanged(JointProperty *property, QString current_name, QString new_name)
  {
    tf_transformer_->updateLink(current_name.toStdString(), property->getChild().toStdString(), new_name.toStdString(), property->getChild().toStdString());

    if (!redrawRobotModel())
    {
        ROS_WARN("Model is invalid, skipping TF transformer and fixed frame update.");
        return;
    }

    // make sure we only update if the model has an actual root link
    if (tree_widget_->getRobotModel()->getRoot() == NULL)
    {
        ROS_WARN("Model has no root link, skipping TF transformer and fixed frame update.");
        return;
    }

    tf_transformer_->updateLink(property);

    // update Rviz base link in the event that root has changed
    rviz_widget_->updateBaseLink(tree_widget_->getRobotModel()->getRoot()->name);
  }

  void URDFProperty::on_propertyWidget_jointOriginChanged(JointProperty *property)
  {
    redrawRobotModel();
    tf_transformer_->updateLink(property);
  }

  void URDFProperty::on_propertyWidget_jointAxisChanged(JointProperty *property)
  {
    redrawRobotModel();
    tf_transformer_->updateLink(property);
  }

}
