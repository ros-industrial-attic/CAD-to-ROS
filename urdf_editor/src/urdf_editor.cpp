#include "include/urdf_editor/urdf_editor.h"
#include "ui_industrial_robot_builder.h"
#include <sstream>



URDFEditor::URDFEditor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::URDFEditor)
{
  ui->setupUi(this);

  root_ = new QTreeWidgetItem(ui->robotTreeWidget);
  root_->setText(0, "RobotModel");
  ui->robotTreeWidget->addTopLevelItem(root_);

  links_ = new QTreeWidgetItem();
  links_->setText(0,"Links");
  root_->addChild(links_);

  joints_ = new QTreeWidgetItem();
  joints_->setText(0,"Chain");
  root_->addChild(joints_);

  model_ = urdf::parseURDFFile("/home/larmstrong/catkin_iiwa_ws/src/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf");

  property_editor_ = new QtTreePropertyBrowser();
  property_editor_->setPropertiesWithoutValueMarked(false);
  property_editor_->setRootIsDecorated(true);
  property_editor_->setResizeMode(QtTreePropertyBrowser::ResizeToContents);

  QVBoxLayout *vlayout = new QVBoxLayout(ui->propertyBrowserContainer);
  vlayout->setMargin(0);
  vlayout->addWidget(property_editor_);

  variant_factory_ = new QtVariantEditorFactory();

  joint_tree_ = new urdf_editor::JointTreeProperty();
  link_tree_ = new urdf_editor::LinkTreeProperty();

}

URDFEditor::~URDFEditor()
{
  delete ui;
}

void URDFEditor::addLink()
{
  QTreeWidgetItem *itm = new QTreeWidgetItem(links_);
  QString name = getValidName("link_", link_names);
  itm->setText(0, name);
  root_->addChild(itm);

  link_names.append(name);
}

void URDFEditor::addJoint(QTreeWidgetItem *parent)
{
  QTreeWidgetItem *itm = new QTreeWidgetItem(parent);
  QString name = getValidName("joint_", joint_names);
  itm->setText(0, name);
  root_->addChild(itm);

  joint_names.append(name);
}

bool URDFEditor::isJoint(QTreeWidgetItem *itm)
{
  QTreeWidgetItem *parent = itm;
  while (parent->parent())
  {
    if (parent->parent() == joints_)
    {
      return true;
    }
    else
    {
      parent = parent->parent();
    }
  }

  return false;
}

QString URDFEditor::getValidName(QString prefix, QList<QString> &current_names)
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

void URDFEditor::on_robotTreeWidget_customContextMenuRequested(const QPoint &pos)
{

    QTreeWidgetItem* sel = ui->robotTreeWidget->selectedItems()[0];

    QMenu *menu = new QMenu(ui->robotTreeWidget);
    menu->addAction("Add");
    menu->addAction("Remove");
    QAction *selected_item = menu->exec(ui->robotTreeWidget->mapToGlobal(pos));
    if (selected_item)
    {
      if (selected_item->text() == "Add")
      {
        if (sel == links_ || sel->parent() == links_)
        {
          addLink();
        }
        else if (sel == joints_ || isJoint(sel))
        {
          addJoint(sel);
        }
      }
      else
      {
        if (sel->parent() == links_)
        {
          link_names.removeOne(sel->text(0));
          links_->removeChild(sel);
        }
        else if (isJoint(sel))
        {
          joint_names.removeOne(sel->text(0));
          sel->parent()->removeChild(sel);
        }
      }
    }

    delete menu;
}

void URDFEditor::on_robotTreeWidget_itemClicked(QTreeWidgetItem *item, int column)
{

  if (item->parent() == links_)
  {
    property_editor_->clear();
    property_editor_->setFactoryForManager(link_tree_->getManager(), variant_factory_);
    property_editor_->addProperty(link_tree_->getTopItem());
  }
  else if (isJoint(item))
  {
    property_editor_->clear();
    property_editor_->setFactoryForManager(joint_tree_->getManager(), variant_factory_);
    property_editor_->addProperty(joint_tree_->getTopItem());
  }
  ui->mainTabWidget->setCurrentIndex(0);
}
