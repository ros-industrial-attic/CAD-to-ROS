#include "industrial_robot_builder.h"
#include "ui_industrial_robot_builder.h"
#include <sstream>

IndustrialRobotBuilder::IndustrialRobotBuilder(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::IndustrialRobotBuilder)
{
  ui->setupUi(this);

  link_page_ = ui->mainTabWidget->widget(0);
  joint_page_ = ui->mainTabWidget->widget(1);
  ui->mainTabWidget->removeTab(0);
  ui->mainTabWidget->removeTab(0);

  root_ = new QTreeWidgetItem(ui->robotTreeWidget);
  root_->setText(0, "RobotModel");
  ui->robotTreeWidget->addTopLevelItem(root_);

  links_ = new QTreeWidgetItem();
  links_->setText(0,"Links");
  root_->addChild(links_);

  joints_ = new QTreeWidgetItem();
  joints_->setText(0,"Chain");
  root_->addChild(joints_);

  addLink();
  addLink();
}

IndustrialRobotBuilder::~IndustrialRobotBuilder()
{
  delete ui;
}

void IndustrialRobotBuilder::addLink()
{
  QTreeWidgetItem *itm = new QTreeWidgetItem(links_);
  QString name = getValidName("link_", link_names);
  itm->setText(0, name);
  root_->addChild(itm);

  link_names.append(name);
}

void IndustrialRobotBuilder::addJoint(QTreeWidgetItem *parent)
{
  QTreeWidgetItem *itm = new QTreeWidgetItem(parent);
  QString name = getValidName("joint_", joint_names);
  itm->setText(0, name);
  root_->addChild(itm);

  joint_names.append(name);
}

bool IndustrialRobotBuilder::isJoint(QTreeWidgetItem *itm)
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

QString IndustrialRobotBuilder::getValidName(QString prefix, QList<QString> &current_names)
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



void IndustrialRobotBuilder::on_robotTreeWidget_customContextMenuRequested(const QPoint &pos)
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

void IndustrialRobotBuilder::on_robotTreeWidget_itemClicked(QTreeWidgetItem *item, int column)
{

  if (item->parent() == links_)
  {
    if (ui->mainTabWidget->count() == 1)
    {
      ui->mainTabWidget->insertTab(0, link_page_, "Link Details");
    }
    else if (ui->mainTabWidget->widget(0) != link_page_)
    {
      ui->mainTabWidget->removeTab(0);
      ui->mainTabWidget->insertTab(0, link_page_, "Link Details");
    }
  }
  else if (isJoint(item))
  {
    if (ui->mainTabWidget->count() == 1)
    {
      ui->mainTabWidget->insertTab(0, joint_page_, "Joint Details");
    }
    else if (ui->mainTabWidget->widget(0) != joint_page_)
    {
      ui->mainTabWidget->removeTab(0);
      ui->mainTabWidget->insertTab(0, joint_page_, "Joint Details");
    }
  }
  else
  {
    if (ui->mainTabWidget->count() != 1)
    {
      ui->mainTabWidget->removeTab(0);
    }
  }
  ui->mainTabWidget->setCurrentIndex(0);
}
