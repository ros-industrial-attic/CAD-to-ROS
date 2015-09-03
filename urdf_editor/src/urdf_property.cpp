#include "urdf_editor/urdf_property.h"
#include <QVBoxLayout>

namespace urdf_editor
{
  URDFProperty::URDFProperty(QTreeWidget *tree_widget, QWidget *browser_parent)
  {
    tree_widget_.reset(tree_widget);
    browser_parent_.reset(browser_parent);

    root_ = new QTreeWidgetItem(tree_widget_.get());
    root_->setText(0, "RobotModel");
    tree_widget_->addTopLevelItem(root_);

    link_root_ = new QTreeWidgetItem();
    link_root_->setText(0, "Links");
    root_->addChild(link_root_);

    joint_root_ = new QTreeWidgetItem();
    joint_root_->setText(0,"Chain");
    root_->addChild(joint_root_);

    variant_factory_.reset(new QtVariantEditorFactory());

    property_editor_.reset(new QtTreePropertyBrowser());
    property_editor_->setContextMenuPolicy(Qt::CustomContextMenu);
    property_editor_->setPropertiesWithoutValueMarked(false);
    property_editor_->setRootIsDecorated(true);
    property_editor_->setResizeMode(QtTreePropertyBrowser::ResizeToContents);

    QVBoxLayout *vlayout = new QVBoxLayout(browser_parent_.get());
    vlayout->setMargin(0);
    vlayout->addWidget(property_editor_.get());

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

  bool URDFProperty::loadURDF(QString file_path)
  {
    model_ = urdf::parseURDFFile(file_path.toStdString());
    if (model_)
      if (buildTree())
        return true;
      else
        return false;
    else
      return false;
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

  bool URDFProperty::isJoint(QTreeWidgetItem *item)
  {
    QTreeWidgetItem *parent = item;
    while (parent->parent())
    {
      if (parent->parent() == joint_root_)
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

  void URDFProperty::on_treeWidget_customContextMenuRequested(const QPoint &pos)
  {

      QTreeWidgetItem *sel = tree_widget_->selectedItems()[0];

      QMenu *menu = new QMenu(tree_widget_.get());
      menu->addAction("Add");
      menu->addAction("Remove");
      QAction *selected_item = menu->exec(tree_widget_->mapToGlobal(pos));
      if (selected_item)
      {
        if (selected_item->text() == "Add")
        {
          if (sel == link_root_ || sel->parent() == link_root_)
          {
            addLink();
          }
          else if (sel == joint_root_ || isJoint(sel))
          {
            addJoint(sel);
          }
        }
        else
        {
          if (sel->parent() == link_root_)
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

    if (item->parent() == link_root_)
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
    qDebug() << "property custom menu";
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

}
