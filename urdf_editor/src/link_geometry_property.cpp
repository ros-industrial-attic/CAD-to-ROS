
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>
#include <QHBoxLayout>
#include <QToolButton>
#include <QFileDialog>
#include <QFocusEvent>

#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/link.h>

namespace urdf_editor{
class FileBrowserPropertyType
{
};
}
Q_DECLARE_METATYPE(urdf_editor::FileBrowserPropertyType)

namespace urdf_editor
{
  LinkGeometryProperty::LinkGeometryProperty(urdf::GeometrySharedPtr geometry): geometry_(geometry), manager_(new FileBrowserVariantManager()), factory_(new FileBrowserVarianFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));
    //{SPHERE, BOX, CYLINDER, MESH}
    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Geometry"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << "SPHERE" << "BOX" << "CYLINDER" << "MESH");
    top_item_->addSubProperty(item);

    if (geometry_->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Size"));
      sub_item = manager_->addProperty(QVariant::Double, tr("Length X (m)"));
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Y (m)"));
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Z (m)"));
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QVariant::Double, tr("Length (m)"));
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
      item = manager_->addProperty(FileBrowserVariantManager::filePathTypeId(), tr("File Name"));
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Scale"));
      sub_item = manager_->addProperty(QVariant::Double, tr("X"));
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Y"));
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Z"));
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }
    loading_ = false;
  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      if (name == "Type")
        item->setValue(geometry_->type);
      else
      {
        if (geometry_->type == urdf::Geometry::BOX)
        {
          boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
          if (name == "Length X (m)")
            item->setValue(box->dim.x);
          else if (name == "Length Y (m)")
            item->setValue(box->dim.y);
          else if (name == "Length Z (m)")
            item->setValue(box->dim.z);
        }
        else if (geometry_->type == urdf::Geometry::CYLINDER)
        {
          boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
          if (name == "Radius (m)")
            item->setValue(cylinder->radius);
          else if (name == "Length (m)")
            item->setValue(cylinder->length);
        }
        else if (geometry_->type == urdf::Geometry::SPHERE)
        {
          boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
          if (name == "Radius (m)")
            item->setValue(sphere->radius);
        }
        else if (geometry_->type == urdf::Geometry::MESH)
        {
          boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
          if (name == "File Name")
            item->setValue(QString::fromStdString(mesh->filename));
          else if (name == "X")
            item->setValue(mesh->scale.x);
          else if (name == "Y")
            item->setValue(mesh->scale.y);
          else if (name == "Z")
            item->setValue(mesh->scale.z);
        }
      }
    }
    loading_ = false;
  }

  void LinkGeometryProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkGeometryProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Type")
    {
      // if type is changed need to add functionality to remove current geometry and add new one
      switch (val.toInt()) //{SPHERE, BOX, CYLINDER, MESH}
      {
      case 0:
        geometry_->type = urdf::Geometry::SPHERE;
        break;
      case 1:
        geometry_->type = urdf::Geometry::BOX;
        break;
      case 2:
        geometry_->type = urdf::Geometry::CYLINDER;
        break;
      case 3:
        geometry_->type = urdf::Geometry::MESH;
        break;
      }
    }
    else
    {
      if (geometry_->type == urdf::Geometry::BOX)
      {
        boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
        if (name == "Length X (m)")
          box->dim.x = val.toDouble();
        else if (name == "Length Y (m)")
          box->dim.y = val.toDouble();
        else if (name == "Length Z (m)")
          box->dim.z = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::CYLINDER)
      {
        boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
        if (name == "Radius (m)")
          cylinder->radius = val.toDouble();
        else if (name == "Length (m)")
          cylinder->length = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::SPHERE)
      {
        boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
        if (name == "Radius (m)")
          sphere->radius = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::MESH)
      {
        boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
        if (name == "File Name")
          mesh->filename = val.toString().toStdString();
        else if (name == "X")
          mesh->scale.x = val.toDouble();
        else if (name == "Y")
          mesh->scale.y = val.toDouble();
        else if (name == "Z")
          mesh->scale.z = val.toDouble();
      }
    }

    emit LinkGeometryProperty::valueChanged(property, val);
  }

  int FileBrowserVariantManager::filePathTypeId()
  {
      return qMetaTypeId<FileBrowserPropertyType>();
  }

  bool FileBrowserVariantManager::isPropertyTypeSupported(int propertyType) const
  {
      if (propertyType == filePathTypeId())
          return true;
      return QtVariantPropertyManager::isPropertyTypeSupported(propertyType);
  }

  int FileBrowserVariantManager::valueType(int propertyType) const
  {
      if (propertyType == filePathTypeId())
          return QVariant::String;
      return QtVariantPropertyManager::valueType(propertyType);
  }

  QVariant FileBrowserVariantManager::value(const QtProperty *property) const
  {
      if (theValues.contains(property))
          return theValues[property].value;
      return QtVariantPropertyManager::value(property);
  }

  QStringList FileBrowserVariantManager::attributes(int propertyType) const
  {
      if (propertyType == filePathTypeId()) {
          QStringList attr;
          attr << QLatin1String("filter");
          return attr;
      }
      return QtVariantPropertyManager::attributes(propertyType);
  }

  int FileBrowserVariantManager::attributeType(int propertyType, const QString &attribute) const
  {
      if (propertyType == filePathTypeId()) {
          if (attribute == QLatin1String("filter"))
              return QVariant::String;
          return 0;
      }
      return QtVariantPropertyManager::attributeType(propertyType, attribute);
  }

  QVariant FileBrowserVariantManager::attributeValue(const QtProperty *property, const QString &attribute)
  {
      if (theValues.contains(property)) {
          if (attribute == QLatin1String("filter"))
              return theValues[property].filter;
          return QVariant();
      }
      return QtVariantPropertyManager::attributeValue(property, attribute);
  }

  QString FileBrowserVariantManager::valueText(const QtProperty *property) const
  {
      if (theValues.contains(property))
          return theValues[property].value;
      return QtVariantPropertyManager::valueText(property);
  }

  void FileBrowserVariantManager::setValue(QtProperty *property, const QVariant &val)
  {
      if (theValues.contains(property)) {
          if (val.type() != QVariant::String && !val.canConvert(QVariant::String))
              return;
          QString str = qVariantValue<QString>(val);
          Data d = theValues[property];
          if (d.value == str)
              return;
          d.value = str;
          theValues[property] = d;
          emit propertyChanged(property);
          emit valueChanged(property, str);
          return;
      }
      QtVariantPropertyManager::setValue(property, val);
  }

  void FileBrowserVariantManager::setAttribute(QtProperty *property,
                  const QString &attribute, const QVariant &val)
  {
      if (theValues.contains(property)) {
          if (attribute == QLatin1String("filter")) {
              if (val.type() != QVariant::String && !val.canConvert(QVariant::String))
                  return;
              QString str = qVariantValue<QString>(val);
              Data d = theValues[property];
              if (d.filter == str)
                  return;
              d.filter = str;
              theValues[property] = d;
              emit attributeChanged(property, attribute, str);
          }
          return;
      }
      QtVariantPropertyManager::setAttribute(property, attribute, val);
  }

  void FileBrowserVariantManager::initializeProperty(QtProperty *property)
  {
      if (propertyType(property) == filePathTypeId())
          theValues[property] = Data();
      QtVariantPropertyManager::initializeProperty(property);
  }

  void FileBrowserVariantManager::uninitializeProperty(QtProperty *property)
  {
      theValues.remove(property);
      QtVariantPropertyManager::uninitializeProperty(property);
  }

  FileEdit::FileEdit(QWidget *parent)
      : QWidget(parent)
  {
      QHBoxLayout *layout = new QHBoxLayout(this);
      layout->setMargin(0);
      layout->setSpacing(0);
      theLineEdit = new QLineEdit(this);
      theLineEdit->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      QToolButton *button = new QToolButton(this);
      button->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred));
      button->setText(QLatin1String("..."));
      layout->addWidget(theLineEdit);
      layout->addWidget(button);
      setFocusProxy(theLineEdit);
      setFocusPolicy(Qt::StrongFocus);
      setAttribute(Qt::WA_InputMethodEnabled);
      connect(theLineEdit, SIGNAL(textEdited(const QString &)),
                  this, SIGNAL(filePathChanged(const QString &)));
      connect(button, SIGNAL(clicked()),
                  this, SLOT(buttonClicked()));
  }

  void FileEdit::buttonClicked()
  {
      QString filePath = QFileDialog::getOpenFileName(this, tr("Choose a file"), theLineEdit->text(), theFilter);
      if (filePath.isNull())
          return;
      theLineEdit->setText(filePath);
      emit filePathChanged(filePath);
  }

  void FileEdit::focusInEvent(QFocusEvent *e)
  {
      theLineEdit->event(e);
      if (e->reason() == Qt::TabFocusReason || e->reason() == Qt::BacktabFocusReason) {
          theLineEdit->selectAll();
      }
      QWidget::focusInEvent(e);
  }

  void FileEdit::focusOutEvent(QFocusEvent *e)
  {
      theLineEdit->event(e);
      QWidget::focusOutEvent(e);
  }

  void FileEdit::keyPressEvent(QKeyEvent *e)
  {
      theLineEdit->event(e);
  }

  void FileEdit::keyReleaseEvent(QKeyEvent *e)
  {
      theLineEdit->event(e);
  }

  FileBrowserVarianFactory::~FileBrowserVarianFactory()
  {
      QList<FileEdit *> editors = theEditorToProperty.keys();
      QListIterator<FileEdit *> it(editors);
      while (it.hasNext())
          delete it.next();
  }

  void FileBrowserVarianFactory::connectPropertyManager(QtVariantPropertyManager *manager)
  {
      connect(manager, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                  this, SLOT(slotPropertyChanged(QtProperty *, const QVariant &)));
      connect(manager, SIGNAL(attributeChanged(QtProperty *, const QString &, const QVariant &)),
                  this, SLOT(slotPropertyAttributeChanged(QtProperty *, const QString &, const QVariant &)));
      QtVariantEditorFactory::connectPropertyManager(manager);
  }

  QWidget *FileBrowserVarianFactory::createEditor(QtVariantPropertyManager *manager,
          QtProperty *property, QWidget *parent)
  {
      if (manager->propertyType(property) == FileBrowserVariantManager::filePathTypeId()) {
          FileEdit *editor = new FileEdit(parent);
          editor->setFilePath(manager->value(property).toString());
          editor->setFilter(manager->attributeValue(property, QLatin1String("filter")).toString());
          theCreatedEditors[property].append(editor);
          theEditorToProperty[editor] = property;

          connect(editor, SIGNAL(filePathChanged(const QString &)),
                  this, SLOT(slotSetValue(const QString &)));
          connect(editor, SIGNAL(destroyed(QObject *)),
                  this, SLOT(slotEditorDestroyed(QObject *)));
          return editor;
      }
      return QtVariantEditorFactory::createEditor(manager, property, parent);
  }

  void FileBrowserVarianFactory::disconnectPropertyManager(QtVariantPropertyManager *manager)
  {
      disconnect(manager, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                  this, SLOT(slotPropertyChanged(QtProperty *, const QVariant &)));
      disconnect(manager, SIGNAL(attributeChanged(QtProperty *, const QString &, const QVariant &)),
                  this, SLOT(slotPropertyAttributeChanged(QtProperty *, const QString &, const QVariant &)));
      QtVariantEditorFactory::disconnectPropertyManager(manager);
  }

  void FileBrowserVarianFactory::slotPropertyChanged(QtProperty *property,
                  const QVariant &value)
  {
      if (!theCreatedEditors.contains(property))
          return;

      QList<FileEdit *> editors = theCreatedEditors[property];
      QListIterator<FileEdit *> itEditor(editors);
      while (itEditor.hasNext())
          itEditor.next()->setFilePath(value.toString());
  }

  void FileBrowserVarianFactory::slotPropertyAttributeChanged(QtProperty *property,
              const QString &attribute, const QVariant &value)
  {
      if (!theCreatedEditors.contains(property))
          return;

      if (attribute != QLatin1String("filter"))
          return;

      QList<FileEdit *> editors = theCreatedEditors[property];
      QListIterator<FileEdit *> itEditor(editors);
      while (itEditor.hasNext())
          itEditor.next()->setFilter(value.toString());
  }

  void FileBrowserVarianFactory::slotSetValue(const QString &value)
  {
      QObject *object = sender();
      QMap<FileEdit *, QtProperty *>::ConstIterator itEditor =
                  theEditorToProperty.constBegin();
      while (itEditor != theEditorToProperty.constEnd()) {
          if (itEditor.key() == object) {
              QtProperty *property = itEditor.value();
              QtVariantPropertyManager *manager = propertyManager(property);
              if (!manager)
                  return;
              manager->setValue(property, value);
              return;
          }
          itEditor++;
      }
  }

  void FileBrowserVarianFactory::slotEditorDestroyed(QObject *object)
  {
      QMap<FileEdit *, QtProperty *>::ConstIterator itEditor =
                  theEditorToProperty.constBegin();
      while (itEditor != theEditorToProperty.constEnd()) {
          if (itEditor.key() == object) {
              FileEdit *editor = itEditor.key();
              QtProperty *property = itEditor.value();
              theEditorToProperty.remove(editor);
              theCreatedEditors[property].removeAll(editor);
              if (theCreatedEditors[property].isEmpty())
                  theCreatedEditors.remove(property);
              return;
          }
          itEditor++;
      }
  }
}
