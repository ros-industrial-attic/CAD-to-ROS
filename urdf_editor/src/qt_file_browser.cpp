#include <urdf_editor/qt_file_browser.h>
#include <qttreepropertybrowser.h>

#include <QHBoxLayout>
#include <QToolButton>
#include <QFileDialog>
#include <QFocusEvent>

namespace urdf_editor{
class FileBrowserPropertyType
{
};
}
Q_DECLARE_METATYPE(urdf_editor::FileBrowserPropertyType)

namespace urdf_editor {
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
