#ifndef __LINK_GEOMETRY_PROPERTY_H__
#define __LINK_GEOMETRY_PROPERTY_H__

#include <QtCore>
#include <qtvariantproperty.h>
#include <QLineEdit>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class LinkGeometryProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkGeometryProperty(urdf::GeometrySharedPtr geometry);
    ~LinkGeometryProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::GeometrySharedPtr geometry_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
  };

  class FileBrowserVariantManager : public QtVariantPropertyManager
  {
      Q_OBJECT
  public:
      FileBrowserVariantManager(QObject *parent = 0)
          : QtVariantPropertyManager(parent)
              { }

      virtual QVariant value(const QtProperty *property) const;
      virtual int valueType(int propertyType) const;
      virtual bool isPropertyTypeSupported(int propertyType) const;

      virtual QStringList attributes(int propertyType) const;
      virtual int attributeType(int propertyType, const QString &attribute) const;
      virtual QVariant attributeValue(const QtProperty *property, const QString &attribute);


      static int filePathTypeId();
  public slots:
      virtual void setValue(QtProperty *property, const QVariant &val);
      virtual void setAttribute(QtProperty *property,
                  const QString &attribute, const QVariant &value);
  protected:
      virtual QString valueText(const QtProperty *property) const;
      virtual void initializeProperty(QtProperty *property);
      virtual void uninitializeProperty(QtProperty *property);
  private:
      struct Data {
          QString value;
          QString filter;
      };
      QMap<const QtProperty *, Data> theValues;
  };

  class FileEdit : public QWidget
  {
      Q_OBJECT
  public:
      FileEdit(QWidget *parent = 0);
      void setFilePath(const QString &filePath) { if (theLineEdit->text() != filePath) theLineEdit->setText(filePath); }
      QString filePath() const { return theLineEdit->text(); }
      void setFilter(const QString &filter) { theFilter = filter; }
      QString filter() const { return theFilter; }
  signals:
      void filePathChanged(const QString &filePath);
  protected:
      void focusInEvent(QFocusEvent *e);
      void focusOutEvent(QFocusEvent *e);
      void keyPressEvent(QKeyEvent *e);
      void keyReleaseEvent(QKeyEvent *e);
  private slots:
      void buttonClicked();
  private:
      QLineEdit *theLineEdit;
      QString theFilter;
  };

  class FileBrowserVarianFactory : public QtVariantEditorFactory
  {
      Q_OBJECT
  public:
      FileBrowserVarianFactory(QObject *parent = 0)
          : QtVariantEditorFactory(parent)
              { }

      virtual ~FileBrowserVarianFactory();
  protected:
      virtual void connectPropertyManager(QtVariantPropertyManager *manager);
      virtual QWidget *createEditor(QtVariantPropertyManager *manager, QtProperty *property,
                  QWidget *parent);
      virtual void disconnectPropertyManager(QtVariantPropertyManager *manager);
  private slots:
      void slotPropertyChanged(QtProperty *property, const QVariant &value);
      void slotPropertyAttributeChanged(QtProperty *property, const QString &attribute, const QVariant &value);
      void slotSetValue(const QString &value);
      void slotEditorDestroyed(QObject *object);
  private:
      QMap<QtProperty *, QList<FileEdit *> > theCreatedEditors;
      QMap<FileEdit *, QtProperty *> theEditorToProperty;
  };
}

#endif // __LINK_GEOMETRY_PROPERTY_H__
