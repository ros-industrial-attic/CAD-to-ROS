#ifndef __LINK_NEW_MATERIAL_PROPERTY_H__
#define __LINK_NEW_MATERIAL_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  class LinkNewMaterialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkNewMaterialProperty(boost::shared_ptr<urdf::Material> material);
    ~LinkNewMaterialProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Material> material_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
  };
}

#endif // __LINK_NEW_MATERIAL_PROPERTY_H__
