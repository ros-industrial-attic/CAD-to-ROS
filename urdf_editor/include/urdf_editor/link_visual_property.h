#ifndef __LINK_VISUAL_PROPERTY_H__
#define __LINK_VISUAL_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  // forward declared
  class LinkGeometryProperty;
  class LinkNewMaterialProperty;
  class OriginProperty;

  class LinkVisualProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkVisualProperty(boost::shared_ptr<urdf::Visual> visual);
    ~LinkVisualProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    bool hasOriginProperty();
    void createOriginProperty();
    
    bool hasGeometryProperty();
    void createGeometryProperty();
    
    bool hasMaterialProperty();
    void createMaterialProperty();
    
    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Visual> visual_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<OriginProperty> origin_property_;
    boost::shared_ptr<LinkNewMaterialProperty> new_material_property_;
    boost::shared_ptr<LinkGeometryProperty> geometry_property_;

  };
}

#endif // __LINK_VISUAL_PROPERTY_H__
