#ifndef __LINK_COLLISION_PROPERTY_H__
#define __LINK_COLLISION_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>
#include <urdf_editor/property_types.h>


namespace urdf_editor
{
  class LinkCollisionProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkCollisionProperty(urdf::CollisionSharedPtr collision);
    ~LinkCollisionProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    bool hasOriginProperty();

    void createOriginProperty();
    
    bool hasGeometryProperty();
    void createGeometryProperty();
    
    void loadData();

    QtProperty *getTopItem() { return top_item_; }

    LinkGeometryPropertySharedPtr getGeometryProperty()
    {
      return geometry_property_;
    }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::CollisionSharedPtr collision_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    OriginPropertySharedPtr origin_property_;
    LinkGeometryPropertySharedPtr geometry_property_;
  };
}

#endif // __LINK_COLLISION_PROPERTY_H__
