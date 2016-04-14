#ifndef __LINK_COLLISION_PROPERTY_H__
#define __LINK_COLLISION_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <boost/shared_ptr.hpp>


namespace urdf
{
  class Collision;
}

namespace urdf_editor
{
  // forward declared
  class OriginProperty;
  class LinkGeometryProperty;

  class LinkCollisionProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkCollisionProperty(boost::shared_ptr<urdf::Collision> collision);
    ~LinkCollisionProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    bool hasOriginProperty();

    void createOriginProperty();
    
    bool hasGeometryProperty();
    void createGeometryProperty();
    
    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Collision> collision_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<OriginProperty> origin_property_;
    boost::shared_ptr<LinkGeometryProperty> geometry_property_;
  };
}

#endif // __LINK_COLLISION_PROPERTY_H__
