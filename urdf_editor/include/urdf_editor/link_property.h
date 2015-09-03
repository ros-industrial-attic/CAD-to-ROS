#ifndef LINK_PROPERTY_H
#define LINK_PROPERTY_H
#include <QtCore>
#include <boost/shared_ptr.hpp>
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>
#include "urdf_editor/common.h"
#include "urdf_editor/joint_property.h"
#include <urdf_model/link.h>

namespace urdf_editor
{
  class LinkGeometryProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkGeometryProperty(boost::shared_ptr<urdf::Geometry> geometry);
    ~LinkGeometryProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkGeometryValueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Geometry> geometry_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
  };

  class LinkCollisionProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkCollisionProperty(boost::shared_ptr<urdf::Collision> collision);
    ~LinkCollisionProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkCollisionValueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Collision> collision_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<OriginProperty> origin_property_;
    boost::shared_ptr<LinkGeometryProperty> geometry_property_;

  };

  class LinkNewMaterialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkNewMaterialProperty(boost::shared_ptr<urdf::Material> material);
    ~LinkNewMaterialProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkNewMaterialValueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Material> material_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

  class LinkVisualProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkVisualProperty(boost::shared_ptr<urdf::Visual> visual);
    ~LinkVisualProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkVisualValueChanged(QtProperty *property, const QVariant &val);

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

  class LinkInertialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkInertialProperty(boost::shared_ptr<urdf::Inertial> inertial);
    ~LinkInertialProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkInertialValueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Inertial> inertial_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<OriginProperty> origin_property_;

  };

  class LinkProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkProperty(boost::shared_ptr<urdf::Link> link);
    ~LinkProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

  private slots:
    void linkValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void linkNameChanged(LinkProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Link> link_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<LinkInertialProperty> inertial_property_;
    boost::shared_ptr<LinkVisualProperty> visual_property_; // this needs to be array since multiple visuals models are allowed.
    boost::shared_ptr<LinkCollisionProperty> collision_property_; // this needs to be array since multiple collisions models are allowed.
    QtVariantProperty *name_item_;

  };
  typedef boost::shared_ptr<LinkProperty> LinkPropertyPtr;
}
#endif // LINK_TREE_PROPERTY_H
