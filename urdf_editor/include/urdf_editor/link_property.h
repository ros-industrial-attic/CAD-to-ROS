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

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

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

  class LinkInertialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkInertialProperty(boost::shared_ptr<urdf::Inertial> inertial);
    ~LinkInertialProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    bool hasOriginProperty();

    void createOriginProperty();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Inertial> inertial_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    boost::shared_ptr<OriginProperty> origin_property_;

  };
  typedef boost::shared_ptr<LinkInertialProperty> LinkInertialPropertyPtr;
  
  /*! LinkVisualProperty pointer */
  typedef boost::shared_ptr<LinkVisualProperty> LinkVisualPropertyPtr;
  
  /*! LinkCollisionProperty pointer */
  typedef boost::shared_ptr<LinkCollisionProperty> LinkCollisionPropertyPtr;
  
  class LinkProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkProperty(boost::shared_ptr<urdf::Link> link);
    ~LinkProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    bool hasInertialProperty();
    void createInertialProperty();
    LinkInertialPropertyPtr getInertialProperty();

    
    /*! Check if has visual property */
    bool hasVisualProperty();
    
    /*! Create Visual property */
    void createVisualProperty();
   
    /*! Get the Inertial Property */
    LinkVisualPropertyPtr getVisualProperty();
    
    /*! Check if has collision property */
    bool hasCollisionProperty();
    
    /*! Create Collision property */
    void createCollisionProperty();
   
    /*! Get the Collision Property */
    LinkCollisionPropertyPtr getCollisionProperty();
    
    
  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void linkNameChanged(LinkProperty *property, const QVariant &val);
    void linkVisibilityChanged(LinkProperty *property, const QVariant &val);
    void valueChanged();

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
