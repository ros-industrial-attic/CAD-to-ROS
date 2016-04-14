#ifndef __LINK_PROPERTY_H__
#define __LINK_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/common.h>


namespace urdf
{
  class Link;
}

namespace urdf_editor
{
  // forward declared
  //class OriginProperty;
  class LinkGeometryProperty;
  class LinkInertialProperty;
  class LinkVisualProperty;
  class LinkCollisionProperty;

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

#endif // __LINK_PROPERTY_H__
