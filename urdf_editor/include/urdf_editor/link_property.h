#ifndef __LINK_PROPERTY_H__
#define __LINK_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/common.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class LinkProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkProperty(urdf::LinkSharedPtr link);
    ~LinkProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    bool hasInertialProperty();
    void createInertialProperty();
    LinkInertialPropertySharedPtr getInertialProperty();

    
    /*! Check if has visual property */
    bool hasVisualProperty();
    
    /*! Create Visual property */
    void createVisualProperty();
   
    /*! Get the Inertial Property */
    LinkVisualPropertySharedPtr getVisualProperty();
    
    /*! Check if has collision property */
    bool hasCollisionProperty();
    
    /*! Create Collision property */
    void createCollisionProperty();
   
    /*! Get the Collision Property */
    LinkCollisionPropertySharedPtr getCollisionProperty();

    /*! Get the link name */
    QString getName();

    
  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);
    void onVisualGeometryChanged(int);
    void onCollisionGeometryChanged(int);

  signals:
    void linkNameChanged(LinkProperty *property, const QVariant &val);
    void valueChanged(LinkProperty *property);
    void linkVisibilityChanged(const QString &link_name, const bool &val);

  private:
    urdf::LinkSharedPtr link_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    LinkInertialPropertySharedPtr inertial_property_;
    LinkVisualPropertySharedPtr visual_property_; // this needs to be array since multiple visuals models are allowed.
    LinkCollisionPropertySharedPtr collision_property_; // this needs to be array since multiple collisions models are allowed.
    QtVariantProperty *name_item_;
    QtVariantProperty *visibility_item_;
  };
}

#endif // __LINK_PROPERTY_H__
