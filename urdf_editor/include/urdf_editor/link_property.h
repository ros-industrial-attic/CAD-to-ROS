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
   
    /*! Get the list of Visual Properties */
    std::vector<LinkVisualPropertySharedPtr> getVisualProperties();
    
    /*! Check if has collision property */
    bool hasCollisionProperty();
    
    /*! Create Collision property */
    void createCollisionProperty();
   
    /*! Get the list of Collision Properties */
    std::vector<LinkCollisionPropertySharedPtr> getCollisionProperties();

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

  private:
    void createVisualPropertyHelper(urdf::VisualSharedPtr data);
    void createCollisionPropertyHelper(urdf::CollisionSharedPtr data);

    urdf::LinkSharedPtr link_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    LinkInertialPropertySharedPtr inertial_property_;
    std::vector<LinkVisualPropertySharedPtr> visual_property_;
    std::vector<LinkCollisionPropertySharedPtr> collision_property_;
    QtVariantProperty *name_item_;
  };
}

#endif // __LINK_PROPERTY_H__
