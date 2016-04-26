#ifndef __LINK_INERTIAL_PROPERTY_H__
#define __LINK_INERTIAL_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>
#include <urdf_editor/property_types.h>


namespace urdf_editor
{
  class LinkInertialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkInertialProperty(urdf::InertialSharedPtr inertial);
    ~LinkInertialProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

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
    urdf::InertialSharedPtr inertial_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    OriginPropertySharedPtr origin_property_;
  };
}

#endif // __LINK_INERTIAL_PROPERTY_H__
