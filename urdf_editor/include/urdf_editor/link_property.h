#ifndef LINK_PROPERTY_H
#define LINK_PROPERTY_H
#include <QtCore>
#include <boost/shared_ptr.hpp>
#include <include/urdf_editor/qtpropertybrowser/qttreepropertybrowser.h>
#include "include/urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "include/urdf_editor/common.h"
#include "include/urdf_editor/joint_property.h"
#include <urdf_model/link.h>

namespace urdf_editor
{
  class LinkInertialProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkInertialProperty(boost::shared_ptr<urdf::Inertial> inertial);
    ~LinkInertialProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void linkInertialValueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::Inertial> inertial_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    boost::shared_ptr<OriginProperty> origin_property_;

  };

  class LinkProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkProperty(boost::shared_ptr<urdf::Link> link);
    ~LinkProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

  private slots:
    void linkValueChanged(QtProperty *property, const QVariant &val);

  private:
    void addVisualProperty();
    void addCollisionProperty();
    void addInertialProperty();

    boost::shared_ptr<urdf::Link> link_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_; // this needs to be removed
    boost::shared_ptr<LinkInertialProperty> inertial_property_;
    QtVariantProperty *name_item_;

  };
  typedef boost::shared_ptr<LinkProperty> LinkPropertyPtr;
}
#endif // LINK_TREE_PROPERTY_H
