#ifndef __LINK_INERTIAL_PROPERTY_H__
#define __LINK_INERTIAL_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  // forward declared
  class OriginProperty;

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
}

#endif // __LINK_INERTIAL_PROPERTY_H__
