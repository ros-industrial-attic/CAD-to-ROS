#ifndef __ORIGIN_PROPERTY_H__
#define __ORIGIN_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class OriginProperty : public QObject
  {
    Q_OBJECT
  public:
    OriginProperty(urdf::Pose &origin);
    ~OriginProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    void loadData();

    urdf::Pose getOriginData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::Pose &origin_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
  };
}

#endif // __ORIGIN_PROPERTY_H__
