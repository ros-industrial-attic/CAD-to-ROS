#ifndef __ORIGIN_PROPERTY_H__
#define __ORIGIN_PROPERTY_H__

#define _USE_MATH_DEFINES
#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>
#include <cmath>


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

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    double meterToInch(double val);
    double inchToMeter(double val);
    double radianToDegree(double val);
    double degreeToRadian(double val);

    urdf::Pose &origin_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    int in_inches_;
    int in_degrees_;
    double r, p, y;
  };
}

#endif // __ORIGIN_PROPERTY_H__
