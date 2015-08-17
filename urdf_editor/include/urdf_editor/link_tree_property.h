#ifndef LINK_TREE_PROPERTY_H
#define LINK_TREE_PROPERTY_H
#include <QtCore>
#include "include/urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "include/urdf_editor/common.h"

namespace urdf_editor
{
  class LinkTreeProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkTreeProperty();
    ~LinkTreeProperty();

    QtVariantPropertyManager *getManager() { return manager_;}
    QtProperty *getTopItem() { return top_item_;}

  private slots:
    void linkValueChanged(QtProperty *property, const QVariant &val);

  private:
    void addVisualProperty();
    void addCollisionProperty();
    void addInertialProperty();

    class QtVariantPropertyManager *manager_;
    QtProperty *top_item_;
  };
}
#endif // LINK_TREE_PROPERTY_H
