#ifndef JOINT_TREE_PROPERTY
#define JOINT_TREE_PROPERTY
#include <QtCore>
#include "include/urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "include/urdf_editor/common.h"

namespace urdf_editor
{
  class JointTreeProperty : public QObject
  {
    Q_OBJECT
  public:
    JointTreeProperty();
    ~JointTreeProperty();

    QtVariantPropertyManager *getManager() { return manager_;}
    QtProperty *getTopItem() { return top_item_;}

  private slots:
    void jointValueChanged(QtProperty *property, const QVariant &val);

  private:

    class QtVariantPropertyManager *manager_;
    QtProperty *top_item_;
  };
}
#endif // JOINT_TREE_PROPERTY

