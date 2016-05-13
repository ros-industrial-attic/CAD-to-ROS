#ifndef __LINK_GEOMETRY_PROPERTY_H__
#define __LINK_GEOMETRY_PROPERTY_H__

#include <QtCore>
#include <qwidget.h>
#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class LinkGeometryProperty : public QObject
  {
    Q_OBJECT
  public:
    LinkGeometryProperty(urdf::GeometrySharedPtr geometry);
    ~LinkGeometryProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    void loadData();
    void loadMesh();
    bool generateConvexMesh();

    QtProperty *getTopItem() { return top_item_; }

    urdf::GeometrySharedPtr getGeometry()
    {
      return geometry_;
    }


  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::GeometrySharedPtr geometry_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    QtVariantProperty *type_item_;
    bool loading_;
    std::string browse_start_dir_;

    std::map<int,urdf::GeometrySharedPtr> geometries_map_;


  };
}

#endif // __LINK_GEOMETRY_PROPERTY_H__
