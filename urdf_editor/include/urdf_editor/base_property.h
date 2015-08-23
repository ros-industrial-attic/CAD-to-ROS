#ifndef BASE_PROPERTY_H
#define BASE_PROPERTY_H


class BaseProperty
{
public:
  BaseProperty();
};
class OriginProperty : public QObject
{
  Q_OBJECT
public:
  OriginProperty(urdf::Pose &origin);
  ~OriginProperty();

  void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

  QtProperty *getTopItem() { return top_item_; }

private slots:
  void originValueChanged(QtProperty *property, const QVariant &val);

private:
  boost::shared_ptr<urdf::Pose> origin_;
  QtVariantPropertyManager *manager_;
  QtVariantEditorFactory *factory_;
  QtProperty *top_item_;

};

#endif // BASE_PROPERTY_H
