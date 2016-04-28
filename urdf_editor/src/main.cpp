
#include <urdf_editor/urdf_editor.h>

#include <QApplication>

#include <ros/ros.h>

#include <urdf_editor/link_property.h>
#include "qttreepropertybrowser.h"


int main(int argc, char *argv[])
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "urdf_builder");
  }

  QApplication a(argc, argv);
  URDFEditor w;
  w.show();
  return a.exec();
//  QApplication app (argc, argv);

//  QtVariantPropertyManager *variantManager = new urdf_editor::FileBrowserVariantManager();
//  QtVariantProperty *priority = variantManager->addProperty(QVariant::Int, "Priority");

//  priority->setToolTip("Task Priority");

//  priority->setAttribute("minimum", 1);
//  priority->setAttribute("maximum", 5);
//  priority->setValue(3);

//  QtVariantProperty *reportType = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "Report Type");
//  QStringList types;
//  types << "Bug" << "Suggestion" << "To Do";
//  reportType->setAttribute("enumNames", types);
//  reportType->setValue(1); // current value will be "Suggestion"

//  QtVariantProperty *task1 = variantManager->addProperty(QtVariantPropertyManager::groupTypeId(), "Task 1");

//  task1->addSubProperty(priority);
//  task1->addSubProperty(reportType);

//  QtTreePropertyBrowser *browser = new QtTreePropertyBrowser();

//  QtVariantEditorFactory *variantFactory = new urdf_editor::FileBrowserVarianFactory();
//  browser->setFactoryForManager(variantManager, variantFactory);

//  browser->addProperty(task1);
//  browser->show();

//  QtVariantProperty *example = variantManager->addProperty(urdf_editor::FileBrowserVariantManager::filePathTypeId(), "Example");
//  example->setValue("main.cpp");
//  example->setAttribute("filter", "Source files (*.cpp *.c)");

//  task1->addSubProperty(example);

//  return app.exec();



//  ROS_WARN("HI");
}
