/**
 * Forward declared Qt classes (and their shared pointer type).
 *
 * Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
 */
#ifndef __QT_FORWARD_DECLS_H__
#define __QT_FORWARD_DECLS_H__

#include <boost/shared_ptr.hpp>

#define QT_TYPEDEF_CLASS_POINTER(Class) \
class Class; \
typedef boost::shared_ptr<Class> Class##SharedPtr;


QT_TYPEDEF_CLASS_POINTER(QtProperty);
QT_TYPEDEF_CLASS_POINTER(QtTreePropertyBrowser);
QT_TYPEDEF_CLASS_POINTER(QtVariantEditorFactory);
QT_TYPEDEF_CLASS_POINTER(QtVariantProperty);
QT_TYPEDEF_CLASS_POINTER(QtVariantPropertyManager);


#endif // __QT_FORWARD_DECLS_H__
