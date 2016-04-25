#include "urdf_editor/urdf_property_tree.h"

URDFPropertyTree::URDFPropertyTree(QWidget *parent)
{
  setSelectionMode(QAbstractItemView::SingleSelection);
  setDragEnabled(true);
  viewport()->setAcceptDrops(true);
  setDropIndicatorShown(true);
  setDragDropMode(QAbstractItemView::InternalMove);
  setColumnCount(1);
}


void URDFPropertyTree::dropEvent(QDropEvent * event)
{

  QTreeWidget *tree = (QTreeWidget*)event->source();
  QTreeWidgetItem *item = tree->itemAt(event->pos());

  DropIndicatorPosition dp = dropIndicatorPosition();
  QModelIndex droppedIndex = indexAt( event->pos() );
  if( !droppedIndex.isValid() )
    return;

  // sanity check to prevent drags outside of links subtree -- don't loe this, 
  // but have to catch it before calling hte dropEvent() call on the tree
  if(item->text(0).toStdString() == "RobotModel" || item->text(0).toStdString() == "Joints") {
    return;
  }
  if(dp == QAbstractItemView::AboveItem) {
    if((item->parent()->text(0).toStdString() == "RobotModel") ||
       (item->parent()->text(0).toStdString() == "Joints")) {
      return;
    }
  }
  if(dp == QAbstractItemView::BelowItem || item->text(0).toStdString() == "Links"){
    return;
  }

  // this will actually reorder the tree appropraitely
  QTreeWidget::dropEvent(event);

  // send the appropriate item and drop indicator signal
  if (dp == QAbstractItemView::OnItem) {
    itemDropped(item);
  } else {
    itemDropped(item->parent());  
  }
}
