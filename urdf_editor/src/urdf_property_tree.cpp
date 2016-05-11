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

  // sanity check to prevent drags outside of links subtree
  if(item->text(0) == tree->topLevelItem(0)->text(0)) {
    return;
  }

  // this will actually reorder the tree appropraitely
  QTreeWidget::dropEvent(event);  
  
  // send the appropriate item and drop indicator signal
  if (dp == QAbstractItemView::OnItem) {
    itemDropped(item);
  } else if (dp == QAbstractItemView::AboveItem || dp == QAbstractItemView::BelowItem) {
    itemDropped(item->parent());  
  }
 
}
