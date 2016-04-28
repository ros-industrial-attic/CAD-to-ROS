#include "urdf_editor/urdf_property_tree.h"

URDFPropertyTree::URDFPropertyTree(QWidget *parent) :
  drag_item(NULL)
{
  setSelectionMode(QAbstractItemView::SingleSelection);
  setDragEnabled(true);
  viewport()->setAcceptDrops(true);
  setDropIndicatorShown(true);
  setDragDropMode(QAbstractItemView::InternalMove);
  setColumnCount(1);

  connect(this, SIGNAL(itemPressed(QTreeWidgetItem*,int)),
          this, SLOT(on_itemPressed(QTreeWidgetItem*,int)));
}

void URDFPropertyTree::on_itemPressed(QTreeWidgetItem *item, int column)
{
  drag_item = item;
}

void URDFPropertyTree::dropEvent(QDropEvent * event)
{

  QTreeWidget *tree = (QTreeWidget*)event->source();
  QTreeWidgetItem *item = tree->itemAt(event->pos());

  QModelIndex droppedIndex = indexAt( event->pos() );
  if( !droppedIndex.isValid() )
    return;

  DropIndicatorPosition dp = dropIndicatorPosition();
  // send the appropriate item and drop indicator signal
  if (dp == QAbstractItemView::OnItem)
  {
    emit dragDropEvent(drag_item, item);
  }
  else if (dp == QAbstractItemView::AboveItem || dp == QAbstractItemView::BelowItem)
  {
    emit dragDropEvent(drag_item, item->parent());
  }


//  // sanity check to prevent drags outside of links subtree
//  if(item->text(0) == tree->topLevelItem(0)->text(0)) {
//    return;
//  }

//  // this will actually reorder the tree appropraitely
//  QTreeWidget::dropEvent(event);
  

 
}
