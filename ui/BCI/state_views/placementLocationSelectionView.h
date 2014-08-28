#ifndef PLACEMENTLOCATIONSELECTIONVIEW_H
#define PLACEMENTLOCATIONSELECTIONVIEW_H

#include <QWidget>

namespace Ui {
class PlacementLocationSelectionView;
}

class PlacementLocationSelectionView : public QWidget
{
    Q_OBJECT
    
public:
    explicit PlacementLocationSelectionView(QWidget *parent = 0);
    ~PlacementLocationSelectionView();
    
private:
    Ui::PlacementLocationSelectionView *ui;
};

#endif // PLACEMENTLOCATIONSELECTIONVIEW_H
