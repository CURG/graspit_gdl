#include "placementLocationSelectionView.h"
#include "ui_placementLocationSelectionView.h"

PlacementLocationSelectionView::PlacementLocationSelectionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlacementLocationSelectionView)
{
    ui->setupUi(this);
}

PlacementLocationSelectionView::~PlacementLocationSelectionView()
{
    delete ui;
}
