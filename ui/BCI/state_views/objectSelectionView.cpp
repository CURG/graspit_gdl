#include "objectSelectionView.h"
#include "ui_objectSelectionView.h"
#include "debug.h"

ObjectSelectionView::ObjectSelectionView(ObjectSelectionState *state,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ObjectSelectionView)
{
    ui->setupUi(this);

    connect(ui->next, SIGNAL(clicked()), state, SLOT(onNext()));
    connect(ui->select, SIGNAL(clicked()), state, SLOT(onSelect()));

}






ObjectSelectionView::~ObjectSelectionView()
{
    delete ui;
}
