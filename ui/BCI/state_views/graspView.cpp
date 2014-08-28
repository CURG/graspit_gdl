#include "graspView.h"
#include "ui_graspView.h"
#include "body.h"

GraspView::GraspView( QWidget *parent) :
    QWidget(parent),

    ui(new Ui::GraspView)
{
    ui->setupUi(this);
}

GraspView::~GraspView()
{
    delete ui;
}
