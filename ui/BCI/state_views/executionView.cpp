#include "executionView.h"
#include "ui_executionView.h"
#include "BCI/bciService.h"

ExecutionView::ExecutionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ExecutionView)
{
    ui->setupUi(this);
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onStopButtonPressed()));
}

ExecutionView::~ExecutionView()
{
    delete ui;
}


void ExecutionView::onStopButtonPressed()
{
    BCIService::getInstance()->emitGoToNextState1();
}

