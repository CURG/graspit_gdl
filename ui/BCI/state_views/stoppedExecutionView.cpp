#include "stoppedExecutionView.h"
#include "ui_stoppedExecutionView.h"
#include "BCI/bciService.h"

StoppedExecutionView::StoppedExecutionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StoppedExecutionView)
{
    ui->setupUi(this);

    connect(ui->buttonContinueExecution, SIGNAL(clicked()), this, SLOT(onContinueExecutionClicked()));
    connect(ui->buttonStartOver, SIGNAL(clicked()), this, SLOT(onStartOverClicked()));
}

StoppedExecutionView::~StoppedExecutionView()
{
    delete ui;
}


void StoppedExecutionView::onContinueExecutionClicked()
{
    BCIService::getInstance()->emitGoToNextState1();
}


void StoppedExecutionView::onStartOverClicked()
{
    BCIService::getInstance()->emitGoToNextState2();
}
