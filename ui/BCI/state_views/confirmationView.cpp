#include "confirmationView.h"
#include "ui_confirmationView.h"
#include "BCI/bciService.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "body.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/qtWaitingSpinner.h"
#include "BCI/handView.h"

ConfirmationView::ConfirmationView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConfirmationView)
{
    ui->setupUi(this);


    connect(ui->buttonOk, SIGNAL(clicked()), this, SLOT(onOk()));
    connect(ui->buttonBack, SIGNAL(clicked()), this, SLOT(onBack()));

    SoQtExaminerViewer *mainViewer = graspItGUI->getIVmgr()->getViewer();
    Hand * h = OnlinePlannerController::getInstance()->getGraspDemoHand();
    QFrame *parentWindow = this->ui->previewFrame;
    QString viewName = QString("current best grasp");
    handView = new HandView(mainViewer,h,*parentWindow,viewName);

}


void ConfirmationView::setCurrentGrasp(Hand *hand, const GraspPlanningState *graspPlanningState)
{

    if(graspPlanningState)
    {
        handView->update(*graspPlanningState, *hand);
    }
}

ConfirmationView::~ConfirmationView()
{
    delete ui;
}


void ConfirmationView::onOk()
{
    BCIService::getInstance()->emitGoToNextState1();
}

void ConfirmationView::onBack()
{
    BCIService::getInstance()->emitGoToNextState2();
}
