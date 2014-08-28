
#include "BCI/states/confirmationState.h"
#include  <QSignalTransition>
#include "world.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/bciService.h"

using bci_experiment::world_element_tools::getWorld;
using bci_experiment::OnlinePlannerController;

ConfirmationState::ConfirmationState(BCIControlWindow *_bciControlWindow,QState* parent):
        State("ConfirmationState", parent),bciControlWindow(_bciControlWindow)
{    
    confirmationView = new ConfirmationView(bciControlWindow->currentFrame);
    confirmationView->hide();
}


void ConfirmationState::onEntry(QEvent *e)
{
    const GraspPlanningState *grasp = OnlinePlannerController::getInstance()->getCurrentGrasp();
    Hand * hand = OnlinePlannerController::getInstance()->getGraspDemoHand();
    confirmationView->setCurrentGrasp(hand,grasp);
    confirmationView->show();
    bciControlWindow->currentState->setText("Confirmation State");
    OnlinePlannerController::getInstance()->setPlannerToPaused();
}


void ConfirmationState::onExit(QEvent * e)
{
    Q_UNUSED(e);
    confirmationView->hide();
}


