#include "BCI/states/handRotationState.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/bciService.h"


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::OnlinePlannerController;
using bci_experiment::WorldController;


HandRotationState::HandRotationState(QString name , BCIControlWindow *_bciControlWindow,QState* parent):
      State(name, parent),bciControlWindow(_bciControlWindow)
{
    addSelfTransition(BCIService::getInstance(),SIGNAL(rotLat()), this, SLOT(onRotateHandLat()));
    addSelfTransition(BCIService::getInstance(),SIGNAL(rotLong()), this, SLOT(onRotateHandLong()));
    addSelfTransition(this,SIGNAL(entered()), this, SLOT(onHandRotationStateEntry()));
    DBGA("HandRotationState");
}


void HandRotationState::onRotateHandLong()
{
    OnlinePlannerController::getInstance()->rotateHandLong();
}

void HandRotationState::onRotateHandLat()
{
    OnlinePlannerController::getInstance()->rotateHandLat();
}

void HandRotationState::onHandRotationStateEntry()
{
    OnlinePlannerController::getInstance()->alignHand();
}
