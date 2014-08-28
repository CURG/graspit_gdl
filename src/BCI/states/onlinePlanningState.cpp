#include "BCI/states/onlinePlanningState.h"


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::OnlinePlannerController;


OnlinePlanningState::OnlinePlanningState(BCIControlWindow *_bciControlWindow,QState* parent):
    HandRotationState("OnlinePlanningState", _bciControlWindow, parent),
    bciControlWindow(_bciControlWindow)
{

}

void OnlinePlanningState::onEntry(QEvent *e)
{

}


void OnlinePlanningState::onExit(QEvent *e)
{

}
