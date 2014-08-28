#include "BCI/states/activateRefinementState.h"
#include "BCI/bciService.h"

using bci_experiment::OnlinePlannerController;
using bci_experiment::world_element_tools::getWorld;
using bci_experiment::WorldController;

ActivateRefinementState::ActivateRefinementState(BCIControlWindow *_bciControlWindow,QState* parent):
    HandRotationState("ActivateRefinementState",_bciControlWindow, parent)
{
    addSelfTransition(BCIService::getInstance(),SIGNAL(plannerUpdated()), this, SLOT(onPlannerUpdated()));
    connect(this, SIGNAL(entered()),OnlinePlannerController::getInstance(), SLOT(setPlannerToRunning()));
    connect(this, SIGNAL(exited()), OnlinePlannerController::getInstance(), SLOT(setPlannerToPaused()));

    activeRefinementView = new ActiveRefinementView(bciControlWindow->currentFrame);
    activeRefinementView->hide();
}


void ActivateRefinementState::onEntry(QEvent *e)
{
    activeRefinementView->show();
    bciControlWindow->currentState->setText("Active Refinement State");
    onPlannerUpdated();
}


void ActivateRefinementState::onExit(QEvent *e)
{
    activeRefinementView->hide();
}




void ActivateRefinementState::onPlannerUpdated(QEvent * e)
{
    const GraspPlanningState *bestGrasp = OnlinePlannerController::getInstance()->getGrasp(0);
    Hand *hand = OnlinePlannerController::getInstance()->getGraspDemoHand();

    if(bestGrasp)
    {
        activeRefinementView->showSelectedGrasp(hand,bestGrasp);
        QString graspID;
        bciControlWindow->currentState->setText("Planner Updated: " + graspID.setNum(bestGrasp->getAttribute("graspId")) );
    }
    OnlinePlannerController::getInstance()->analyzeNextGrasp();
}


