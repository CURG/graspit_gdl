#include "BCI/states/graspSelectionState.h"
#include "BCI/bciService.h"
#include "BCI/onlinePlannerController.h"

using bci_experiment::OnlinePlannerController;


GraspSelectionState::GraspSelectionState(BCIControlWindow *_bciControlWindow,QState* parent):
    State("GraspSelectionState", parent),bciControlWindow(_bciControlWindow)
{
    /* What should next do?

      It should update the online planner controller's idea of the *CURRENT* grasp

      This has two control consequences - If the grasp is sent to the robot, this is the
      grasp that is sent. If the planner is started, this grasp is used as approach
      direction of the seed.

      It also affects the visualization - This should change which grasp is shown by the demonstration hand
      and which grasp is shown most prominantly in any grasp preview pane.
    */

    addSelfTransition(BCIService::getInstance(),SIGNAL(next()), this, SLOT(onNext()));
    addSelfTransition(BCIService::getInstance(),SIGNAL(plannerUpdated()), this, SLOT(onPlannerUpdated()));
    connect(this, SIGNAL(entered()), OnlinePlannerController::getInstance(), SLOT(setPlannerToReady()));
    graspSelectionView = new GraspSelectionView(bciControlWindow->currentFrame);
    graspSelectionView->hide();

}


void GraspSelectionState::onEntry(QEvent *e)
{

    graspSelectionView->show();
    bciControlWindow->currentState->setText("Grasp Selection State");

    //loads grasps from the database
    //OnlinePlannerController::getInstance()->setPlannerToReady();
    //called so that view will show best grasp from database
    onPlannerUpdated();
    OnlinePlannerController::getInstance()->setPlannerToRunning();
}


void GraspSelectionState::onExit(QEvent *e)
{
    graspSelectionView->hide();
}

void GraspSelectionState::onNext()
{
    OnlinePlannerController::getInstance()->incrementGraspIndex();
}

void GraspSelectionState::onPlannerUpdated()
{
    const GraspPlanningState *bestGrasp = OnlinePlannerController::getInstance()->getGrasp(0);
    Hand *hand = OnlinePlannerController::getInstance()->getGraspDemoHand();

    if(bestGrasp)
    {
        graspSelectionView->showSelectedGrasp(hand,bestGrasp);
        QString graspID;
        bciControlWindow->currentState->setText("Planner Updated: " + graspID.setNum(bestGrasp->getAttribute("graspId")) );
    }
    OnlinePlannerController::getInstance()->analyzeNextGrasp();

}



