#include "BCI/states/objectSelectionState.h"
#include "BCI/bciService.h"
#include "BCI/state_views/objectSelectionView.h"


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::OnlinePlannerController;
using bci_experiment::WorldController;


ObjectSelectionState::ObjectSelectionState(BCIControlWindow *_bciControlWindow,
                                           QState* parent):
    State("ObjectSelectionState", parent),
    bciControlWindow(_bciControlWindow)
{
    objectSelectionView = new ObjectSelectionView(this,bciControlWindow->currentFrame);
    objectSelectionView->hide();
    this->addSelfTransition(getWorld(), SIGNAL(numElementsChanged()), this, SLOT(onNewObjectFound()));
}


void ObjectSelectionState::onEntry(QEvent *e)
{
    objectSelectionView->show();
    WorldController::getInstance()->highlightAllBodies();
    GraspableBody *currentTarget = OnlinePlannerController::getInstance()->getCurrentTarget();
    WorldController::getInstance()->highlightCurrentBody(currentTarget);
    bciControlWindow->currentState->setText("Object Selection State");
}


void ObjectSelectionState::onExit(QEvent *e)
{
    WorldController::getInstance()->unhighlightAllBodies();
    objectSelectionView->hide();
}


void ObjectSelectionState::onNext()
{
    GraspableBody *newTarget = OnlinePlannerController::getInstance()->incrementCurrentTarget();
    WorldController::getInstance()->highlightCurrentBody(newTarget);
}


void ObjectSelectionState::onSelect()
{
    BCIService::getInstance()->emitGoToNextState1();
}

void ObjectSelectionState::onNewObjectFound()
{
    GraspableBody *currentTarget = OnlinePlannerController::getInstance()->getCurrentTarget();

    if(currentTarget)
    {
        WorldController::getInstance()->highlightCurrentBody(currentTarget);
    }

}
