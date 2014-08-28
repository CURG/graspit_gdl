
#include "BCI/BCIStateMachine.h"

#include "BCI/states/objectSelectionState.h"
#include "BCI/states/graspSelectionState.h"
#include "BCI/states/placementLocationSelectionState.h"
#include "BCI/states/confirmationState.h"
#include "BCI/states/onlinePlanningState.h"
#include "BCI/states/activateRefinementState.h"
#include "BCI/states/objectRecognitionState.h"
#include "BCI/states/executionState.h"
#include "BCI/states/stoppedExecutionState.h"

#include "BCI/bciService.h"

BCIStateMachine::BCIStateMachine(BCIControlWindow *_bciControlWindow, BCIService *_bciService):
    bciControlWindow(_bciControlWindow),
    bciService(_bciService)
{
    ObjectRecognitionState *objectRecognitionState = new ObjectRecognitionState(bciControlWindow);
    ObjectSelectionState *objectSelectionState = new ObjectSelectionState(bciControlWindow);
    GraspSelectionState *initialGraspSelectionState = new GraspSelectionState(bciControlWindow);
    OnlinePlanningState * onlinePlanningState = new OnlinePlanningState(bciControlWindow);
    ActivateRefinementState *activateRefinementState = new ActivateRefinementState(bciControlWindow);
    GraspSelectionState *finalGraspSelectionState = new GraspSelectionState(bciControlWindow);
    ConfirmationState *confirmationState = new ConfirmationState(bciControlWindow);
    ExecutionState *executionState = new ExecutionState(bciControlWindow);
    StoppedExecutionState *stoppedExecutionState = new StoppedExecutionState(bciControlWindow);

    //Add all state transistions that are not self transitions----------------------------
    objectRecognitionState->addTransition(bciService,SIGNAL(goToNextState1()), objectSelectionState);

    objectSelectionState->addTransition(bciService,SIGNAL(goToNextState1()), initialGraspSelectionState);

    initialGraspSelectionState->addTransition(bciService, SIGNAL(goToNextState2()), activateRefinementState);
    initialGraspSelectionState->addTransition(bciService, SIGNAL(goToPreviousState()), objectSelectionState);
    initialGraspSelectionState->addTransition(bciService, SIGNAL(goToNextState1()), confirmationState);

    activateRefinementState->addTransition(bciService, SIGNAL(goToNextState1()), confirmationState);

    finalGraspSelectionState->addTransition(bciService, SIGNAL(goToNextState1()),confirmationState);

    onlinePlanningState->addTransition(bciService, SIGNAL(goToNextState1()), finalGraspSelectionState);

    confirmationState->addTransition(bciService, SIGNAL(goToNextState1()), executionState);
    confirmationState->addTransition(bciService, SIGNAL(goToNextState2()), initialGraspSelectionState);

    executionState->addTransition(bciService, SIGNAL(goToNextState1()), stoppedExecutionState);

    stoppedExecutionState->addTransition(bciService, SIGNAL(goToNextState1()), executionState);
    stoppedExecutionState->addTransition(bciService, SIGNAL(goToNextState2()), objectRecognitionState);

    stateMachine.addState(objectRecognitionState);
    stateMachine.addState(objectSelectionState);
    stateMachine.addState(initialGraspSelectionState);
    stateMachine.addState(activateRefinementState);
    stateMachine.addState(finalGraspSelectionState);
    stateMachine.addState(onlinePlanningState);
    stateMachine.addState(confirmationState);
    stateMachine.addState(executionState);
    stateMachine.addState(stoppedExecutionState);

    stateMachine.setInitialState(objectRecognitionState);
}

void BCIStateMachine::start()
{        
    stateMachine.start();
    bciControlWindow->currentFrame->show();
}

