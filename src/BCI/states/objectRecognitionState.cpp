#include "BCI/states/objectRecognitionState.h"
#include "BCI/onlinePlannerController.h"

using bci_experiment::OnlinePlannerController;

ObjectRecognitionState::ObjectRecognitionState(BCIControlWindow *_bciControlWindow,QState* parent):
        State("ObjectRecognitionState", parent),bciControlWindow(_bciControlWindow)
{
     objectRecognitionView = new ObjectRecognitionView(bciControlWindow->currentFrame);
     objectRecognitionView->hide();
}


void ObjectRecognitionState::onEntry(QEvent *e)
  {
    objectRecognitionView->show();
    bciControlWindow->currentState->setText("Object Recognition State");
    BCIService::getInstance()->getCameraOrigin(NULL,NULL);
    BCIService::getInstance()->runObjectRecognition(NULL,NULL);
    if(OnlinePlannerController::getInstance()->hasRecognizedObjects())
    {
        BCIService::getInstance()->emitGoToNextState1();
    }
}

void ObjectRecognitionState::onExit(QEvent *e)
{
     objectRecognitionView->hide();
}

