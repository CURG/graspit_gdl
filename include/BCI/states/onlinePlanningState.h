#ifndef ONLINESELECTIONSTATE_H
#define ONLINESELECTIONSTATE_H


#include "BCI/states/handRotationState.h"
#include "BCI/bciControlWindow.h"

class OnlinePlanningState : public HandRotationState
{
public:
    OnlinePlanningState(BCIControlWindow *_bciControlWindow, QState* parent = 0);
    virtual void onEntry(QEvent * e);
    virtual void onExit(QEvent * e);

    BCIControlWindow *bciControlWindow;


};

#endif // ONLINESELECTIONSTATE_H
