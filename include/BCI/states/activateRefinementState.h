#ifndef STARTSTATE_H
#define STARTSTATE_H

#include "BCI/states/handRotationState.h"

#include <QSignalTransition>
#include "debug.h"
#include "BCI/bciControlWindow.h"
#include "BCI/state_views/activeRefinementView.h"

class ActivateRefinementState: public HandRotationState
{
    Q_OBJECT

public:
    ActivateRefinementState(BCIControlWindow *_bciControlWindow, QState* parent = 0 );

public slots:
    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);
    virtual void onPlannerUpdated(QEvent *e = NULL);

protected:
    ActiveRefinementView *activeRefinementView;


};


#endif // STARTSTATE_H


