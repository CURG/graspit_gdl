#ifndef GRASPSELECTIONSTATE_H
#define GRASPSELECTIONSTATE_H


#include "BCI/state.h"

#include <QSignalTransition>
#include "debug.h"
#include "BCI/bciControlWindow.h"
#include "BCI/state_views/graspSelectionView.h"

class GraspSelectionState:public State
{
    Q_OBJECT

public:
    GraspSelectionState(BCIControlWindow *_bciControlWindow, QState* parent = 0 );


    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);

private:
    BCIControlWindow *bciControlWindow;
    GraspSelectionView *graspSelectionView;

public slots:
    void onNext();
    void onPlannerUpdated();
};


#endif // GRASPSELECTIONSTATE_H
