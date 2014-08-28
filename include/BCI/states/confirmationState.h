#ifndef CONFIRMATIONSTATE_H
#define CONFIRMATIONSTATE_H


#include "BCI/bciControlWindow.h"
#include "BCI/state.h"
#include "BCI/state_views/confirmationView.h"
class ConfirmationState:public State
{
    Q_OBJECT

public:
    ConfirmationState(BCIControlWindow *_bciControlWindow, QState* parent = 0 );

     virtual void onEntry(QEvent *e);
     virtual void onExit(QEvent *e);


private:
    ConfirmationView* confirmationView;
    BCIControlWindow *bciControlWindow;
};


#endif // CONFIRMATIONSTATE_H
