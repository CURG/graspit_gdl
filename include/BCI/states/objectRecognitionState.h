#ifndef OBJECTRECOGNITIONSTATE_H
#define OBJECTRECOGNITIONSTATE_H

#include "BCI/state_views/objectRecognitionView.h"
#include "BCI/bciControlWindow.h"
#include "BCI/state.h"

class ObjectRecognitionState : public State
{
public:
    ObjectRecognitionState(BCIControlWindow *_bciControlWindow,QState* parent=0);

    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);

private:
   ObjectRecognitionView *objectRecognitionView;
   BCIControlWindow *bciControlWindow;
};

#endif // OBJECTRECOGNITIONSTATE_H
