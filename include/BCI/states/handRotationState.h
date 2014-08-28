#ifndef HANDROTATIONSTATE_H
#define HANDROTATIONSTATE_H

#include "BCI/state.h"
#include <QSignalTransition>
#include "BCI/onlinePlannerController.h"
#include "BCI/bciControlWindow.h"



class HandRotationState:public State
{
    Q_OBJECT

public:
    HandRotationState(const QString name ,BCIControlWindow *_bciControlWindow,QState* parent = 0);
protected:
    BCIControlWindow *bciControlWindow;

public slots:
    void onRotateHandLat();
    void onRotateHandLong();
    void onHandRotationStateEntry();
};

#endif // HANDROTATIONSTATE_H
