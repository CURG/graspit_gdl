#ifndef BCISERVICE_H
#define BCISERVICE_H

#include <QObject>

#include "BCI/worldController.h"
#include "Servers/rosRPCZClient.h"

class GraspableBody;
class DrawableFrame;
class GraspPlanningState;
class BCIControlWindow;

using namespace bci_experiment;

class BCIService:public QObject
{

    Q_OBJECT

public:

    void emitGoToNextState1(){emit goToNextState1();}
    void emitGoToNextState2(){emit goToNextState2();}
    void emitGoToPreviousState(){emit goToPreviousState();}


    void emitExec(){emit exec();}
    void emitNext(){emit next();}
    void emitTargetBodyChanged(GraspableBody * b){emit targetBodyChanged(b);}
    void emitResetStateMachine(){resetStateMachine();}
    void emitCursorPosition(double x, double y){emit cursorPosition(x,y);}
    void emitDrawableFrame(DrawableFrame *drawableFrame){emit drawShapes(drawableFrame);}
    void emitRotLat(){emit rotLat();}
    void emitRotLong(){emit rotLong();}

    void emitAnalyzeGrasp(const GraspPlanningState * gps) {emit analyzeGrasp(gps); }
    void emitAnalyzeNextGrasp() {emit analyzeNextGrasp(); }
    void emitRunObjectRecognition(){}

    void emitAnalyzeApproachDir(GraspPlanningState * gs){emit analyzeApproachDir(gs);}

    //called when active planner is updated
    void onPlannerUpdated(){emit plannerUpdated();}

    //ros server calls


    void runObjectRecognition(QObject * callbackReceiver , const char * slot);


    void getCameraOrigin(QObject * callbackReceiver, const char * slot);



    void checkGraspReachability(const GraspPlanningState * state,
                                            QObject * callbackReceiver,
                                            const char * slot);


    void executeGrasp(const GraspPlanningState * gps,
                                  QObject * callbackReceiver,
                                  const char * slot);

    static BCIService* getInstance();

    void init(BCIControlWindow *bciControlWindow);
signals:

    //tell state machine to go to next state
    void goToNextState1();
    //tell state machine to go to second option for next state
    void goToNextState2();
    //tell state machine to return to previous state;
    void goToPreviousState();

    void plannerUpdated();

    void runObjectRecognitionSignal();
    void getCameraOriginSignal();
    void checkGraspReachabilitySignal();


    //! Signal to planner to execute
    void exec();

    //! Signal to planner to go to next state
    void next();

    //! Target body changed
    void targetBodyChanged(GraspableBody * b);

    //! Reset the current state machine
    void resetStateMachine();

    //! signal to give cursor position for binary message cursor
    void cursorPosition(double x, double y);

    //! signal to have something draw 2D shapes
    void drawShapes(DrawableFrame *drawableFrame);

    //! signal to rotate the hand longitudinally around the target object
    void rotLong();

    //! signal to rotate the hand latitudinally around the target object
    void rotLat();

    //! Signal that object recognition system should be rerun
    //void runObjectRecognition();
    void sendString(const QString & s);

    // determine reachability of the grasp at this index
    void analyzeGrasp(const GraspPlanningState * gps);
    void analyzeNextGrasp();
    void analyzeApproachDir(GraspPlanningState * gps);


private:
        static BCIService * bciServiceInstance;
        BCIService();

        RosRPCZClient rosServer;


};

#endif // BCISERVICE_H
