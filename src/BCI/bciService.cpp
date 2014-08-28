#include "BCI/bciService.h"
#include "BCI/BCIStateMachine.h"

BCIService * BCIService::bciServiceInstance = NULL;

BCIService* BCIService::getInstance()
{
    if(!bciServiceInstance)
    {
        bciServiceInstance = new BCIService();
    }

    return bciServiceInstance;
}

BCIService::BCIService()
{

}

void BCIService::init(BCIControlWindow *bciControlWindow)
{
    //builds and starts a qtStateMachine
    BCIStateMachine *bciStateMachine = new BCIStateMachine(bciControlWindow,this);
    bciStateMachine->start();
}




void BCIService::runObjectRecognition(QObject * callbackReceiver ,
                                      const char * slot)
{
    rosServer.runObjectRecognition(callbackReceiver, slot);
}

void BCIService::getCameraOrigin(QObject * callbackReceiver, const char * slot)
{
    rosServer.getCameraOrigin(callbackReceiver, slot);
}


void BCIService::checkGraspReachability(const GraspPlanningState * state,
                                        QObject * callbackReceiver,
                                        const char * slot)
{
    rosServer.checkGraspReachability(state, callbackReceiver, slot);
}
 
void BCIService::executeGrasp(const GraspPlanningState * gps, 
			      QObject * callbackReceiver,
			      const char * slot)
{
    rosServer.executeGrasp(gps);
}


