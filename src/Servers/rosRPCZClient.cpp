#include "rosRPCZClient.h"

#include "BCI/bciService.h"
#include "EGPlanner/searchState.h"


RosRPCZClient::RosRPCZClient():
    _application(),
    graspReachabilityStub(_application.create_rpc_channel("tcp://localhost:5561")),
    objectRecognitionStub(_application.create_rpc_channel("tcp://localhost:5561")),
    cameraOriginStub(_application.create_rpc_channel("tcp://localhost:5561")),
    executeGraspStub(_application.create_rpc_channel("tcp://localhost:5561"))
{

}


void RosRPCZClient::runObjectRecognition(QObject * callbackReceiver, const char * slot)
{
    objectRecognitionStub.sendRequest(callbackReceiver, slot);
}

void RosRPCZClient::getCameraOrigin(QObject * callbackReceiver, const char *slot)
{
    cameraOriginStub.sendRequest(callbackReceiver, slot);
}

void RosRPCZClient::checkGraspReachability(const GraspPlanningState * gps, QObject * callbackReceiver, const char * slot)
{    
    graspReachabilityStub.buildRequest(gps);
    graspReachabilityStub.sendRequest(callbackReceiver, slot);
}


void RosRPCZClient::executeGrasp(const GraspPlanningState * gps, QObject * callbackReceiver, const char * slot)
{
  executeGraspStub.buildRequest(gps);
  executeGraspStub.sendRequest(callbackReceiver, slot);
}




