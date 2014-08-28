#ifndef ROSRPCZCLIENT_H
#define ROSRPCZCLIENT_H

#include "rpcz/rpcz.hpp"

#include <boost/shared_ptr.hpp>
#include "BCI/requests/objectRecognitionStub.h"
#include "BCI/requests/cameraOriginStub.h"
#include "BCI/requests/graspReachabilityStub.h"
#include "BCI/requests/executeGraspStub.h"

using namespace graspit_rpcz;
class GraspPlanningState;
class RosRPCZClient
{

public:
    RosRPCZClient();
    void runObjectRecognition(QObject * callbackReceiver = NULL, const char * slot = NULL);
    void getCameraOrigin(QObject * callbackReceiver = NULL, const char * slot = NULL);
    void checkGraspReachability(const GraspPlanningState * gps, QObject * callbackReceiver = NULL, const char * slot = NULL);
    void executeGrasp(const GraspPlanningState * gps, QObject * callbackReceiver = NULL, const char * slot = NULL);

private:
    rpcz::application _application;

    GraspReachabilityStub graspReachabilityStub;
    ObjectRecognitionStub objectRecognitionStub;
    CameraOriginStub cameraOriginStub;
    ExecuteGraspStub executeGraspStub;

};

#endif // ROSRPCZCLIENT_H
