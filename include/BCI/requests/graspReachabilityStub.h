#ifndef GRASPREACHABILITYSERVICEWRAPPER_H
#define GRASPREACHABILITYSERVICEWRAPPER_H

#include "rpcz/rpc_channel.hpp"
#include "rpcz/rpc_channel_impl.hpp"
#include "check_grasp_reachability.pb.h"
#include "check_grasp_reachability.rpcz.h"

#include "BCI/requests/requestStub.h"

//class GraspReachabilityStub : public RequestType
//{
//public:
//    GraspReachabilityStub(rpcz::rpc_channel * channel);

//protected:
//    virtual void callback();
//};

class GraspPlanningState;

class GraspReachabilityStub : public Request
{
    Q_OBJECT
public:
    GraspReachabilityStub(rpcz::rpc_channel * channel);
    void buildRequest(const GraspPlanningState * gps);

protected:
    virtual void sendRequestImpl();
    virtual void callbackImpl();

private:
    graspit_rpcz::CheckGraspReachabilityService_Stub graspReachability_stub;
    graspit_rpcz::CheckGraspReachabilityRequest request;
    graspit_rpcz::CheckGraspReachabilityResponse response;
};

#endif // GRASPREACHABILITYSERVICEWRAPPER_H
