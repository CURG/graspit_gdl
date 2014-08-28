#ifndef EXECUTEGRASPSTUB_H
#define EXECUTEGRASPSTUB_H

#include "rpcz/rpc_channel.hpp"
#include "rpcz/rpc_channel_impl.hpp"

#include "execute_grasp.pb.h"
#include "execute_grasp.rpcz.h"

#include "BCI/requests/requestStub.h"

class GraspPlanningState;

class ExecuteGraspStub : public Request
{
    Q_OBJECT
public:
    ExecuteGraspStub(rpcz::rpc_channel * channel);

    void buildRequest(const GraspPlanningState * gps);

protected:
    virtual void sendRequestImpl();
    virtual void callbackImpl();

private:
    graspit_rpcz::ExecuteGraspService_Stub executeGrasp_stub;
    graspit_rpcz::ExecuteGraspRequest request;
    graspit_rpcz::ExecuteGraspResponse response;
};


#endif // EXECUTEGRASPSTUB_H
