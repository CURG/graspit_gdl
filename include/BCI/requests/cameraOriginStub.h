#ifndef CAMERAORIGINSERVICE_H
#define CAMERAORIGINSERVICE_H

#include "rpcz/rpc_channel.hpp"
#include "rpcz/rpc_channel_impl.hpp"

#include "get_camera_origin.pb.h"
#include "get_camera_origin.rpcz.h"

#include "BCI/requests/requestStub.h"

class CameraOriginStub : public Request
{
    Q_OBJECT
public:
    CameraOriginStub(rpcz::rpc_channel * channel);

protected:
    virtual void sendRequestImpl();
    virtual void callbackImpl();

private:
    graspit_rpcz::CameraOriginService_Stub cameraOrigin_stub;
    graspit_rpcz::CameraOriginRequest request;
    graspit_rpcz::CameraOriginResponse response;
};

#endif // CAMERAORIGINSERVICE_H
