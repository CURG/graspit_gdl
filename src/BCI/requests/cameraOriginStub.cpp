#include "BCI/requests/cameraOriginStub.h"

#include "matvec3D.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include <Inventor/nodes/SoCamera.h>

CameraOriginStub::CameraOriginStub(rpcz::rpc_channel * channel)
    :cameraOrigin_stub(channel, "CameraOriginService")
{
}


void CameraOriginStub::sendRequestImpl()
{
    cameraOrigin_stub.run(request,&response, _rpc,rpcz::new_callback<CameraOriginStub>(this, &CameraOriginStub::callback));
}

void CameraOriginStub::callbackImpl()
{
    std::cout << "hello world I am a callback!!!!"<< std::endl;
    float x = response.cameraorigin().position().x();
    float y = response.cameraorigin().position().y();
    float z = response.cameraorigin().position().z();
    transf t(mat3::IDENTITY, vec3(x,y,z));
    graspItGUI->getIVmgr()->setCameraTransf(t);
    graspItGUI->getIVmgr()->getViewer()->getCamera()->pointAt(SbVec3f(0,0,0), SbVec3f(0,0,1));
}
