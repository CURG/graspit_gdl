
#ifndef OBJECT_RECOGNITION_STUB_H
#define OBJECT_RECOGNITION_STUB_H

#include "BCI/requests/requestStub.h"
#include "run_recognition.rpcz.h"
#include "run_recognition.pb.h"
#include "rpcz/rpc_channel.hpp"
#include "rpcz/rpc_channel_impl.hpp"
#include <QString>
#include "matvec3D.h"
using namespace graspit_rpcz;
class Body;

class ObjectRecognitionStub: public Request
{
    Q_OBJECT
public:
    ObjectRecognitionStub(rpcz::rpc_channel * channel);

signals:
    void addToWorld(const QString model_filename, const QString object_name, const QString object_pose);
    void clearGB();

protected:
    virtual void sendRequestImpl();
    virtual void callbackImpl();

private:
    graspit_rpcz::ObjectRecognitionService_Stub objectRecognition_stub;
    graspit_rpcz::ObjectRecognitionRequest request;
    graspit_rpcz::ObjectRecognitionResponse response;

    void addObject(GraspableObject object);

};

#endif // OBJECT_RECOGNITION_STUB_H
