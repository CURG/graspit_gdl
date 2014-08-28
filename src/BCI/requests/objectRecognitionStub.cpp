#include "BCI/requests/objectRecognitionStub.h"
#include "graspable_object.pb.h"
#include "robot.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/bciService.h"

using bci_experiment::WorldController;
using bci_experiment::OnlinePlannerController;

ObjectRecognitionStub::ObjectRecognitionStub(rpcz::rpc_channel * channel):
    objectRecognition_stub(channel, "ObjectRecognitionService", true)
{
    connect(
        this,
        SIGNAL(addToWorld(const QString, const QString, const QString )),
        OnlinePlannerController::getInstance(),
        SLOT(addToWorld(const QString , const QString, const QString )));

    connect(
        this,
        SIGNAL(clearGB()),
        OnlinePlannerController::getInstance(),
        SLOT(clearObjects()));
}

void ObjectRecognitionStub::sendRequestImpl()
{
    objectRecognition_stub.run(request,&response, _rpc,rpcz::new_callback<ObjectRecognitionStub>(this, &ObjectRecognitionStub::callback));
}

void ObjectRecognitionStub::callbackImpl()
{
    // ask bodies to be cleared
    emit clearGB();
    // clear any existing graspable bodies
    while(graspItGUI->getIVmgr()->getWorld()->getNumGB())
        usleep(10000);

    // add each of the new bodies
    std::for_each(response.foundobjects().begin(),
                  response.foundobjects().end(),
                  boost::bind(&ObjectRecognitionStub::addObject, this, _1));
}


void ObjectRecognitionStub::addObject(GraspableObject object)
{
    QString  modelName(QString::fromStdString(object.model()) + ".xml");
    QString objectName(QString::fromStdString(object.name()));
    //Body b = addToWorld(objectName);

    //b.setName(objectName);

    transf object_pose = transf(
                Quaternion(
                    object.pose().orientation().w(),
                    object.pose().orientation().x(),
                    object.pose().orientation().y(),
                    object.pose().orientation().z()),
                vec3(
                    object.pose().position().x()*1000.0,
                    object.pose().position().y()*1000.0,
                    object.pose().position().z()*1000.0
                    ));
    std::stringstream s;
    s << object_pose;
    QString stringPose(QString::fromStdString(s.str()));
    emit addToWorld(modelName, objectName, stringPose);
}



