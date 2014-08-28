
#include "BCI/requests/executeGraspStub.h"

#include "searchState.h"
#include "body.h"
#include "robot.h"
#include <QFileInfo>

ExecuteGraspStub::ExecuteGraspStub(rpcz::rpc_channel * channel)
    :executeGrasp_stub(channel, "ExecuteGraspService")
{
}

void ExecuteGraspStub::buildRequest(const GraspPlanningState * gps)
{

    request.clear_grasp();

    request.mutable_grasp()->set_graspid(gps->getAttribute("graspId"));

    request.mutable_grasp()->mutable_object()->set_name(gps->getObject()->getName());
    QString filename = gps->getObject()->getFilename();
    QFileInfo modelFileInfo(filename);

    request.mutable_grasp()->mutable_object()->set_model(modelFileInfo.baseName());

    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_position()->set_x(gps->getObject()->getTran().translation().x());
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_position()->set_y(gps->getObject()->getTran().translation().y());
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_position()->set_z(gps->getObject()->getTran().translation().z());
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_orientation()->set_w(gps->getObject()->getTran().rotation().w);
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_orientation()->set_x(gps->getObject()->getTran().rotation().x);
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_orientation()->set_y(gps->getObject()->getTran().rotation().y);
    request.mutable_grasp()->mutable_object()->mutable_pose()->mutable_orientation()->set_z(gps->getObject()->getTran().rotation().z);

    request.mutable_grasp()->set_epsilon_quality(gps->getEpsilonQuality());
    request.mutable_grasp()->set_volume_quality(gps->getVolume());

    for(int i = 0; i < gps->getQualityMeasures()->size(); i++)
    {
        request.mutable_grasp()->add_secondary_qualities(gps->getQualityMeasures()->at(i));
    }


    double dof[gps->getHand()->getNumDOF()];
    const_cast<GraspPlanningState *>(gps)->getPosture()->getHandDOF(dof);
    for(int i = 0; i < gps->getHand()->getNumDOF(); ++i)
    {
        request.mutable_grasp()->mutable_pre_grasp_hand_state()->add_hand_dof(dof[i]);
        request.mutable_grasp()->mutable_final_grasp_hand_state()->add_hand_dof(dof[i]);
    }


    transf finalHandTransform = gps->readPosition()->getCoreTran();

    float tx = finalHandTransform.translation().x();
    float ty = finalHandTransform.translation().y();
    float tz = finalHandTransform.translation().z();
    float rw = finalHandTransform.rotation().w;
    float rx = finalHandTransform.rotation().x;
    float ry = finalHandTransform.rotation().y;
    float rz = finalHandTransform.rotation().z;

    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_x(tx);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_y(ty);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_z(tz);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_w(rw);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_x(rx);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_y(ry);
    request.mutable_grasp()->mutable_final_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_z(rz);

    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_x(tx);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_y(ty);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_position()->set_z(tz);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_w(rw);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_x(rx);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_y(ry);
    request.mutable_grasp()->mutable_pre_grasp_hand_state()->mutable_hand_pose()->mutable_orientation()->set_z(rz);
}



void ExecuteGraspStub::sendRequestImpl()
{
    executeGrasp_stub.run(request,&response, _rpc,rpcz::new_callback<ExecuteGraspStub>(this, &ExecuteGraspStub::callback));
}

void ExecuteGraspStub::callbackImpl()
{
    std::cout << "hello world I am a callback!!!!"<< std::endl;
}
