#ifndef GRASPSELECTIONVIEW_H
#define GRASPSELECTIONVIEW_H

#include <QWidget>
#include<Inventor/nodes/SoSeparator.h>
#include<Inventor/nodes/SoMaterial.h>
#include<Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoLightModel.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/SoQtRenderArea.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/engines/SoElapsedTime.h>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "body.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/qtWaitingSpinner.h"
#include "BCI/handView.h"

namespace Ui {
class GraspSelectionView;
}

class GraspSelectionView : public QWidget
{
    Q_OBJECT
    
public:
    explicit GraspSelectionView(QWidget *parent = 0);
    void showSelectedGrasp(Hand *hand ,const GraspPlanningState *graspPlanningState);
    ~GraspSelectionView();

public slots:
    void onRefineGrasp();
    void onOk();
    void onBack();


    
private:
     void showSpinner();
     void hideSpinner();

     QtWaitingSpinner* spinner;
     HandView *handView;

    Ui::GraspSelectionView *ui;
};

#endif // GRASPSELECTIONVIEW_H
