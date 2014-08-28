#ifndef BCIPREVIEWVIEW_H
#define BCIPREVIEWVIEW_H

#include <QWidget>

#include<Inventor/Qt/SoQtRenderArea.h>
#include<Inventor/nodes/SoSeparator.h>
#include<Inventor/nodes/SoMaterial.h>
#include<Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoLightModel.h>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "body.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"

namespace Ui {
class BCIPreviewView;
}

class BCIPreviewView : public QWidget
{
    Q_OBJECT
    
public:
    SoSeparator *IVHandGeometry;
    SoSeparator * IVObjectGeometry;

    explicit BCIPreviewView(QWidget *parent = 0);

    void showBestGrasp();
    ~BCIPreviewView();
    
private:
    Ui::BCIPreviewView *ui;
    SoQtRenderArea *soQtRenderArea;
    void setupIVHandGeometry(Hand * h);
};

#endif // BCIPREVIEWVIEW_H
