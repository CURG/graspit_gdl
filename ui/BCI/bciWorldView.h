#ifndef BCIWORLDVIEW_H
#define BCIWORLDVIEW_H

#include <QWidget>

#include<Inventor/Qt/SoQtRenderArea.h>
#include<Inventor/nodes/SoSeparator.h>
#include<Inventor/nodes/SoMaterial.h>
#include<Inventor/nodes/SoTransformSeparator.h>

#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include<Inventor/nodes/SoLightModel.h>

#include "graspitGUI.h"
#include "ivmgr.h"

namespace Ui {
class BCIWorldView;
}

class BCIWorldView : public QWidget
{
    Q_OBJECT
    
public:
    explicit BCIWorldView(QWidget *parent = 0);
   ~BCIWorldView();
public slots:
    void drawGuides();

    
private:
    Ui::BCIWorldView *ui;
};

#endif // BCIWORLDVIEW_H
