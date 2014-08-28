#ifndef OBJECTSELECTIONVIEW_H
#define OBJECTSELECTIONVIEW_H

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

#include "BCI/states/objectSelectionState.h"

namespace Ui {
class ObjectSelectionView;
}

class ObjectSelectionView : public QWidget
{
    Q_OBJECT
    
public:
    explicit ObjectSelectionView(ObjectSelectionState *state, QWidget *parent = 0);
    ~ObjectSelectionView();
    
private:

    Ui::ObjectSelectionView *ui;

};

#endif // OBJECTSELECTIONVIEW_H
