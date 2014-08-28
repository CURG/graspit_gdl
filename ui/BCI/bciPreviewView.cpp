#include "bciPreviewView.h"
#include "ui_bciPreviewView.h"
#include "debug.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/SoQtRenderArea.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/engines/SoElapsedTime.h>



BCIPreviewView::BCIPreviewView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BCIPreviewView)
{
    ui->setupUi(this);
}

void BCIPreviewView::showBestGrasp()
{
    // create scene root
    SoSeparator *root = new SoSeparator;
    root->setName("BestGraspRoot");
    root->ref();

//<<<<<<< HEAD
    // camera
    SoQtExaminerViewer *mainViewer_ =  graspItGUI->getIVmgr()->getViewer();
    SoCamera *camera = static_cast<SoCamera *>(mainViewer_->getCamera()->copy());
    root->addChild(camera);
//=======
//    SoQtRenderArea * renderArea = new SoQtRenderArea(this, " ",true);
//    SoSeparator * bciWorldViewRoot = new SoSeparator;
//    SoMaterial * soMaterial = new SoMaterial;
//    SoTransformSeparator *lightSep = new SoTransformSeparator;
//    SoRotation *lightDir = new SoRotation;
//    SoLightModel * lightModel = new SoLightModel;

//    SoNode *ivRoot = graspItGUI->getIVmgr()->getViewer()->getSceneGraph();

//    soMaterial->diffuseColor.setValue(1,0,0);

//    lightDir->rotation.connectFrom(&graspItGUI->getIVmgr()->getViewer()->getCamera()->orientation);
//    lightSep->addChild(lightDir);
//    lightSep->addChild(graspItGUI->getIVmgr()->getViewer()->getHeadlight());

//    lightModel->model=SoLightModel::PHONG;

//    bciWorldViewRoot->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
//    bciWorldViewRoot->addChild(lightSep);
//    bciWorldViewRoot->addChild(lightModel);
//    bciWorldViewRoot->addChild(ivRoot);

//    renderArea->setSceneGraph(bciWorldViewRoot);
//    renderArea->setBackgroundColor(SbColor(1,1,1));
//    renderArea->scheduleRedraw();
//    renderArea->render();
//    renderArea->show();

////    SoQtRenderArea *soQtRenderArea = new SoQtRenderArea(this, " ",true);
////    SoQtExaminerViewer *mainViewer_ =  graspItGUI->getIVmgr()->getViewer();
////    SoSeparator *ivRoot = new SoSeparator();
////    SoCamera *camera_ = static_cast<SoCamera *>(mainViewer_->getCamera()->copy());
////    SoRotation *lightDir = new SoRotation;
////    SoTransformSeparator *lightSep = new SoTransformSeparator;
////    Hand *currentHand =   graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
//>>>>>>> a56deb358ef66d31a0968a9b77679eb1046725ad

    // light
    root->addChild(new SoDirectionalLight);

    // material
    SoMaterial *material = new SoMaterial;
    material->diffuseColor.setValue(1.0, 0.0, 0.0);
    root->addChild(material);

    //add hand
    Hand *currentHand =   graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    setupIVHandGeometry(currentHand);
    root->addChild(IVHandGeometry);

    //add object
    IVObjectGeometry = static_cast<SoSeparator*>(currentHand->getGrasp()->getObject()->getIVRoot()->copy(false));
    root->addChild(IVObjectGeometry);

    // create rendering window
    soQtRenderArea = new SoQtRenderArea(this);
    soQtRenderArea->setBackgroundColor(SbColor(1,1,1));

    // camera setup
    camera->viewAll(root, soQtRenderArea->getViewportRegion());

    // rendering window setup
    soQtRenderArea->setSceneGraph(root);
    soQtRenderArea->show();

    // show window and enter main loop
    SoQt::show(this);


}

SoSeparator *  buildLinkCopy(Link * l)
{
//    SoSeparator * newRoot = new SoSeparator();
//    SoTransform * newTran = newTran = l->getIVTran()->copy(false);
//    newRoot->addChild(newTran);
//    newRoot->addChild(l->getIVMat());
//    newRoot->addChild(l->getIVGeomRoot());
//    return newRoot;
}

/*!
  Copies the scene graph of the hand in the viewer.
  Here, we initialize the hand geometry scene graph that will
  be used in the view window. We do this by performing a deep copy
  of the scene graph for each individual link to avoid getting
  extraneous geometry attached to the hand such as contact points and
  approach directions indicators. This assumes that the hand is free of such
  decorations when the HandView is initialized.

  \param h a hand to copy
*/
 void BCIPreviewView::setupIVHandGeometry(Hand * h)
{
  //Set up the scene graphs for the hand geometry
  IVHandGeometry = new SoSeparator;
  IVHandGeometry->setName("IVHandGeometryRoot");
  //copy palm geometry
  //IVHandGeometry->addChild(h->getPalm()->getIVRoot()->copy(false));
  IVHandGeometry->addChild(buildLinkCopy(h->getPalm()));
  std::vector<DynamicBody *> links;

  //Get link pointers
  h->getAllLinks(links);

  //copy link geometry
  for(unsigned int i = 0; i < links.size(); ++i)
  {
    //IVHandGeometry->addChild(links[i]->getIVRoot()->copy(false));
     // IVHandGeometry->addChild(buildLinkCopy(links.at(i)));
  }
}

BCIPreviewView::~BCIPreviewView()
{
    delete ui;
    delete soQtRenderArea;
}
