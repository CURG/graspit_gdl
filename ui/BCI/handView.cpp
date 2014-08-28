#include "handView.h"
#include "BCI/utils/plannerTools.h"


#include <QWidget>
#include <QApplication>
#include <QObject>
#include <Q3MainWindow>
#include <QTimer>
#include <QThread>

#include <Inventor/SbColor.h>


HandView::HandView(SoQtExaminerViewer *mainViewer, Hand * h, QFrame &parentWindow, QString viewName)
    :viewName_(viewName),
     mainViewer_(mainViewer),
     IVRoot(new SoSeparator()),
     stateID_(-1),
     parentWindow(&parentWindow)
{
  IVRoot->setName("HandviewRoot");
  IVRoot->ref();
  handViewSoQtRenderArea = new SoQtRenderArea(&parentWindow, " ",true);
  handViewSoQtRenderArea->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
  handViewSoQtRenderArea->setBackgroundColor(SbColor(1.0,1.0,1.0));

  ivCamera = initIVCamera();
  SoTransformSeparator *lightSep = initIVLightSeparator();
  IVHandGeometry = initIVHandGeometry(h);
  IVObjectGeometry = initIVObjectGeometry(h);

  IVRoot->addChild(ivCamera);
  IVRoot->addChild(lightSep);
  IVRoot->addChild(IVHandGeometry);
  IVRoot->addChild(IVObjectGeometry);

  handViewSoQtRenderArea->setSceneGraph(IVRoot);

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
SoSeparator * HandView::initIVHandGeometry(Hand * h)
{
  //Set up the scene graphs for the hand geometry
  SoSeparator *IVHandGeometry = new SoSeparator;
  IVHandGeometry->setName("HandViewIVHandGeometry");
  //copy palm geometry
  IVHandGeometry->addChild(h->getPalm()->getIVRoot()->copy(false));
  std::vector<DynamicBody *> links;

  //Get link pointers
  h->getAllLinks(links);

  //copy link geometry
  for(unsigned int i = 0; i < links.size(); ++i)
  {
    IVHandGeometry->addChild(links[i]->getIVRoot()->copy(false));
  }

  return IVHandGeometry;
}

SoSeparator * HandView::initIVObjectGeometry(Hand * h)
{
    return static_cast<SoSeparator*>(h->getGrasp()->getObject()->getIVRoot()->copy(false));
}

SoTransformSeparator * HandView::initIVLightSeparator()
{
    SoRotation *lightDir = new SoRotation;
    lightDir->rotation.connectFrom(&mainViewer_->getCamera()->orientation);

    SoTransformSeparator *lightSep = new SoTransformSeparator;
    lightSep->addChild(lightDir);
    lightSep->addChild(mainViewer_->getHeadlight());
    return lightSep;
}

SoCamera * HandView::initIVCamera()
{
    SoCamera * camera_ = static_cast<SoCamera *>(mainViewer_->getCamera()->copy());
    camera_->scaleHeight(.7);
    camera_->nearDistance = .1;
    camera_->farDistance=1e5;
    camera_->setName("ViewCamera");
    return camera_;
}


/*
  Copies a scene graph transform in to the link's scene graph.
  This must fail if the link's scene graph does not contain the 
  SoTransform node as its first child - which it must be to work properly
  anyway. Recall that children effect their antecedent peers, 
  so to move the root of the geometry of the link, the SoTransform must 
  precede it in the child list. Always putting it first therefore makes sense. 
*/
void HandView::copyIVTran(SoSeparator * parentNode, const SoTransform & ivTran)
{
  SoTransform* tranNode = static_cast<SoTransform *>(ivTran.copy());
  parentNode->removeChild(0);
  parentNode->insertChild(tranNode, 0);
}

/*
    Copies current hand scenegraph transforms to stored scenegraph. 
    Rather than discarding and making a new copy of the scenegraph each time
    the HandView is updated, we keep the old scenegraph and simply update the
    SoTransform node associated with each link within it. This is necessary
    because each body in the Inventor scene graph has its position set independently,
    even when the node is part of an ensemble of bodies such as a robot. The
    joint constraints between bodies are maintained independently of the scene graph. 
*/
void  HandView::copyLinkTransforms(Hand * h, SoSeparator * handIVRoot)
{

  //copy the palm Transform, done separately since we must call getPalm()
  //rather than h->getAllLinks()
  copyIVTran(static_cast<SoSeparator*>(handIVRoot->getChild(0)),*h->getPalm()->getIVTran());

  //copy the transforms for the rest of the links
  std::vector<DynamicBody *> links;
  h->getAllLinks(links);
  for(unsigned int i = 0; i < links.size(); ++i)
  {
    copyIVTran(static_cast<SoSeparator*>(handIVRoot->getChild(i+1)), *links[i]->getIVTran());
  }
  
}

/*
  Updates the current scene graph to represent the a new hand/object relationship.
  This does not copy the current scene graph but instead updates the positions
  of the bodies in the scenegraph by updating the SoTransform node associated with
  each body. See CopyLinkTransforms for more details. 
*/

void HandView::update(const GraspPlanningState & s, Hand & cloneHand)
{
  double testResult = s.getAttribute("testResult");
  double stateID = s.getAttribute("graspId");

  if(testResult > 0.0)
  {
    handViewSoQtRenderArea->setBackgroundColor(SbColor(.8,1,.8));
  }
  else if(testResult <= -1.0)
  {
    handViewSoQtRenderArea->setBackgroundColor(SbColor(1,0.8,0.8));
  }

  if(testResult <= 0.0 && testResult >-1.0){
    handViewSoQtRenderArea->setBackgroundColor(SbColor(1,1.0,1.0));
  }

  if(stateID_ != stateID)
  {
    stateID_ = stateID;

    //First copy the current hand state so that it can be restored.
    cloneHand.saveState();
    std::vector<bool> oldCollisionStatus;
    bci_experiment::planner_tools::resetHandCollisions(&cloneHand, false, oldCollisionStatus);
    //cloneHand.getWorld()->toggleCollisions(false, &cloneHand);
    //need to activate the collision on the copied hand using the cloned hand
    //container object
    cloneHand.getWorld()->toggleCollisions(true, &cloneHand, s.getObject());
    s.execute(&cloneHand);
    
    copyLinkTransforms(&cloneHand, IVHandGeometry);

    copyIVTran(IVObjectGeometry, *s.getObject()->getIVTran());
    //disable collisions between clone hand and everything
    //cloneHand.getWorld()->toggleCollisions(false, &cloneHand, s.getObject());
    bci_experiment::planner_tools::setCollisionState(&cloneHand, oldCollisionStatus);
    //cloneHand.restoreState();
  }

  ivCamera->position = mainViewer_->getCamera()->position;
  ivCamera->orientation = mainViewer_->getCamera()->orientation;
  ivCamera->viewAll(IVRoot, handViewSoQtRenderArea->getViewportRegion());
  mainViewer_->render();
  handViewSoQtRenderArea->render();

}


//Get name (useful to treat names as Unique IDs - needed to save state of preview Window)
QString HandView::getViewName()
{
  return viewName_;
}

HandView::~HandView()
{
    IVRoot->unref();
  delete handViewSoQtRenderArea;
}

