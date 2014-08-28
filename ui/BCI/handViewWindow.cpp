#include "handViewWindow.h"

//HandViewWindow:
//Set the HandViewWindow which contains he views themselves
HandViewWindow::HandViewWindow(QWidget * parent, Hand * h, const QRect & geom, SoNode * IVRoot, QWidget * stageFrame):currentPreviewIndex(-1),
maxViewSize(3), cloneHand(new Hand(h->getWorld(), "newHand")), geom_(geom), grid(NULL), stageFrame_(stageFrame)
{


  handViewWindow = new QFrame(NULL);
  handViewWindow->setWindowFlags(Qt::WindowStaysOnTopHint);
  viewHolder = new QFrame(NULL);
  handViewWindow->setGeometry(geom);
  hbox = new QHBoxLayout(handViewWindow);

    handViewWindow->show();
    viewHolder->setGeometry(0,0,2*geom_.width()/3,2*geom_.height()/3);
  viewHolder->setMinimumSize(2*geom_.width()/3,2*geom_.height()/3);
  viewHolder->setMaximumSize(2*geom_.width()/3,2*geom_.height()/3);
  viewHolder->setFrameStyle(QFrame::Box | QFrame::Raised);
 viewHolder->setLineWidth(2);
  if(IVRoot)
    {
      SoQtRenderArea * render = new SoQtRenderArea(viewHolder, " ",true);

      vbox1 = new QVBoxLayout();
      hbox->addLayout(vbox1);
      vbox1->addWidget(viewHolder);
      vbox1->addWidget(stageFrame);
      //render->setSceneGraph(IVRoot);
      SoSeparator * testRoot = new SoSeparator;
      //SoCone * testCone = new SoCone;
      //SoCamera * testCamera = new SoPerspectiveCamera;

      SoMaterial * testMaterial = new SoMaterial;

      testMaterial->diffuseColor.setValue(1,0,0);
      testRoot->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
      SoTransformSeparator *lightSep = new SoTransformSeparator;
      SoRotation *lightDir = new SoRotation;
      lightDir->rotation.connectFrom(&graspItGUI->getIVmgr()->getViewer()->getCamera()->orientation);
      lightSep->addChild(lightDir);
      lightSep->addChild(graspItGUI->getIVmgr()->getViewer()->getHeadlight());
      SoLightModel * model = new SoLightModel;
      model->model=SoLightModel::PHONG;
      testRoot->addChild(lightSep);
      testRoot->addChild(model);
      testRoot->addChild(IVRoot);
      render->setSceneGraph(testRoot);
      render->setBackgroundColor(SbColor(1,1,1));
      render->scheduleRedraw();
      render->render();
      render->show();
    }
    vbox2 = new QVBoxLayout();
  vbox2->setGeometry(QRect(0,0,geom_.width()/3,geom_.height()));
  vbox2->setSizeConstraint(QLayout::SetMinimumSize);
  hbox->addLayout(vbox2);

 // vbox2->addWidget(new QWidget());
  //handViewWindow->setWindowFlags(Qt::WindowStaysOnTopHint);
  //handViewWindow->resize(QSize(900,100));
   //handViewFrame->setGeometry(geom_);
   handViewWindow->setFrameStyle(QFrame::Box | QFrame::Raised);
   handViewWindow->setLineWidth(4);


  cloneHand->cloneFrom(h);
  cloneHand->setTransparency(0.0);
  cloneHand->setRenderGeometry(false);
  cloneHand->showVirtualContacts(false);
  cloneHand->getWorld()->toggleCollisions(false, cloneHand);
}

void HandViewWindow::clearViews()
{
//! First clear the grid if necessary:
  views.clear();
 // vbox2->removeWidget(grid);
  if(grid)
    QWidget().setLayout(grid);
  delete grid;
  //if ( grid->layout() != NULL )
  //{
  //  QWidget().setLayout(grid->layout());
  //}
    /*QLayoutItem* item;
    while ( ( item = grid->layout()->takeAt( 0 ) ) != NULL )
    {
        grid->remove(item->widget());
        delete item->widget();
        delete item;
    }

    //delete grid->layout();
  }*/
}

void HandViewWindow::initViews(Hand * h)
{
 grid = new QGridLayout(vbox2,3,1);//Set the second parameter to 1 for single column

  for(unsigned int i = 0; i < maxViewSize; ++i)
    {

      QFrame * viewWindowFrame = new QFrame(handViewWindow);
      double t = h->getPalm()->getTransparency();
      h->setTransparency(0);
      HandView * hv = new HandView(graspItGUI->getIVmgr()->getViewer(), h, *viewWindowFrame, QString::number(i));        //add a view
      h->setTransparency(t);
      if(i == 0){
        viewWindowFrame->setFrameStyle(QFrame::Box| QFrame::Plain);
        hv->getViewViewer()->setBorder(true);
      }
      else{
      viewWindowFrame->setFrameStyle(QFrame::Box | QFrame::Raised);
      }
      viewWindowFrame->setLineWidth(2);
      views.push_back(hv);
      grid->addWidget(viewWindowFrame);
    }
}



/* Adds a view of a particular grasp.
   This function updates a view window to contain a visualization of a grasp planning
   state using a preset hand and object. If the hand or object have changed since the
   initialization of the view window, that will not be reflected in the view.
*/
bool HandViewWindow::addView(HandObjectState& s, int i)
{
  if (i < maxViewSize)
    views[i]->update(s,*cloneHand);
  return true;
}

//Used to go directly to a choice
void HandViewWindow::setCurrentView(int num)
{
  if (num > maxViewSize - 1)
    return;
  if (currentPreviewIndex >= 0)
    views[currentPreviewIndex]->getViewWindow()->move(views[currentPreviewIndex]->getViewWindow()->x(),views[currentPreviewIndex]->getViewWindow()->y()+10);

  currentPreviewIndex = num;

  views[currentPreviewIndex]->getViewWindow()->move(views[currentPreviewIndex]->getViewWindow()->x(),views[currentPreviewIndex]->getViewWindow()->y()-10);
}

//Returns which grasp is currently in the preview window
int HandViewWindow::getCurrentGrasp()
{
  return currentPreviewIndex;
}
