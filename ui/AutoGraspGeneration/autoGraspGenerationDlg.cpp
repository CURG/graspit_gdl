//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: egPlannerDlg.cpp,v 1.9.2.1 2009/07/23 21:18:13 cmatei Exp $
//
//######################################################################

#include <algorithm>

#include "autoGraspGenerationDlg.h"

#include <QGridLayout>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QValidator>
#include <QFileDialog>
#include "search.h"
#include "searchState.h"
#include "body.h"
#include "robot.h"
#include "egPlanner.h"
#include "eigenGrasp.h"

#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "contactExaminerDlg.h"
#include "onLinePlanner.h"
#include "timeTest.h"
#include "guidedPlanner.h"
#include "loopPlanner.h"
#include <QPainter>
#include <QDesktopWidget>
#include <QTimer>

//Graspit DB stuff
#include "DBase/graspit_db_grasp.h"
#include "DBase/graspit_db_model.h"


//#define GRASPITDBG
#include "debug.h"
#include <boost/filesystem.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/features/feature.h>

#include <math.h>
#include <fstream>

#include <chrono>
#include <thread>

#define X_WINDOW_OFFSET 0

void AutoGraspGenerationDlg::exitButton_clicked()
{
  QDialog::accept();
}

void AutoGraspGenerationDlg::init()
{ 


    millisecondsPerMeshPoint = 30000;
    meshPointIncrement = 10;
    currentMeshPointIndex = 0;
    grasp_dir =  "/home/jared/grasp_deep_learning/graspit_gdl/saved_grasps/";

    seedHandMovementTimer = new QTimer(this);
    connect(seedHandMovementTimer,SIGNAL(timeout()), this, SLOT(timerUpdate()));

  energyBox->insertItem("Hand Contacts");
  energyBox->insertItem("Potential Quality");
  energyBox->insertItem("Contacts AND Quality");
  energyBox->insertItem("Autograsp Quality");
  energyBox->insertItem("Guided Autograsp");
  energyBox->setCurrentItem(1);//CHANGED!
  plannerTypeBox->insertItem("Sim. Ann.");
  plannerTypeBox->insertItem("Loop");
  plannerTypeBox->insertItem("Multi-Threaded");
  plannerTypeBox->insertItem("Online");
  plannerTypeBox->insertItem("Assisted Control");
  plannerTypeBox->insertItem("Time Test");
  plannerTypeBox->setCurrentItem(3);//CHANGED!

  plannerInitButton->setEnabled(TRUE);
  plannerResetButton->setEnabled(FALSE);
  plannerStartButton->setEnabled(FALSE);
  instantEnergyButton->setEnabled(FALSE);

  QString n;
  QIntValidator* vAnnSteps = new QIntValidator(1,500000,this);
  annStepsEdit->setValidator(vAnnSteps);
  n.setNum(70000);
  annStepsEdit->setText(n);

  spaceSearchBox->insertItem("Complete");
  spaceSearchBox->insertItem("Axis-angle");
  spaceSearchBox->insertItem("Ellipsoid");
  spaceSearchBox->insertItem("Approach");
  spaceSearchBox->setCurrentItem(3);

  prevGraspButton->setEnabled(FALSE);
  nextGraspButton->setEnabled(FALSE);
  bestGraspButton->setEnabled(FALSE);
  executeGraspButton->setEnabled(FALSE);

  variableBox->setColumnLayout(0, Qt::Vertical);

  varGridLayout = new QGridLayout( variableBox->layout(),1,5 );
  varGridLayout->setSpacing(5);
  varGridLayout->setAlignment(Qt::AlignTop);
  varGridLayout->addMultiCellWidget(spaceSearchLabel,0,0,0,1);
  varGridLayout->addMultiCellWidget(spaceSearchBox,0,0,2,4);

  varGridLayout->addWidget( new QLabel("On", variableBox),1,0 );
  varGridLayout->addWidget( new QLabel("Name", variableBox),1,1 );
  varGridLayout->addWidget( new QLabel("Input", variableBox),1,2 );
  varGridLayout->addWidget( new QLabel("Target", variableBox),1,3 );
  varGridLayout->addWidget( new QLabel("Confidence", variableBox),1,4 );

  inputGloveBox->setEnabled(FALSE);
  inputLoadButton->setEnabled(FALSE);


 
}



void AutoGraspGenerationDlg::destroy()
{
  delete mHandObjectState;
  if (mPlanner) delete mPlanner;

  //cleanup
  for (unsigned int i=0; i<varNames.size(); i++) {
    //  varMainLayout->removeItem(varLayouts[i]);

    varGridLayout->remove(varNames[i]);
    varGridLayout->remove(varCheck[i]);
    varGridLayout->remove(varInput[i]);
    varGridLayout->remove(varTarget[i]);
    varGridLayout->remove(varConfidence[i]);

    delete varNames[i];
    delete varCheck[i];
    delete varInput[i];
    delete varTarget[i];
    delete varConfidence[i];

  }
  varNames.clear();
  varCheck.clear();
  varInput.clear();
  varConfidence.clear();
  varTarget.clear();
  varLayouts.clear();
}


/*
 * Set up the internal structures to use a new object. Sets up the internal hand object state
 * and the grasp. If a planner is running, it does not do anything sensible. 
 */
void AutoGraspGenerationDlg::updateObject(GraspableBody * b)
{
  mObject = b;
  mHand->getGrasp()->setObjectNoUpdate(mObject);
  mHandObjectState->setObject(mObject);
  mHandObjectState->setRefTran(mObject->getTran());

}

void AutoGraspGenerationDlg::setWorld( World *w )
{
  world = w;
}

void AutoGraspGenerationDlg::setMembers( Hand *h, GraspableBody *b )
{
  mPlanner = NULL;
  mHand = h;
  mHandObjectState = new GraspPlanningState(mHand);
  updateObject(b); 
  // mHand->getGrasp()->setGravity(true);
  mHand->getGrasp()->setGravity(false); 
  mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
  mHandObjectState->reset();
  setVariableLayout();

  if (mHand->getNumVirtualContacts() > 0) {
    setContactsBox->setChecked(TRUE);
  }

  updateVariableLayout();
  updateInputLayout();  
 
}

// ----------------------------------- Search State and variable layout management -------------------------------

void AutoGraspGenerationDlg::setVariableLayout()
{
  //cleanup
  for (unsigned int i=0; i<varNames.size(); i++) {
    varGridLayout->remove(varNames[i]);
    varGridLayout->remove(varCheck[i]);
    varGridLayout->remove(varInput[i]);
    varGridLayout->remove(varTarget[i]);
    varGridLayout->remove(varConfidence[i]);
    delete varNames[i];
    delete varCheck[i];
    delete varInput[i];
    delete varTarget[i];
    delete varConfidence[i];
  }
  varNames.clear();
  varCheck.clear();
  varInput.clear();
  varTarget.clear();
  varConfidence.clear();
  varLayouts.clear();

  int size = mHand->getEigenGrasps()->getSize() + 7;

  for (int i=0; i<size; i++) {
    QLabel *name = new QLabel("foo",variableBox);
    QCheckBox *check = new QCheckBox(variableBox);
    connect(check, SIGNAL(clicked()), this, SLOT(variableCheckBoxChanged()));
    QCheckBox *inputCheck = new QCheckBox(variableBox);
    connect(inputCheck, SIGNAL(clicked()), this, SLOT(variableInputChanged()));
    QLabel *target = new QLabel("N/A",variableBox);
    QSlider *slider = new QSlider(0,100,10,0,Qt::Horizontal,variableBox);
    connect(slider, SIGNAL(sliderReleased()), this, SLOT(variableInputChanged()));

    slider->setLineStep(1);
    slider->setMaximumWidth(50);
    varGridLayout->addWidget(check,2+i,0);
    varGridLayout->addWidget(name,2+i,1);
    varGridLayout->addWidget(inputCheck,2+i,2);
    varGridLayout->addWidget(target,2+i,3);
    varGridLayout->addWidget(slider,2+i,4);


    varCheck.push_back( check );
    varNames.push_back( name );
    varInput.push_back( inputCheck );
    varTarget.push_back( target );
    varConfidence.push_back( slider );
  }

}

void AutoGraspGenerationDlg::updateVariableLayout() 
{
  int i;
  for (i=0; i<mHandObjectState->getNumVariables(); i++) {
    varNames[i]->setEnabled(TRUE);
    varNames[i]->setText( mHandObjectState->getVariable(i)->getName() );
    varCheck[i]->setEnabled(TRUE);
    if (mHandObjectState->getVariable(i)->isFixed()) varCheck[i]->setChecked(false);
    else varCheck[i]->setChecked(true);
  }

  for (i=mHandObjectState->getNumVariables(); i < mHand->getEigenGrasps()->getSize() + 7; i++) {
    varNames[i]->setEnabled(FALSE);
    varNames[i]->setText( "n/a" );
    varCheck[i]->setChecked(false);
    varCheck[i]->setEnabled(FALSE);
  }
}

void AutoGraspGenerationDlg::updateInputLayout()
{
  int i;
  for (i=0; i<mHandObjectState->getNumVariables(); i++) {
    if ( !mPlanner || !(mPlanner->isReady() || mPlanner->isActive()) ) {
      varInput[i]->setEnabled(FALSE);
      varInput[i]->setChecked(false);
      varTarget[i]->setText("N/A");
      varTarget[i]->setEnabled(FALSE);
      varConfidence[i]->setValue( 0 );
      varConfidence[i]->setEnabled(FALSE);
    } else {
      GraspPlanningState *t = mPlanner->getTargetState();
      varInput[i]->setEnabled(TRUE);
      QString n;
      n.setNum(t->getVariable(i)->getValue(),'f',3);
      varTarget[i]->setText(n);
      varConfidence[i]->setValue( t->getVariable(i)->getConfidence() * 100 );
      if ( t->getVariable(i)->isFixed() ) {
        varInput[i]->setChecked(TRUE);
        varTarget[i]->setEnabled(TRUE);
        varConfidence[i]->setEnabled(TRUE);
      } else {
        varInput[i]->setChecked(FALSE);
        varTarget[i]->setEnabled(FALSE);
        varConfidence[i]->setEnabled(FALSE);
      }
      if (mHandObjectState->getVariable(i)->getName() == "Tx" || mHandObjectState->getVariable(i)->getName() == "Ty") {
        varInput[i]->setChecked(true);
        varConfidence[i]->setEnabled(true);
        varConfidence[i]->setValue(70);
        mHandObjectState->getVariable(i)->setConfidence(.70);
      }
    }
  }

  for (i=mHandObjectState->getNumVariables(); i < mHand->getEigenGrasps()->getSize() + 7; i++) 
    {
      varInput[i]->setEnabled(FALSE);
      varInput[i]->setChecked(false);
      varTarget[i]->setText("N/A");
      varTarget[i]->setEnabled(FALSE);
      varConfidence[i]->setValue( 0 );
      varConfidence[i]->setEnabled(FALSE);
    }
}

void AutoGraspGenerationDlg::variableInputChanged()
{
  assert(mPlanner);
  GraspPlanningState *t = mPlanner->getTargetState();
  assert(t);
  for (int i=0; i<mHandObjectState->getNumVariables(); i++) {
    if (varInput[i]->isChecked()) {
      varTarget[i]->setEnabled(TRUE);
      varConfidence[i]->setEnabled(TRUE);
      t->getVariable(i)->setFixed(true);
      t->getVariable(i)->setConfidence( ((double)varConfidence[i]->value()) / 100.0);
      DBGP("Conf: " << ((double)varConfidence[i]->value()) / 100.0);
    }
    else {
      varTarget[i]->setEnabled(FALSE);
      t->getVariable(i)->setFixed(false);
      t->getVariable(i)->setConfidence(0);
      varConfidence[i]->setValue(0);
      varConfidence[i]->setEnabled(FALSE);
    }
  }
}

void AutoGraspGenerationDlg::variableCheckBoxChanged()
{
  for (int i=0; i<mHandObjectState->getNumVariables(); i++) {
    if (varCheck[i]->isChecked()) mHandObjectState->getVariable(i)->setFixed(false);
    else mHandObjectState->getVariable(i)->setFixed(true);
  }
  //force a reset of the planner
  plannerStartButton->setEnabled(FALSE);
}

void AutoGraspGenerationDlg::spaceSearchBox_activated( const QString &s )
{
  if ( s==QString("Complete") ) {
    mHandObjectState->setPositionType(SPACE_COMPLETE);
    mHandObjectState->setRefTran( mObject->getTran() );
  }  else if ( s==QString("Axis-angle") ) {
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran( mObject->getTran() );
  } else if ( s==QString("Ellipsoid") ) {
    mHandObjectState->setPositionType(SPACE_ELLIPSOID);
    mHandObjectState->setRefTran( mObject->getTran() );
  } else if ( s==QString("Approach") ) {
    mHandObjectState->setPositionType(SPACE_APPROACH);
    mHandObjectState->setRefTran( mHand->getTran() );
  } else {
    fprintf(stderr,"WRONG SEARCH TYPE IN DROP BOX!\n");
  }
  mHandObjectState->reset();
  updateVariableLayout();
  //force a reset of the planner
  if (mPlanner) mPlanner->invalidateReset();
  updateStatus();
}

//------------------------------------- Show Results stuff ---------------------------------

void AutoGraspGenerationDlg::prevGraspButton_clicked()
{
  mDisplayState--;
  updateResults(true, false);
}

void AutoGraspGenerationDlg::bestGraspButton_clicked()
{
  if (!mPlanner) return;
  mDisplayState = 0;
  updateResults(true, false);
}

void AutoGraspGenerationDlg::nextGraspButton_clicked()
{
  mDisplayState++;
  updateResults(true, false);
};

//If there is a planner and a valid grasp, execute it
//by activating the online planner's execute 
void AutoGraspGenerationDlg::executeGraspButton_clicked()
{

  if (mPlanner && mPlanner->getListSize() > 0)
    {
      updateResults(true, true);
      std::cout<< " Posture in executeGraspButton_clicked():";
      const_cast<GraspPlanningState * >(mPlanner->getGrasp(mDisplayState))->getPosture()->print();
      onlineGraspButton_clicked();
    }
}

void AutoGraspGenerationDlg::plannerUpdate()
{

  assert(mPlanner);
  //  mDisplayState = 0;
  updateResults(false, false);
  //if we are using the CyberGlove for input this updates the target values
  if (inputGloveBox->isChecked()) {
    updateInputLayout();
  }
}

void AutoGraspGenerationDlg::updateResults(bool render, bool execute)
{
  {
  assert(mPlanner);
  if (execute) assert( mPlanner->getType() == PLANNER_ONLINE);

  QString nStr;
  nStr.setNum(mPlanner->getCurrentStep());
  currentStepLabel->setText(QString("Current step: ") + nStr);

  nStr.setNum(mPlanner->getRunningTime(),'f',0);
  timeLabel->setText(QString("Time used: ") + nStr + QString(" sec."));

  int d = mPlanner->getListSize();
  int rank, size, iteration; double energy;


  //CHANGED! so now it cycles
  if (d==0) {
    mDisplayState = rank = size = energy = iteration = 0; render = false;
  } else if (mDisplayState < 0){
    mDisplayState = 9;
  } else if ( mDisplayState >= d) {
    mDisplayState = 0;
  } 

  if ( d!=0 ){
    const GraspPlanningState *s = mPlanner->getGrasp( mDisplayState);
    rank = mDisplayState+1;
    size = d;
    iteration = s->getItNumber();
    energy = s->getEnergy();
  }





  QString n1,n2;
  n1.setNum(rank);
  n2.setNum(size);
  rankLabel->setText("Rank: " + n1 + "/" + n2);
  n1.setNum(iteration);
  iterationLabel->setText("Iteration: " + n1);
  n1.setNum(energy,'f',3);
  energyLabel->setText("Energy: " + n1);

  if (render) {
    mPlanner->showGrasp(mDisplayState);
    //mHand->getWorld()->findAllContacts();
    //mHand->getGrasp()->update();
    mPlanner->getGrasp(mDisplayState)->printState();
    //mHand->autoGrasp(true);
  }

  if (execute) {
    OnLinePlanner *op = (OnLinePlanner*)mPlanner;
    op->executeGrasp(mDisplayState);
  }
  if(d!=0)
    {
      bool unlock = false;
      if (!mPlanner->mListAttributeMutex.locked())
      {
        mPlanner->mListAttributeMutex.lock();
        unlock = true;
      }

      if (unlock)
        mPlanner->mListAttributeMutex.unlock();
    }
  }//end lock scope


}

// ----------------------------- Settings management ---------------------------


void AutoGraspGenerationDlg::updateStatus()
{
  PlannerState s = DONE;
  if (mPlanner) s = mPlanner->getState();
  DBGP("Update Layout");
  switch(s) {
  case INIT:
    plannerInitButton->setEnabled(FALSE);
    plannerResetButton->setEnabled(TRUE);
    plannerStartButton->setEnabled(FALSE);
    plannerStartButton->setText(">");

    prevGraspButton->setEnabled(FALSE);
    nextGraspButton->setEnabled(FALSE);
    bestGraspButton->setEnabled(FALSE);
    executeGraspButton->setEnabled(FALSE);
    mObject->showFrictionCones(true);

    inputGloveBox->setEnabled(FALSE);
    inputLoadButton->setEnabled(FALSE);
    onlineDetailsGroup->setEnabled(TRUE);

    break;
  case READY:
    {
      plannerInitButton->setEnabled(FALSE);
      plannerResetButton->setEnabled(TRUE);
      plannerStartButton->setEnabled(TRUE);
      plannerStartButton->setText(">");

      prevGraspButton->setEnabled(TRUE);
      nextGraspButton->setEnabled(TRUE);
      bestGraspButton->setEnabled(TRUE);
      executeGraspButton->setEnabled(TRUE);
      mObject->showFrictionCones(true);

      inputGloveBox->setEnabled(TRUE);
      inputLoadButton->setEnabled(TRUE);
      if (mPlanner->getType() == PLANNER_ONLINE) {
	onlineDetailsGroup->setEnabled(TRUE);
	showSolutionBox->setChecked(TRUE);
	showCloneBox->setChecked(TRUE);
      }
      else onlineDetailsGroup->setEnabled(FALSE);
    }
    break;
  case RUNNING:
    plannerInitButton->setEnabled(FALSE);
    plannerResetButton->setEnabled(FALSE);
    plannerStartButton->setEnabled(TRUE);
    plannerStartButton->setText("||");

    prevGraspButton->setEnabled(FALSE);
    nextGraspButton->setEnabled(FALSE);
    bestGraspButton->setEnabled(FALSE);
    executeGraspButton->setEnabled(FALSE);
    executeGraspButton->setEnabled(FALSE);
    mObject->showFrictionCones(false);
    break;
  default:
    plannerInitButton->setEnabled(TRUE);
    plannerResetButton->setEnabled(FALSE);
    plannerStartButton->setEnabled(FALSE);
    plannerStartButton->setText(">");

    prevGraspButton->setEnabled(FALSE);
    nextGraspButton->setEnabled(FALSE);
    bestGraspButton->setEnabled(FALSE);
    executeGraspButton->setEnabled(FALSE);
    mObject->showFrictionCones(true);

    inputGloveBox->setEnabled(FALSE);
    inputLoadButton->setEnabled(FALSE);
    onlineDetailsGroup->setEnabled(FALSE);
    break;
  }
  updateInputLayout();
}

void AutoGraspGenerationDlg::energyBox_activated( const QString & )
{
  //force a reset of the planner
  if (mPlanner) mPlanner->invalidateReset();
  updateStatus();
}

void AutoGraspGenerationDlg::setContactsBox_toggled( bool checked)
{
  if (checked) {
    if ( mHand->getNumVirtualContacts() == 0 ) {
      //if we are asking to use pre-set contacts, but none are available, pop up the dialog
      //for loading virtual contacts
      ContactExaminerDlg dlg(this);
      dlg.exec();
    }
    if (mHand->getNumVirtualContacts() == 0) {
      //if we still have no virtual contacts, un-check the box
      setContactsBox->setChecked(false);
    }
  }
  if (mPlanner) mPlanner->invalidateReset();
  updateStatus();
}

void AutoGraspGenerationDlg::readPlannerSettings()
{
  assert(mPlanner);
  //energy type
  QString s = energyBox->currentText();
  if ( s == QString("Hand Contacts") ) {
    mPlanner->setEnergyType(ENERGY_CONTACT);
  } else if ( s == QString("Potential Quality") ) {
    mPlanner->setEnergyType(ENERGY_POTENTIAL_QUALITY);
  } else if ( s == QString("Autograsp Quality") ) {
    mPlanner->setEnergyType(ENERGY_AUTOGRASP_QUALITY);
  } else if ( s == QString("Contacts AND Quality") ) {
    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
  } else if ( s == QString("Guided Autograsp") ) {
    mPlanner->setEnergyType(ENERGY_GUIDED_AUTOGRASP);
  } else {
    fprintf(stderr,"WRONG ENERGY TYPE IN DROP BOX!\n");
  }

  //contact type
  if ( setContactsBox->isChecked() ) {
    mPlanner->setContactType(CONTACT_PRESET);
  } else {
    mPlanner->setContactType(CONTACT_LIVE);
  }

  //steps
  int steps = annStepsEdit->text().toInt();
  mPlanner->setMaxSteps(steps);
}

void AutoGraspGenerationDlg::plannerComplete()
{
  updateStatus();
  bestGraspButton_clicked();
}






void AutoGraspGenerationDlg::plannerInit_clicked()
{  
  assert(world->getCurrentHand());
  if (world->getCurrentHand()->getEigenGrasps() == NULL) {
      fprintf(stderr,"Current hand has no EigenGrasp information!\n");
      return;
  }

  if (world->getNumGB() > 1) {
    fprintf(stderr,"Too many graspable bodies!\n");
    return;
  }

  this->setMembers(world->getCurrentHand(), world->getGB(0));
  
  spaceSearchBox_activated(QString("Approach"));

  QString s = plannerTypeBox->currentText();
  if (s == QString("Sim. Ann.")) {
    if (mPlanner) delete mPlanner;
    mPlanner = new SimAnnPlanner(mHand);
    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);
    energyBox->setEnabled(TRUE);
  } else if (s == QString("Loop")) {
    if (mPlanner) delete mPlanner;
    mPlanner = new LoopPlanner(mHand);
    ((LoopPlanner*)mPlanner)->setModelState(mHandObjectState);
    energyBox->setEnabled(TRUE);
  } else if (s == QString("Multi-Threaded")) {
    if (mPlanner) delete mPlanner;
    mPlanner = new GuidedPlanner(mHand);
    ((GuidedPlanner*)mPlanner)->setModelState(mHandObjectState);
    energyBox->setCurrentItem(2);
    energyBox->setEnabled(FALSE);


  } else if (s == QString("Online") ) {
    if (mPlanner) delete mPlanner;
    mPlanner = new OnLinePlanner(mHand);
    ((OnLinePlanner*)mPlanner)->setModelState(mHandObjectState);
    energyBox->setEnabled(TRUE);
    energyBox->setCurrentItem(2);
    QString n;
    n.setNum(3000);
    annStepsEdit->setText(n);
    QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(onlinePlannerUpdate())); 
  }


  else if ( s == QString("Time Test") ) {
    if (mPlanner) delete mPlanner;
    mPlanner = new MTTester(mHand);
  } else {
    fprintf(stderr,"Unknown planner type requested\n");
    return;
  } 

  graspItGUI->getIVmgr()->getWorld()->setCurrentPlanner(mPlanner);


  QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(plannerUpdate()));
  QObject::connect(mPlanner,SIGNAL(complete()),this,SLOT(plannerComplete()));

  updateStatus();
  plannerReset_clicked();  

}




#include <set>
bool operator<(const position & a, const position & b)
{
    if(a.x() < b.x() || a.y() < b.y() || a.z() < b.z())
        return true;
    return false;

}

void uniquifyPointCloud(std::vector<position> & vertices)
{
    std::set<position> s;
    unsigned size = vertices.size();
    for( unsigned i = 0; i < size; ++i ) s.insert( vertices[i] );
    vertices.assign( s.begin(), s.end() );
}

void showNormals(Body * b, pcl::PointCloud<pcl::PointNormal> & cloud)
{
    b->breakVirtualContacts();
   for(int i = 0; i < cloud.size(); ++i)
    {
        PointContact c(b, b, position(cloud.at(i).x,cloud.at(i).y,cloud.at(i).z),
                       vec3(cloud.at(i).normal_x,cloud.at(i).normal_y,cloud.at(i).normal_z));
        VirtualContact * vc = new VirtualContact(1, 1, &c);
        b->addVirtualContact(vc);
    }
  b->showFrictionCones(1);
}

void showSingleNormal(Body * b, pcl::PointCloud<pcl::PointNormal> & cloud, int normalIndex)
{
    b->breakVirtualContacts();

    PointContact c(b, b, position(cloud.at(normalIndex).x,cloud.at(normalIndex).y,cloud.at(normalIndex).z),
                   vec3(cloud.at(normalIndex).normal_x,cloud.at(normalIndex).normal_y,cloud.at(normalIndex).normal_z));
    VirtualContact * vc = new VirtualContact(1, 1, &c);
    b->addVirtualContact(vc);

    b->showFrictionCones(1);
}

void AutoGraspGenerationDlg::generateHandPoses()
{
    std::vector<position> vertices;
    mPlanner->getTargetState()->getObject()->getGeometryVertices(&vertices);
    uniquifyPointCloud(vertices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout  << "vertices.size()" << vertices.size();
    for(int i=0; i < vertices.size();i++)
    {
        const pcl::PointXYZ point(vertices[i].x(),vertices[i].y(),vertices[i].z());
        cloud->push_back(point);
    }

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    normalEstimation.setInputCloud (cloud);
    normalEstimation.setSearchMethod (tree);
    normalEstimation.setKSearch (20);
    normalEstimation.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    //pcl::PointCloud<pcl::PointNormal> cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, cloud_with_normals);

    currentMeshPointIndex =0;
    //showNormals(mPlanner->getTargetState()->getObject(), cloud_with_normals);
}


void AutoGraspGenerationDlg::chooseNewScene(int handId, int modelId)
{
  if (rob != NULL)
    world->destroyElement(rob, true);
  if (obj != NULL)
    world->destroyElement(obj, true);


  rob = world->importRobot(handXMLNames[handId]);
  obj = world->importBody("GraspableBody", modelXMLNames[modelId]);

}

void AutoGraspGenerationDlg::moveHandToNextPose()
{

    pcl::PointNormal pointNormalInBodyCoord = this->cloud_with_normals.at(currentMeshPointIndex);

    double x = pointNormalInBodyCoord.x;
    double y = pointNormalInBodyCoord.y;
    double z = pointNormalInBodyCoord.z;

    double normal_x = pointNormalInBodyCoord.normal_x;
    double normal_y = pointNormalInBodyCoord.normal_y;
    double normal_z = pointNormalInBodyCoord.normal_z;


//http://stackoverflow.com/questions/21622956/how-to-convert-direction-vector-to-euler-angles
//    The nose of that airplane points towards the direction vector
//    D=(XD,YD,ZD) .
      vec3 D = vec3(normal_x,normal_y,normal_z);

//    Towards the roof is the up vector
//    U=(XU,YU,ZU) .
      vec3 U = D * vec3(0,1,0) * D;

    transf worldToObject = mPlanner->getTargetState()->getObject()->getTran();
    transf meshPointToApproachTran = translate_transf(vec3(-150*normal_x, -150*normal_y, -150*normal_z));
    transf orientHand = rotXYZ(0,-M_PI/2.0,0) * coordinate_transf(position(x,y,z),D,U) ;

    mHand->setTran( orientHand * meshPointToApproachTran*worldToObject );

    //showSingleNormal(mPlanner->getTargetState()->getObject(), cloud_with_normals,currentHandPositionIndex );

    currentMeshPointIndex+=meshPointIncrement;

}


void AutoGraspGenerationDlg::saveGrasps()
{

    std::stringstream ss;
    //folder to save grasps
    ss << grasp_dir;

    //subfolder for the object/hand we are grasping
    ss << mPlanner->getTargetState()->getObject()->getName().toStdString().c_str();


    std::string fullGraspPath = ss.str();

    ss << "/grasps";
    ss << ".txt";

    std::string fullGraspPathAndFilename = ss.str();

    boost::filesystem3::create_directories(fullGraspPath);

    FILE *f = fopen(fullGraspPathAndFilename.c_str(),"w");
    for (int i=0; i<mPlanner->getListSize(); i++)
    {
        fprintf(f,"graspId: %d\n", i);
        fprintf(f,"energy: %f\n", mPlanner->getGrasp(i)->getEnergy());

        mPlanner->getGrasp(i)->writeToFile(f);
        fprintf(f,"\n");
    }
    fclose(f);

    std::cout << "saved grasps to: " << fullGraspPathAndFilename.c_str() << std::endl;
}

void AutoGraspGenerationDlg::plannerReset_clicked() 
{
  assert(mPlanner);
  readPlannerSettings();
  mPlanner->resetPlanner();
  updateStatus();
}

void AutoGraspGenerationDlg::timerUpdate()
{
    std::cout << "timer update called" << std::endl;
    std::cout << "cloud_with_normals.size() " << cloud_with_normals.size() << std::endl;
    std::cout << "currentHandPositionIndex" << currentMeshPointIndex << std::endl;

    if (cloud_with_normals.size() < currentMeshPointIndex)
    {
        stopPlanner();
        saveGrasps();
    }
    else
    {
        currentModel++;
        if (currentHand < handXMLNames.size() && currentModel < modelXMLNames.size())
          chooseNewScene(currentHand, currentModel); 
        else if (currentModel > modelXMLNames.size()) {
          currentHand++; currentModel = 0;
        }
        moveHandToNextPose();
    }
}

void AutoGraspGenerationDlg::startPlanner()
{
    generateHandPoses();
    seedHandMovementTimer->start(millisecondsPerMeshPoint);
    mPlanner->startPlanner();
    updateStatus();
}


void AutoGraspGenerationDlg::stopPlanner()
{
  seedHandMovementTimer->stop();
  mPlanner->pausePlanner();
  updateStatus();
}

void AutoGraspGenerationDlg::plannerStart_clicked()
{	
  
  if (!mPlanner->isActive()){
    startPlanner();
  } else {
    stopPlanner();
  }
}

void AutoGraspGenerationDlg::plannerTypeBox_activated( const QString & )
{
  if (mPlanner) {
    mHand->getWorld()->setCurrentPlanner(NULL);
    delete mPlanner;
    mPlanner = NULL;
  }
  updateStatus();
}


//----------------------------------- Dedicated on-line planner control ---------------------------

void AutoGraspGenerationDlg::autoGraspBox_clicked()
{

}

//this slot does the updating that's specific to the online planner
void AutoGraspGenerationDlg::onlinePlannerUpdate()
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  QString num;

  double objDist = op->getObjectDistance();
  num.setNum(objDist,'f',0);
  objDistLabel->setText("Object distance: "  + num);
  /*
    double solDist = op->getSolutionDistance();
    if (solDist >= 0) num.setNum(solDist,'f',0);
    else num.setAscii("N/A");
    solDistLabel->setText("Solution distance: " + num);
  */
  ActionType s = op->getAction();
  switch (s) {
  case ACTION_PLAN:
    num.setAscii("PLANNING");
    break;
  case ACTION_GRASP:
    num.setAscii("GRASPING");
    break;
  case ACTION_OPEN:
    num.setAscii("OPEN");
    break;
  default:
    num.setAscii("N/A");
    break;
  }
  onlineStatusLabel->setText("Status: " + num);

  num.setNum( op->getSABufferSize() );
  saBufferLabel->setText("SimAnn buffer: " + num);
  num.setNum( op->getFCBufferSize() );
  fcBufferLabel->setText("FC Thread buffer: " + num); 
}

void AutoGraspGenerationDlg::onlineGraspButton_clicked()
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->action(ACTION_GRASP);
  onlinePlannerUpdate();
}

void AutoGraspGenerationDlg::onlineReleaseButton_clicked()
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->action(ACTION_OPEN);
  onlinePlannerUpdate();
}


void AutoGraspGenerationDlg::onlinePlanButton_clicked()
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->action(ACTION_PLAN);
  onlinePlannerUpdate(); 
}


void AutoGraspGenerationDlg::instantEnergyButton_clicked()
{
  assert(mPlanner);
  // mPlanner->instantEnergy();
}


void AutoGraspGenerationDlg::showCloneBox_toggled( bool c)
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->showClone(c);  
}


void AutoGraspGenerationDlg::showSolutionBox_toggled( bool c)
{	
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->showSolutionClone(c);
}

void AutoGraspGenerationDlg::useVirtualHandBox_clicked()
{
  /*
    assert( mPlanner->getType() == PLANNER_ONLINE);
    OnLinePlanner *op = (OnLinePlanner*)mPlanner;
    bool c = useVirtualHandBox->isChecked();
    op->controlVirtualHand(c);

  */
}

void AutoGraspGenerationDlg::useRealBarrettBox_toggled( bool s)
{
  assert( mPlanner->getType() == PLANNER_ONLINE);
  OnLinePlanner *op = (OnLinePlanner*)mPlanner;
  op->useRealBarrettHand(s);
}

//---------------------------------------- Input selection -----------------------------------------

void AutoGraspGenerationDlg::inputGloveBox_toggled( bool on)
{
  assert(mPlanner);
  mPlanner->setInput(INPUT_GLOVE, on);
}

void AutoGraspGenerationDlg::inputLoadButton_clicked()
{
  assert(mPlanner);
  QString fn = QFileDialog::getOpenFileName(this, QString(),  QString(getenv("GRASPIT"))+QString("/models/grasps"),
					    "Grasp Files (*.txt)" );

  if ( fn.isEmpty() ) {
    return;
  }

  FILE *fp = fopen(fn.latin1(), "r");
  bool success = true;
  if (!fp) {
    DBGA("Failed to open input file!");
    success = false;
  }else if (!mPlanner->getTargetState()->readFromFile(fp)) {
    DBGA("Failed to read target from input file!");
    success = false;
  } else {
    DBGA("Target values loaded successfully");
  }
  fclose(fp);
  mPlanner->setInput(INPUT_FILE, success);
  updateInputLayout();
}


//-------------------------------------------- Auto grasp planning -------------------------------------

void AutoGraspGenerationDlg::loadModelsDirButton_clicked()
{
    QString fn( QFileDialog::getExistingDirectory(this, QString(), QString(getenv("GRASPIT"))+QString("/models/object_database")) );
    if ( fn.isEmpty() ) {
        return;
    }
    modelsDirName = fn;
    modelsDirLbl->setText(modelsDirName);

}

void AutoGraspGenerationDlg::loadHandsDirButton_clicked()
{
    QString fn( QFileDialog::getExistingDirectory(this, QString(), QString(getenv("GRASPIT"))+QString("/models/robots")) );
    if ( fn.isEmpty() ) {
        return;
    }
    handsDirName = fn;
    handsDirLbl->setText(handsDirName);

}


void AutoGraspGenerationDlg::startAutoGraspButton_clicked()
{

    autoGenStatusLbl->setText("Collecting models and hands...");


    struct dirent *parentEntry; 
    DIR *parentDir;
    DIR *childDir;

    modelXMLNames.clear();
    handXMLNames.clear();

    // --------------------------- MODELS ---------------------------
    DBGA("Looking for models in " << modelsDirName.toStdString() << "...");

    parentDir = opendir(modelsDirName);
    if (parentDir == NULL) {
      DBGA("Please selected a models directory...");
      return;
    }

    while ((parentEntry = readdir(parentDir))) {
      if (parentEntry->d_name[0] != '.') {
        childDir = opendir(modelsDirName + "/" + parentEntry->d_name);
        if (childDir != NULL) {
          modelXMLNames.push_back(QString(modelsDirName + "/" + parentEntry->d_name + "/" + parentEntry->d_name + ".xml"));
          closedir(childDir);
        }
      }
    }
    closedir(parentDir);

    DBGA("FOUND MODELS!");
    for (unsigned int i=0; i<modelXMLNames.size(); i++) {
      DBGA(" --- " << modelXMLNames[i].toStdString());
    }

    // --------------------------- HANDS ---------------------------
    DBGA("Looking for hands in " << handsDirName.toStdString() << "...");

    parentDir = opendir(handsDirName);
    if (parentDir == NULL) {
      DBGA("Please selected a hands directory...");
      return;
    }

    while ((parentEntry = readdir(parentDir))) {
      if (parentEntry->d_name[0] != '.') {
        childDir = opendir(handsDirName + "/" + parentEntry->d_name);
        if (childDir != NULL) {
          handXMLNames.push_back(QString(handsDirName + "/" + parentEntry->d_name + "/" + parentEntry->d_name + ".xml"));
          closedir(childDir);
        }
      }
    }
    closedir(parentDir);

    DBGA("FOUND HANDS!");
    for (unsigned int i=0; i<handXMLNames.size(); i++) {
      DBGA(" --- " << handXMLNames[i].toStdString());
    }

    autoGenStatusLbl->setText("HIT THE OTHER GO BUTTON...");

    currentHand = 0;
    currentModel = 0;

    rob = world->importRobot(handXMLNames[currentHand]);
    obj = world->importBody("GraspableBody", modelXMLNames[currentModel]);



}




