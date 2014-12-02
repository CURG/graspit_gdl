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
// Author(s): Andrew T. Miller 
//
// $Id: graspitServer.cpp,v 1.7.4.1 2009/07/23 21:18:01 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the application's TCP server.
 */
#include <QDateTime>
#include <QTextStream>
#include <QApplication>
#include <iostream>
#include "graspitServer.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "contact.h"
#include "onLinePlanner.h"
#include "searchStateUtils.h"
#include <sstream>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/fields/SoSFVec3f.h>
#include <QMutexLocker>
#include "BCI/bciService.h"

//helper function to get current world planner
EGPlanner* currentWorldPlanner(){ return graspItGUI->getIVmgr()->getWorld()->getCurrentPlanner();}



ClientSocket::ClientSocket( int sock, QObject *parent, const char *name ) :
  Q3Socket( parent, name )
{
      connect( this, SIGNAL(readyRead()), SLOT(readClient()) );
      connect( this, SIGNAL(connectionClosed()), SLOT(connectionClosed()) );
      setSocket( sock );
}


/*!
  Stub destructor.
*/
ClientSocket::~ClientSocket()
{
#ifdef GRASPITDBG
  std::cout << "client socket destroyed"<<std::endl;
#endif
}

/*!
  This reads the next portion of a command line after the command to collect
  the body indices, and returns a vector of pointers to those bodies.  If
  this portion starts with the word "ALL", then all the bodies in the world
  are added to the body vector.  Otherwise it reads the number of body
  indices, and reads each index in turn, adding the corresponding body
  pointer to the vector.  If an index is read that does not exist, an error
  message is sent back and this method returns FAILURE.
*/
int
ClientSocket::readBodyIndList(std::vector<Body *> &bodyVec)
{
  QTextStream os(this);
  int i,numBodies,bodNum;
  bool ok;
  World *world = graspItGUI->getIVmgr()->getWorld();
  std::cout << "ReadBodyIndList Line:"<<line.latin1() << std::endl;

  /* if the index list is empty, use every body and send
     back the count
  */

  if (strPtr == lineStrList.end()) return FAILURE;

  if ((*strPtr).startsWith("ALL")) {
    strPtr++;
    for (i=0;i<world->getNumBodies();i++)
      bodyVec.push_back(world->getBody(i));
    std::cout << "Sending num bodies: "<<world->getNumBodies()<<std::endl;
    os << world->getNumBodies() << endl;
    return SUCCESS;
  }
  
  numBodies = (*strPtr).toInt(&ok);
  if (!ok) return FAILURE;
  strPtr++;
  
  for (i=0;i<numBodies;i++) {
    if (strPtr == lineStrList.end()) return FAILURE;
    bodNum = (*strPtr).toInt(&ok);
    if (!ok) return FAILURE;
    
    if (bodNum>=0 && bodNum<world->getNumBodies()) {
      bodyVec.push_back(world->getBody(bodNum));
      if (world->getBody(bodNum)==NULL) {
	os << "Error: Cannot find body " << bodNum <<"\n";
        return FAILURE;
      }
    }
    else {
      os << "Error: Cannot find body " << bodNum <<"\n";
      return FAILURE;
    }
    strPtr++;
  }
  return SUCCESS;
}

// read in 7 param transf given as  pos(x y z) Qauternion(w x y z) 
int ClientSocket::readTransf(transf * tr){
	bool ok = true;
	try{
		*tr = transf(Quaternion((*(strPtr + 3)).toDouble(&ok),  //qw
			(*(strPtr + 4)).toDouble(&ok),  //qx
			(*(strPtr + 5)).toDouble(&ok),  //qy
			(*(strPtr + 6)).toDouble(&ok)), //qz
			vec3((*strPtr).toDouble(&ok), //x
			(*(strPtr+1)).toDouble(&ok),  //y
			(*(strPtr + 2)).toDouble(&ok)));  //z
		strPtr +=7;
	}
	catch(...){
		if (!ok)
			return FAILURE;
		else
			std::cout <<"unknown error in ClientSocket::readTransf \n";
	}
	return SUCCESS;
}


/*!
  This reads the next portion of a command line after the command to collect
  the robot indices, and returns a vector of pointers to those robots.  If
  this portion starts with the word "ALL", then all the robots in the world
  are added to the robot vector.  Otherwise it reads the number of robot
  indices, and reads each index in turn, adding the corresponding robot
  pointer to the vector.  If an index is read that does not exist, an error
  message is sent back and this method returns FAILURE.
*/
int
ClientSocket::readRobotIndList(std::vector<Robot *> &robVec)
{
  QTextStream os(this);
  int i,robNum,numRobots;
  bool ok;
  World *world = graspItGUI->getIVmgr()->getWorld();
  std::cout << "ReadRobotIndList Line:"<<line.latin1() << std::endl;

  /* if the index list is empty, use every robot and send
     back the count
  */
  if (strPtr == lineStrList.end()) return FAILURE;

  if ((*strPtr).startsWith("ALL")) {
    strPtr++;
    for (i=0;i<world->getNumRobots();i++)
      robVec.push_back(world->getRobot(i));
    std::cout << "Sending num robots: "<<world->getNumRobots()<<std::endl;
    os << world->getNumRobots() << endl;
    return SUCCESS;
  }
  
  numRobots = (*strPtr).toInt(&ok);
  if (!ok) return FAILURE;
  strPtr++;
  
  for (i=0;i<numRobots;i++) {
    if (strPtr == lineStrList.end()) return FAILURE;
    robNum = (*strPtr).toInt(&ok);
    if (!ok) return FAILURE;
    
    if (robNum>=0 && robNum<world->getNumRobots()) {
      robVec.push_back(world->getRobot(robNum));
      if (world->getRobot(robNum)==NULL) {
	os << "Error: Cannot find robot " << robNum <<"\n";
        return FAILURE;
      }
    }
    else {
      os << "Error: Cannot find robot " << robNum <<"\n";
      return FAILURE;
    }
    strPtr++;
  }
  return SUCCESS;
}

/*!
  This is the main routine for parsing input on the clientSocket.
  There should be one command for each line of input.  This reads one
  line, and looks at the first word (up to the first space character) to
  determine the command.   Then if there are body or robot indices to read,
  it calls a support routine to read those and return a vector of bodies or
  robots.  These are then passed to the appropriate routine to carry out the
  action and write out any necessary results.
*/
void
ClientSocket::readClient()
{

  graspItGUI->startStuffHere();
  return;

  int i,numData,numBodies,numRobots;
  double time;
  std::vector<Body *> bodyVec;
  std::vector<Robot *> robVec;

  bool ok;

  while ( canReadLine() ) {
    line = readLine();
    line.truncate(line.length()-1); //strip newline character
    lineStrList =
      QStringList::split(' ',line);
    strPtr = lineStrList.begin();

#ifdef GRASPITDBG
    std::cout <<"Command parser line: "<<line << std::endl;
#endif
    
    if (*strPtr == "getContacts") {
      strPtr++; if (strPtr == lineStrList.end()) continue;
      numData = (*strPtr).toInt(&ok); strPtr++;
      if (!ok) continue;

#ifdef GRASPITDBG
      std::cout << "Num data: "<<numData<<std::endl;
#endif

      if (readBodyIndList(bodyVec)) continue;
      numBodies = bodyVec.size();
      for (i=0;i<numBodies;i++)
        sendContacts(bodyVec[i],numData);
    }
    
    else if (*strPtr == "getAverageContacts") {
      strPtr++;
      if (readBodyIndList(bodyVec)) continue;
      numBodies = bodyVec.size();
      for (i=0;i<numBodies;i++)
	sendAverageContacts(bodyVec[i]);
    }
    
    else if (*strPtr == "getBodyName") {
      strPtr++;
      if (readBodyIndList(bodyVec)) continue;
      numBodies = bodyVec.size();
      for (i=0;i<numBodies;i++)
	sendBodyName(bodyVec[i]);
    }
    else if(*strPtr == "setBodyName") {
      strPtr++;
      int body_index;
      if(strPtr != lineStrList.end()){
        body_index = strPtr->toInt(&ok);
        strPtr++;
        if(strPtr == lineStrList.end())
          return;
        if (body_index == -1 || body_index >= graspItGUI->getIVmgr()->getWorld()->getNumBodies())
        {
          body_index = graspItGUI->getIVmgr()->getWorld()->getNumBodies() - 1;          
        }
        graspItGUI->getIVmgr()->getWorld()->getBody(body_index)->setName(*strPtr);
      }
    }
    
    else if (*strPtr == "getRobotName") {
      strPtr++;
      if (readRobotIndList(robVec)) continue;
      numRobots = robVec.size();
      for (i=0;i<numRobots;i++)
	sendRobotName(robVec[i]);
    }
    
    else if (*strPtr == "getDOFVals") {
      strPtr++;
      if (readRobotIndList(robVec)) continue;
      numRobots = robVec.size();
      for (i=0;i<numRobots;i++)
	sendDOFVals(robVec[i]);
    }
    
    else if (*strPtr == "moveDOFs") {
      strPtr++;
      readDOFVals();
    }
    
    else if (*strPtr == "render")
      graspItGUI->getIVmgr()->getViewer()->render();
    
    else if (*strPtr == "setDOFForces") {
      strPtr++;
      if (readRobotIndList(robVec)) continue;
      numRobots = robVec.size();
      for (i=0;i<numRobots;i++)
	if (readDOFForces(robVec[i])==FAILURE) continue;
    }
    
    else if ((*strPtr) == "moveDynamicBodies") {
      strPtr++;
      if (strPtr == lineStrList.end()) ok = FALSE;
      else {
	time = (*strPtr).toDouble(&ok); strPtr++;
      }
      if (!ok)
	moveDynamicBodies(-1);
      else
	moveDynamicBodies(time);
    }
    
    else if (*strPtr == "computeNewVelocities") {

#ifdef GRASPITDBG
      std::cout << "cnv" << std::endl;
#endif

      strPtr++; if (strPtr == lineStrList.end()) continue;
      time = (*strPtr).toDouble(&ok); strPtr++;
      if (!ok) continue;

#ifdef GRASPITDBG
      std::cout << time <<std::endl;
#endif
      computeNewVelocities(time);
    }    
    else if ((*strPtr) == "outputPlannerResults"){      
      strPtr++;
      outputPlannerResults(0);      
    }
    else if ((*strPtr) == "outputCurrentGrasp"){      
      strPtr++;
      outputCurrentGrasp();
    }    
    else if ((*strPtr) == "sendBodyTransf"){            
      strPtr++;
      verifyInput(1);
      sendBodyTransf();
    }
    else if ((*strPtr) == "setBodyTransf"){            
      strPtr++;
      verifyInput(7);
      setBodyTransf();
    }
    else if ((*strPtr) == "addObstacle"){            
      strPtr++;
      verifyInput(1);
      addObstacle(*(strPtr+1));
      strPtr+=2;
    }
    else if ((*strPtr) == "addObject"){
      verifyInput(2);
      addGraspableBody(*(strPtr+1), *(strPtr+2));
      strPtr+=3;
      verifyInput(7);
      transf object_pose;
      readTransf(&object_pose);
      World * w = graspItGUI->getIVmgr()->getWorld();
      w->getGB(w->getNumGB() - 1)->setTran(object_pose);
    }
    
    else if ((*strPtr) == "getCurrentHandTran"){
      strPtr++;
      getCurrentHandTran();
    }
    else if ((*strPtr) == "signalGraspUnreachable"){
      strPtr+=4;
      std::cout << line.toStdString() << std::endl;
      graspItGUI->getIVmgr()->blinkBackground();
    }
    else if ((*strPtr) == "setBackgroundColor"){
      ++strPtr;
      bool ok;
      double r = strPtr->toDouble(&ok);
      ++strPtr;
      double g = strPtr->toDouble(&ok);
      ++strPtr;
      double b = strPtr->toDouble(&ok);
      ++strPtr;
      graspItGUI->getIVmgr()->getViewer()->setBackgroundColor(SbColor(r,g,b));
    }

    else if ((*strPtr) == "getPlannerTarget"){
      strPtr+=1;
      QTextStream os (this) ;
      os << graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getObject()->getName() << "\n";
    } 
    else if ((*strPtr) == "setPlannerTarget"){
      QTextStream os(this);
      os << setPlannerTarget(*(strPtr+1)) << "\n";
      strPtr+=2;

    }    

    else if ((*strPtr) == "rotateHandLat"){
      strPtr+=1;
      rotateHandLat();
    }
    else if ((*strPtr) == "rotateHandLong"){
      strPtr+=1;
      rotateHandLong();
    }    
    else if ((*strPtr) == "exec"){
      strPtr+=1;
      exec();
    } 
    else if ((*strPtr) == "next"){
      strPtr+=1;
      next();
    }
    else if ((*strPtr) == "addPointCloud")
    {
      strPtr += 1;
      addPointCloud();
      //QTextStream os(this);  
      //os << addPointCloud() <<" \n";

    }
    else if ((*strPtr) == "setCameraOrigin")
      {
	strPtr += 1;
	setCameraOrigin();
      }
    else if ((*strPtr) == "removeBodies"){
      strPtr += 1;
      removeBodies();
    }
    else if ((*strPtr) == "clearGraspableBodies"){
      strPtr += 1;
      removeBodies(true);
            
    }
    else if ((*strPtr) == "setGraspAttribute"){
      	strPtr += 1;
	verifyInput(3);
	setGraspAttribute();
      	
    }
    else if ((*strPtr) == "drawCircle"){
      strPtr += 1;
	    drawCircle();  
    }
    else if ((*strPtr) == "drawCursor"){
      strPtr += 1;
        drawCursor();
    }
    else if ((*strPtr) == "connectToPlanner")
    {
      connect(graspItGUI->getIVmgr(), SIGNAL( analyzeApproachDir(GraspPlanningState *) ), this, SLOT(analyzeApproachDir(GraspPlanningState*)));
      connect(graspItGUI->getIVmgr(), SIGNAL( analyzeGrasp(const GraspPlanningState *) ), this, SLOT(analyzeGrasp(const GraspPlanningState*)));
      connect(graspItGUI->getIVmgr(), SIGNAL( analyzeNextGrasp() ), this, SLOT(analyzeNextGrasp()));       
      connect(graspItGUI->getIVmgr(), SIGNAL( processWorldPlanner(int) ), this, SLOT( outputPlannerResults(int)));
      connect(graspItGUI->getIVmgr(), SIGNAL( runObjectRecognition() ), this, SLOT( runObjectRecognition() ));
      connect(graspItGUI->getIVmgr(), SIGNAL( sendString(const QString &) ), this, SLOT( sendString(const QString &) ));	  
      QTextStream os(this);
      os << "1 \n";
      os.flush();
    }
    else if ((*strPtr) =="setRobotColor")
    {
      setRobotColor();
    }
  }
}

void ClientSocket::drawCircle()
{
  QString circleName = *strPtr;
  strPtr++;
  bool ok;
  double x = .9*(2*strPtr->toDouble(&ok) - 1);
  strPtr++;
  double y = .9*(2*strPtr->toDouble(&ok) - 1);
  strPtr++;
  double radius = strPtr->toDouble(&ok);
  strPtr++;
  double r = strPtr->toDouble(&ok);
  strPtr++;
  double g = strPtr->toDouble(&ok);
  strPtr++;
  double b = strPtr->toDouble(&ok);
  strPtr++;
  double thickness = strPtr->toDouble(&ok);
  strPtr++;
  double transparency = strPtr->toDouble(&ok);
  
  SbColor circleColor(r,g,b);
  
  //graspItGUI->getIVmgr()->drawCircle(circleName, x, y, radius, circleColor, thickness, transparency);
}

void ClientSocket::drawCursor()
{
  bool ok;
  double x = (strPtr->toDouble(&ok));
  strPtr++;
  double y = (strPtr->toDouble(&ok));

  BCIService::getInstance()->emitCursorPosition(x,y);
}

void ClientSocket::setGraspAttribute()
{		
  double graspIdentifier = strPtr->toDouble();
  strPtr += 1;
  QString attributeString = *strPtr;
  strPtr += 1;
  double value = strPtr->toDouble();
  strPtr += 1;
  if (!currentWorldPlanner())
    return;
  QMutexLocker lock(&currentWorldPlanner()->mListAttributeMutex);
  for(int i = 0; i < currentWorldPlanner()->getListSize(); i++ )
    {
      const GraspPlanningState * gs = currentWorldPlanner()->getGrasp(i);
    if (gs->getAttribute("graspId") == graspIdentifier)
  	  {

        currentWorldPlanner()->setGraspAttribute(i,attributeString, value);
      std::cout << "SetGraspAttribute graspId " << graspIdentifier << " attributeString " << value << "\n";

    }
  }
  lock.unlock();
  analyzeNextGrasp();
}

/*!
  Given a pointer to a body, this examines all the contacts on that body and
  finds the average wrench acting on the body through those contacts and the
  average contact location in body coordinates.  This is written to the
  socket on 2 separate lines.
*/
void
ClientSocket::sendAverageContacts(Body* bod)
{
  QTextStream os(this);
  std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  int i,numContacts;
  double totalWrench[6]={0.0,0.0,0.0,0.0,0.0,0.0};
  double *wrench;
  vec3 totalLoc = vec3::ZERO;

  numContacts = bod->getNumContacts();
  contactList = bod->getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++) {
    wrench = (*cp)->getDynamicContactWrench();
    for (i=0;i<6;i++) totalWrench[i] += wrench[i];
    totalLoc += (*cp)->getContactFrame().translation();
  }

  if (numContacts>1) {
    for (i=0;i<6;i++) totalWrench[i] /= numContacts;
    totalLoc = totalLoc / numContacts;
  }

  os << totalWrench[0]<<" "<<totalWrench[1]<<" "<<totalWrench[2]<<
    " "<<totalWrench[3]<<" "<<totalWrench[4]<<" "<<totalWrench[5]<<"\n";
  os << totalLoc[0] << " "<<totalLoc[1] << " " <<totalLoc[2]<<"\n";

}

/*!
  Given a pointer to a body, this writes the name of the body and a newline
  to the socket.
*/
void
ClientSocket::sendBodyName(Body* bod)
{
  QTextStream os(this);
  std::cout << "sending " << bod->getName().latin1() << "\n";
  os << bod->getName().latin1() << "\n";
}

/*!
  Given a pointer to a robot, this writes the name of the robot and a newline
  to the socket.
*/
void
ClientSocket::sendRobotName(Robot* rob)
{
  QTextStream os(this);
  std::cout << "sending " << rob->getName().latin1() << "\n";
  os << rob->getName().latin1() << "\n";
}

/*!
  Given a pointer to a body, this first writes a line containing the number
  of contacts on the body.  Then it writes \a numData lines for each contact.
  The first line contains the 6 numbers of the contact wrench.  The next line
  (if required) contains the 3 numbers specifying the contact location in
  body coordinates, and the last line (if required) contains the scalar
  constraint error for that contact.
*/
void
ClientSocket::sendContacts(Body *bod,int numData)
{
  QTextStream os(this);
  std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  vec3 loc;
  double err;
  double *wrench;

#ifdef GRASPITDBG
  std::cout << "sending numContacts: "<<bod->getNumContacts()<<std::endl;
#endif

  os << bod->getNumContacts()<<"\n";

  contactList = bod->getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++) {
	wrench = (*cp)->getDynamicContactWrench();
	loc = (*cp)->getContactFrame().translation();
	err = (*cp)->getConstraintError();

    os << wrench[0]<<" "<<wrench[1]<<" "<<wrench[2]<<" "<<wrench[3]<<" "<<
      wrench[4]<<" "<<wrench[5]<<"\n";
	if (numData > 1) 
	  os << loc[0] << " "<< loc[1] << " " << loc[2]<<"\n";
	if (numData > 2)
	  os << err << "\n";
  }
}

/*!
  Given a pointer to robot, this first writes a line to the socket containing
  the number of %DOF's in the robot, then for each %DOF, it writes a line
  containting the current value of that %DOF.
*/
void
ClientSocket::sendDOFVals(Robot *rob)
{
  QTextStream os(this);
  int i;

  os << rob->getNumDOF() << "\n";
  for (i=0;i<rob->getNumDOF();i++)
    os << rob->getDOF(i)->getVal() << "\n";
}

/*!
  After the readDOFVals command was read by readClient, this expects to
  read a valid robot index, then the correct number of %DOF's for this robot,
  then a desired value for each %DOF, and finally a value for each DOF to step
  by during the move.  It performs the %DOF moves, finds the new contacts,
  updates the grasp, and then it sends one line for each %DOF containing
  the actual value for the %DOF after the move.
*/
int
ClientSocket::readDOFVals()
{
  Robot *rob;
  double *val,*stepby;
  QTextStream os(this);
  int numDOF,i,robNum;
  bool ok=TRUE;

#ifdef GRASPITDBG
  std::cout << "in read dof vals"<<std::endl;
#endif

  if (strPtr == lineStrList.end()) ok=FALSE;
  if (ok) robNum = (*strPtr).toInt(&ok);

  if (!ok || robNum < 0 ||
    robNum >= graspItGUI->getIVmgr()->getWorld()->getNumRobots()) {
	os <<"Error: Robot does not exist.\n";
    return FAILURE;
  }
  rob = graspItGUI->getIVmgr()->getWorld()->getRobot(robNum);

#ifdef GRASPITDBG
  std::cout << "robnum: "<<robNum<<std::endl;
#endif

  strPtr++;
  if (strPtr == lineStrList.end()) return FAILURE;

  numDOF=(*strPtr).toInt(&ok);
  if (!ok) return FAILURE;
  strPtr++;

#ifdef GRASPITDBG
  std::cout << "read robot has: "<< numDOF << " DOF?"<<std::endl;
#endif

  if (numDOF < 1) {

#ifdef GRASPITDBG
	std::cout << "numDOF was zero."<<std::endl;
#endif
	return FAILURE;
  }
  if (numDOF != rob->getNumDOF()) {
    os <<"Error: robot has " << rob->getNumDOF() <<" DOF."<<endl;
	return FAILURE;
  }

  val = new double[numDOF];
  stepby = new double[numDOF];

  for (i=0;i<rob->getNumDOF();i++) {
	if (strPtr == lineStrList.end()) return FAILURE;
	val[i] = (*strPtr).toDouble(&ok);
	if (!ok) return FAILURE;
	strPtr++;
#ifdef GRASPITDBG
	std::cout<<val[i]<<" ";
#endif
  }

#ifdef GRASPITDBG
  std::cout<<std::endl;
#endif

  for (i=0;i<rob->getNumDOF();i++) {
    if (strPtr == lineStrList.end()) return FAILURE;
	stepby[i] = (*strPtr).toDouble(&ok);
	if (!ok) return FAILURE;
	strPtr++;
  }

  rob->moveDOFToContacts(val,stepby,true);
  
  // these should be separate commands
  graspItGUI->getIVmgr()->getWorld()->findAllContacts();
  graspItGUI->getIVmgr()->getWorld()->updateGrasps();

  for (i=0;i<rob->getNumDOF();i++) {
    os << rob->getDOF(i)->getVal() << "\n";

#ifdef GRASPITDBG
    std::cout << "Sending: "<< rob->getDOF(i)->getVal() << "\n";
#endif
  }
  delete [] val;
  delete [] stepby;
  return SUCCESS;
}

/*!
  After the readDOFForces command was read by readClient, this expects to
  the correct number of %DOF's for this robot, and then a desired force for
  each %DOF.  It sets each %DOF force and sends a line for each containing
  the current force.
*/
int
ClientSocket::readDOFForces(Robot *rob)
{
  double val;
  bool ok;
 // QTextStream is(this);
  QTextStream os(this);
  int numDOF,i;

  if (strPtr == lineStrList.end()) return FAILURE;
 
  numDOF=(*strPtr).toInt(&ok);
  if (!ok) return FAILURE;
  strPtr++;

#ifdef GRASPITDBG
  std::cout << "read robot has: "<< numDOF << " DOF?"<<std::endl;
#endif

  if (numDOF < 1) {
#ifdef GRASPITDBG
    std::cout << "numDOF was zero."<<std::endl;
#endif
    return FAILURE;
  }
  
  if (numDOF != rob->getNumDOF()) {
    os <<"Error: robot has " << rob->getNumDOF() <<" DOF."<<endl;
    return FAILURE;
  }
  
  for (i=0;i<rob->getNumDOF();i++) {
    if (strPtr == lineStrList.end()) return FAILURE;
    val = (*strPtr).toDouble(&ok);
    if (!ok) return FAILURE;
    strPtr++;
    rob->getDOF(i)->setForce(val);
    
#ifdef GRASPITDBG
    std::cout<<val<<" ";
#endif
  }
  
#ifdef GRASPITDBG
  std::cout<<std::endl;
#endif
  
  for (i=0;i<rob->getNumDOF();i++) {
    os << rob->getDOF(i)->getForce() << "\n";
    
#ifdef GRASPITDBG
    std::cout << "Sending: "<< rob->getDOF(i)->getForce() << "\n";
#endif
  }
  return SUCCESS;
}

/*
//not finished yet
void
ClientSocket::moveBody(Body *bod)
{
  double x,y,z,ax,ay,az,r;

  QTextStream is(this);
  is>>x>>y>>z>>ax>>ay>>az>>r;
  bod = NULL;  // unused parameter warning
}
*/

/*!
  If given a positive time step value, this will move the dynamic world bodies
  with that value.  Otherwise it uses the world default time step.  A line
  with the actual length of that timestep is then sent back.
*/
void
ClientSocket::moveDynamicBodies(double timeStep)
{
  QTextStream os(this);
  if (timeStep<0)
    timeStep = graspItGUI->getIVmgr()->getWorld()->getTimeStep();

  double actualTimeStep =
    graspItGUI->getIVmgr()->getWorld()->moveDynamicBodies(timeStep);
  if (actualTimeStep < 0)
    os << "Error: Timestep failsafe reached.\n";
  else 
    os << actualTimeStep << "\n";
}

/*!
  This calls the computeNewVelocities routine in the dynamics with the given
  value of the timestep.  Afterwards, it sends out a line containing the
  result code from that operation.
*/
void
ClientSocket::computeNewVelocities(double timeStep)
{
  QTextStream os(this);
  int result = graspItGUI->getIVmgr()->getWorld()->computeNewVelocities(timeStep);
  os << result << "\n";
}


void
ClientSocket::updatePlannerParams(QStringList & qsl)
{
  
  /*parse arguments */
  //confidence arguments
  
  
  return;
}


void
ClientSocket::outputPlannerResults(int solution_index)
{
  QTextStream os(this);
  //Test for existence of planner
  if(!currentWorldPlanner()){
    os << "No Planner Set \n";
    return;
  }
  //Iterate through solutions list outputing each one
  //WARNING:: THIS IS NOT REALLY THREADSAFE BECAUSE THE PLANNER KEEPS RUNNING
  //FIXME
  //os << currentWorldPlanner()->getListSize() << "\n";
  os << "doGrasp {";
  //  for(int solution_index =0; solution_index < currentWorldPlanner()->getListSize(); ++solution_index)
  // {
  os << *(currentWorldPlanner()->getGrasp(solution_index)) << ',';      
      // }
  os << '}';
  os << '\n';
  std::cout << "sent grasp\n";
  return;
}

/*Signal object recognition to run. 
*/
void ClientSocket::runObjectRecognition()
{
  QTextStream os(this);
  os << "runObjectRecognition \n";
  return;
}

void ClientSocket::sendString(const QString & message)
{
  QTextStream os(this);
  os << message << "\n";
  os.flush();
  flush();
  return;
}

void 
ClientSocket::outputCurrentGrasp()
{
  QTextStream os(this);
  GraspPlanningState g(graspItGUI->getIVmgr()->getWorld()->getCurrentHand());
  g.setPostureType(POSE_DOF, false);
  g.saveCurrentHandState();
  os << "doGrasp{";
  os << g <<',';
 
  os << '}';
  os << '\n';
  std::cout << "sent grasp: " << g << " \n";
 
}



/*!
  This is not complete yet.

void
ClientSocket::readTorques()
{
  QString line;
  line = readLine();
  int numDOF = line.toInt();
  
  if (numDOF < 0) {} //unused parameter warning
}
*/

/*!
  Starts a TCP server that listens on port \a port.  \a backlog specifies
  the number of pending connections the server can have.
*/
GraspItServer::GraspItServer(Q_UINT16 port, int backlog,
			     QObject *parent,const char *name) : 
  Q3ServerSocket(port,backlog,parent,name)
{
  if (!ok()) {
    qWarning("Failed to bind to port");
  }
}

/*! 
  Creates a new ClientSocket to handle communication with this client.
*/
void
GraspItServer::newConnection(int socket)
{
  (void)new ClientSocket(socket, this);
  


#ifdef GRASPITDBG
  std::cout << "new connection" << std::endl;
#endif
}


//Body transforms are sent backwards by the planner for some reason compared to the overloading
//of << for transfs
void ClientSocket::setBodyTransf(){
  //FIXME add checking for badly formatted input here.
	std::vector <Body *> bd;
	readBodyIndList(bd);
	transf object_pose;
	for(std::vector<Body *>::iterator bp = bd.begin(); bp != bd.end(); ++bp){
		readTransf(&object_pose);
		(*bp)->setTran(object_pose);
	}
	QTextStream os(this);
	os << "1 \n";
	os.flush();
}

void ClientSocket::sendBodyTransf(){
	std::vector <Body *> bd;
	readBodyIndList(bd);
	QTextStream os(this);
	for(std::vector<Body *>::iterator bp = bd.begin(); bp != bd.end(); ++bp){
		//this is a hack around the overloading for standard strings and not qstrings.  
		//this should be templated
	  std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << (*bp)->getTran().translation() << (*bp)->getTran().rotation() << "\n";
		std::cout << ss.str() << "\n";
		os << QString(ss.str().c_str());
	}
	os << "\n";
	os.flush();
	std::cout << "Body list size:"<< bd.size() << "\n";
}

void ClientSocket::removeBodies(bool graspable){
  std::vector <Body *> bd;
  readBodyIndList(bd);
  QTextStream os(this);
  for(std::vector<Body *>::iterator bp = bd.begin(); bp != bd.end(); ++bp){
    if(!graspable || (*bp)->inherits("GraspableBody") )
      graspItGUI->getIVmgr()->getWorld()->destroyElement(*bp, true);    
  }
  os << 1 << '\n';
}

Body *addToWorld(const QString & relative_model_dir, const QString & model_type, const QString & model_filename)
{
 QString body_file = QString(getenv("GRASPIT")) + "/" +  relative_model_dir + model_filename;
 std::cout << "body string: "<< body_file.toStdString() << std::endl;
 return graspItGUI->getIVmgr()->getWorld()->importBody(model_type, body_file);
}

void ClientSocket::addGraspableBody(const QString & bodyName, const QString & objectName){
  QTextStream os(this);
  Body * b = addToWorld(QString("models/objects/"), "GraspableBody", bodyName);
  if(!b)
     b = addToWorld("models/object_database/", QString("GraspableBody"), bodyName);
  os << (b!=NULL) << " \n";  
  os.flush();
  std::cout << "done adding graspable body \n";
  b->setName(objectName);
}


void ClientSocket::addObstacle(const QString & bodyName){
  QTextStream os(this);
  os << (addToWorld("models/obstacles/", "Body", bodyName)!=NULL) << '\n';
}

void ClientSocket::getCurrentHandTran()
{
  QTextStream os(this);
  os << graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran() << "\n";
  
}

bool ClientSocket::verifyInput(int minimum_arg_number){
  // Method one, copy the iterator and test manually
  QStringList::const_iterator strPtr_copy = strPtr;
  for (int iter = 0; iter < minimum_arg_number; ++iter)
    {
      if (strPtr_copy == lineStrList.end()){
	QTextStream os(this);
	os << 0 << '\n';
	std::cout << "verifyInput failed \n";
	return false;
      }
      strPtr_copy++;
    }
  if (strPtr == lineStrList.end())
    std::cout << "copying pointers doesn't work right \n";
  return true;
}


bool ClientSocket::rotateHandLat()
{
    BCIService::getInstance()->emitRotLat();
    return true;
}

bool ClientSocket::rotateHandLong()
{
    BCIService::getInstance()->emitRotLong();
    return true;
}

bool ClientSocket::exec()
{
    BCIService::getInstance()->emitExec();
    return true;
}

bool ClientSocket::next()
{
    BCIService::getInstance()->emitNext();
    return true;
}

bool ClientSocket::setPlannerTarget(const QString & bodyName)
{  
  
  World * w = graspItGUI->getIVmgr()->getWorld();
  //remove the current planner target if it exists
  if(w->getCurrentHand())
  {
    GraspableBody * currentObject = w->getCurrentHand()->getGrasp()->getObject();

    if(currentObject)
      {
        w->getCurrentHand()->getGrasp()->setObject(NULL);
        w->destroyElement(currentObject);
      }
  }
  if (!addToWorld("models/objects/", QString("GraspableBody"), bodyName) && !addToWorld("models/object_database/", QString("GraspableBody"), bodyName))  
    return false; // failed to add body to world. 
  
  
  //The body has been added. Now load the table obstacle if necessary  
  //Find table obstacle and move it down
  Body * b = NULL;
  for(int i = 0; i < w->getNumBodies(); ++i)
  {
    if(w->getBody(i)->getName() == "experiment_table")
      b = w->getBody(i);
  }
  if(b == NULL)
    b = addToWorld("models/objects/", "Body", "experiment_table.xml");
  if(b == NULL)
      return false;
  //move obstacle down below object

  // Get most recent GB, which we have just added
  GraspableBody * gb = w->getGB(w->getNumGB() - 1);
  std::vector<position> vertices;
	gb->getGeometryVertices(&vertices);
  double zmin = 0;
  for(unsigned int iv = 0; iv < vertices.size(); ++iv)
  {
    if(zmin > vertices[iv].z())
    zmin = vertices[iv].z();
  }
  int collisionThreshold = 1;
  transf t(mat3::IDENTITY,vec3(0,0,zmin - collisionThreshold));      
  b->setTran(t);
  //Disable collisions between main hand and the experiment table
  w->toggleCollisions(false, w->getCurrentHand(), b);
  //w->emitTargetBodyChanged(gb);
  return true;  

}

double convertToNumber(const QStringList::const_iterator & strPtr, QStringList * lineStrList, bool & ok)
{  
  double d = -1;
  ok = false;
  if(strPtr != lineStrList->end())
    d = strPtr->toDouble(&ok);
  return d;
}

SoSeparator * newSphere(double x, double y, double z)
{
  SbMatrix tr = SbMatrix::identity();
  tr.setTranslate(SbVec3f(x,y,z));
  SoTransform * t = new SoTransform;
  t->setMatrix(tr);
  SoSeparator * sphere_sep = new SoSeparator;
  SoMaterial * black = new SoMaterial;
  SoMFColor black_color;
  black_color.setValue(0.0,0.0,0.0);
  black->ambientColor = black_color;
  black->specularColor = black_color;
  black->emissiveColor = black_color;
  black->transparency = 0.8f;
  sphere_sep->addChild(black);
  sphere_sep->addChild(t);
  SoSphere * sphere = new SoSphere;
  sphere->radius = 10;
  sphere_sep->addChild(sphere);
  return sphere_sep;
}

bool ClientSocket::setCameraOrigin()
{
  bool ok = false;
  float x = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
  if (!ok)
    return false;
  float y = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
  if (!ok)
    return false;
  float z = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
  if (!ok)
    return false;
  std::cout << "x y z:" << x << " " <<  y << " " << z << std::endl;
  transf t(mat3::IDENTITY, vec3(x,y,z));
  graspItGUI->getIVmgr()->setCameraTransf(t);
  graspItGUI->getIVmgr()->getViewer()->getCamera()->pointAt(SbVec3f(0,0,0), SbVec3f(0,0,1));
  return true;
}

bool ClientSocket::addPointCloud()
{  
  
  
  bool ok = false;  
  
  bool has_colors = true;

  double pointNum = convertToNumber(strPtr++, &lineStrList, ok);
  if (!ok)
    return false;  
  SoNodeList l;
  SoCoordinate3 * coord;
  SoMaterial * mat;

  unsigned int listLen = SoCoordinate3::getByName("PointCloudCoordinates", l);
  if (listLen < 1)
  {
    SoSeparator * coords_sep = new SoSeparator();
    SoTransform * coord_tran = new SoTransform();
    coord_tran->setName("PointCloudTransform");
    coord = new SoCoordinate3();  
    coord->setName("PointCloudCoordinates");
    SoPointSet * pointSet = new SoPointSet();
    SoDrawStyle * drawStyle = new SoDrawStyle(); 
    coords_sep->addChild(coord_tran);
    coords_sep->addChild(coord);
    mat = new SoMaterial();
    mat->setName("PointCloudColorMaterial");
    SoMaterialBinding * matBinding = new SoMaterialBinding();
    matBinding->value = SoMaterialBinding::PER_PART;    
    
    coords_sep->addChild(mat);
    coords_sep->addChild(matBinding);
    
    drawStyle->pointSize = 3;
    coords_sep->addChild(drawStyle);
    coords_sep->addChild(pointSet);
    graspItGUI->getIVmgr()->getWorld()->getIVRoot()->addChild(coords_sep);  
  }  else if (listLen > 1)
  {
    std::cout << "More than 1 Point Cloud coordinate node. What the heck!\n";
  }
  else{
    coord = static_cast<SoCoordinate3 *>(l[0]);
    SoNodeList l2;
    unsigned int listLen2 = SoMaterial::getByName("PointCloudColorMaterial", l2);
    if (listLen2 != 1){
      std::cout << "Wrong number of Point Cloud Materials: " << listLen2 << "\n";
      return false;
    }
    mat = static_cast<SoMaterial *>(l2[0]);
  }

  std::vector<SbVec3f> points;
  std::vector<SbColor> colors;    
  points.reserve(pointNum);
  colors.reserve(pointNum);
  //std::vector<SoSeparator *> points(pointNum);
  
  for(int pointIndex = 0; pointIndex < pointNum; ++pointIndex)
  {
    double x = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
    if (!ok)
      return false;
    double y = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
    if (!ok)
      return false;
    double z = convertToNumber(strPtr++, &lineStrList, ok)*1000;    
    if (!ok)
      return false;      

    points.push_back(SbVec3f(x,y,z));
  
    
    
  
    double r = convertToNumber(strPtr++, &lineStrList, ok)/255.0;    
    if (!ok)
      return false;
      double g = convertToNumber(strPtr++, &lineStrList, ok)/255.0;    
    if (!ok)
      return false;
      double b = convertToNumber(strPtr++, &lineStrList, ok)/255.0;    
    if (!ok)
      return false;

    
    colors.push_back(SbColor(r,g,b));
  }
  
  coord->point.setValues(0,points.size(), &points[0]);  
  mat->diffuseColor.setValues(0,colors.size(), &colors[0]);            
  
  return true;
}


void ClientSocket::analyzeGrasp(const GraspPlanningState * gps)
{  
	QTextStream os(this);
	os << "analyzeGrasp " << gps->getAttribute("graspId") << " " << *gps << "\n";
  std::cout << "analyzeGrasp " << gps->getAttribute("graspId") << " " << *gps << "\n";
}

void ClientSocket::analyzeNextGrasp()
{  
  std::cout << "Emitted analyze next grasp\n";  
  QMutexLocker lock(&currentWorldPlanner()->mListAttributeMutex);
  if(currentWorldPlanner())
  {  
  for(int i = 0; i < currentWorldPlanner()->getListSize(); ++i)
  {
    const GraspPlanningState * gs = currentWorldPlanner()->getGrasp(i);    
    if(gs->getAttribute("testResult") == 0.0)
    {
    if(gs->getAttribute("testTime") >  QDateTime::currentDateTime().toTime_t() - 10)
       return;
    }
  }
  for(int i = 0; i < currentWorldPlanner()->getListSize(); ++i){
    const GraspPlanningState * gs = currentWorldPlanner()->getGrasp(i);
    if(gs->getAttribute("testResult") == 0.0)
    {
      currentWorldPlanner()->setGraspAttribute(i, "testTime",  QDateTime::currentDateTime().toTime_t());
      analyzeGrasp(gs);
      break;
    }
  }
  
  }
}
   


bool ClientSocket::setRobotColor()
{
    ++strPtr;
    bool ok = false;
    verifyInput(4);
  
    std::vector<Robot *> robVec;
    readRobotIndList(robVec);


    double r = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;
    double g = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;
    double b = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;

    SbColor color(r,g,b);
    SoMFColor colorField;
    colorField.setValuesPointer(3,&color);

   
    for(unsigned int i = 0; i < robVec.size(); ++i)
    {
        robVec[i]->setEmissiveColor(color);
    }

    return true;


}

void ClientSocket::analyzeApproachDir(GraspPlanningState * gs)
{
	QTextStream os(this);
	os << "analyzeApproachDirection " << *gs << " \n";
  os.flush();
  delete gs;
}

bool ClientSocket::setBodyColor()
{
    bool ok = false;
    verifyInput(5);
    std::vector<Robot *> robVec;
    readRobotIndList(robVec);

    double bodyNum = convertToNumber(strPtr++, &lineStrList, ok)*1000;
    if (!ok)
      return false;
    double r = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;
    double g = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;
    double b = convertToNumber(strPtr++, &lineStrList, ok);
    if (!ok)
      return false;

    SbColor color(r,g,b);

    World * world = graspItGUI->getIVmgr()->getWorld();
    world->getBody(bodyNum)->setEmissiveColor(color);

    return true;
}
