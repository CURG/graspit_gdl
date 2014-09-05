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
// $Id: egPlannerDlg.h,v 1.5.2.1 2009/07/23 21:18:13 cmatei Exp $
//
//######################################################################

#ifndef _autograspgenerationdlg_h_
#define _autograspgenerationdlg_h_

#include <QDialog>

#include "ui_autoGraspGenerationDlg.h"
#include "BCI/handViewWindow.h"
#include <vector>
#include <QtGui/QShortcut>
#include "DBase/DBPlanner/sql_database_manager.h"
#include <math.h>


class QGridLayout;
class QCheckBox;
class QVBoxLayout;
class QCheckBox;
class QLabel;
class QHBoxLayout;
class QSlider;
class QLabel;

class GraspPlanningState;
class GraspableBody;
class Hand;
class EGPlanner;
class HandViewWindow;


Quaternion eulerToQuaternion(double roll, double pitch, double yaw);


/*! The EGPlannerDialog can interface to most of the types of EGPlanners, 
	pass them any type of HandObjectState as a starting position, accept 
	changes to the Input provided to the Planner, etc. This is the primary 
	way of getting familiar with the various implementations of EGPlanner.
*/
class AutoGraspGenerationDlg : public QDialog, public Ui::autoGraspGenerationDlgUI
{
	Q_OBJECT
private:
  int resetCount;
  int currentHandPositionIndex;
  std::vector<vec3> handPositions;

  QGridLayout *varGridLayout;
  QVBoxLayout *varMainLayout;
  QShortcut * initShortcut;
  std::vector<QCheckBox*> varInput;
  std::vector<QCheckBox*> varCheck;
  std::vector<QLabel*> varNames;
  std::vector<QHBoxLayout*> varLayouts;
  std::vector<QSlider*> varConfidence;
  std::vector<QLabel*> varTarget;
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  int mDisplayState;
  EGPlanner *mPlanner;
  void init();
  void destroy();
  void setVariableLayout();
  void updateVariableLayout();
  void updateInputLayout();
  void readPlannerSettings();
  void startPlanner();
  void stopPlanner();
  QTime timer;
  QWidget * circleDraw;
  void initializeTarget();
  void loadGraspsToHandviewWindow();
  void initializeDbInterface();
  void initializeHandviewWindow();


  
public:
  AutoGraspGenerationDlg(QWidget *parent = 0) : 
        QDialog(parent)

  {
      setupUi(this);
      init();
      resetCount = 0;
      currentHandPositionIndex = 0;
      std::cout<< "adding hand position";
      double numSteps = 3.0;
      for(int x_index=-numSteps+1; x_index < numSteps+1; x_index++){
          double x = x_index / numSteps * 220;
          for(int y_index=-numSteps+1; y_index < numSteps+1; y_index++){
              double y = y_index / numSteps * 220;
              for(int z_index=-numSteps+1; z_index < numSteps+1; z_index++){
                  double z = z_index / numSteps * 220;
                  if(abs(x_index)+abs(y_index)+abs(z_index) > 4)
                  {
                      std::cout<< "adding hand position:(" << x <<"," << y << ","<< z <<")";
                      handPositions.push_back(vec3(x,y,z));
                  }
              }
          }
      }

	}
	
    ~AutoGraspGenerationDlg(){destroy();}

public slots:
	void exitButton_clicked();
	void setMembers( Hand *h, GraspableBody *b );
	void variableInputChanged();
	void variableCheckBoxChanged();
	void spaceSearchBox_activated( const QString &s );
	void prevGraspButton_clicked();
	void bestGraspButton_clicked();
	void nextGraspButton_clicked();
	void executeGraspButton_clicked();
	void plannerUpdate();
	void updateResults(bool render, bool execute);
	void updateStatus();
	void energyBox_activated( const QString & );
	void setContactsBox_toggled( bool checked);
	void plannerComplete();
	void plannerInit_clicked();
	void plannerReset_clicked(); 
	void plannerStart_clicked();
    void updateObject(GraspableBody *b);
	void plannerTypeBox_activated( const QString & );
	void autoGraspBox_clicked();
	void onlinePlannerUpdate();
	void onlineGraspButton_clicked();
	void onlineReleaseButton_clicked();
	void onlinePlanButton_clicked();
	void instantEnergyButton_clicked();
	void showCloneBox_toggled( bool c);
	void showSolutionBox_toggled( bool c);
	void useVirtualHandBox_clicked();
	void useRealBarrettBox_toggled( bool s);
	void inputGloveBox_toggled( bool on);
	void inputLoadButton_clicked();
    void timerUpdate();



};

#endif
