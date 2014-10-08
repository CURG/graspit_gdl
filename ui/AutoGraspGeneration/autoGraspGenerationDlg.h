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

#include <dirent.h>

#include <QDialog>

#include "ui_autoGraspGenerationDlg.h"
#include "BCI/handViewWindow.h"
#include <vector>
#include <QtGui/QShortcut>
#include "DBase/DBPlanner/sql_database_manager.h"
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

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

class World;
class Robot;


/*! The EGPlannerDialog can interface to most of the types of EGPlanners, 
	pass them any type of HandObjectState as a starting position, accept 
	changes to the Input provided to the Planner, etc. This is the primary 
	way of getting familiar with the various implementations of EGPlanner.
*/
class AutoGraspGenerationDlg : public QDialog, public Ui::autoGraspGenerationDlgUI
{
	Q_OBJECT
private:

    //-----------------------
    //Grasp Generation Settings that need UI elements created for them

    //how long to sample at each mesh point
    int millisecondsPerMeshPoint;

    //how many meshpoints to skip
    int meshPointIncrement;

    int currentHand;
    int currentModel;

    //where to save the generated grasps
    std::string grasp_dir;
    //------------------------

    //current meshpoint we are using to position the seed hand
    int currentMeshPointIndex;

    QTimer *seedHandMovementTimer;

    //The mesh we are generating grasps for as a cloud with normals
    pcl::PointCloud<pcl::PointNormal> cloud_with_normals;

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

    void moveHandToNextPose();
    void saveGrasps();
    void generateHandPoses();
    void chooseNewScene(int handId, int modelId);

    QString modelsDirName;
    QString handsDirName;
    std::vector<QString> modelXMLNames;
    std::vector<QString> handXMLNames;
    World *world;
    Robot *rob;
    Body *obj;
  
public:
  AutoGraspGenerationDlg(QWidget *parent = 0) : 
        QDialog(parent)

  {
      setupUi(this);
      init();
  }
	
    ~AutoGraspGenerationDlg(){destroy();}

public slots:
	void exitButton_clicked();
	void setMembers( Hand *h, GraspableBody *b );
    void setWorld( World *w );
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

    void loadModelsAndHandsButton_clicked();
    void loadHandsDirButton_clicked();
    void startAutoGraspButton_clicked();



};

#endif
