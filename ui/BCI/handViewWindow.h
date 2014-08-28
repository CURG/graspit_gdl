#ifndef HANDVIEWWINDOW_H
#define HANDVIEWWINDOW_H

#include <Q3MainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QObject>
#include <QtGui/QShortcut>
#include <set>

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <exception>
#include <stdexcept>
#include <typeinfo>

#include <q3listbox.h>
#include <QApplication>
#include <QThread>

#include <QTimer>

#include "ivmgr.h"
#include "robot.h"
#include "joint.h"
#include "humanHand.h"
#include "body.h"
#include "contact.h"
#include "grasp.h"
#include "world.h"
#include "mainWindow.h"
#include "matvec3D.h"
#include "graspitGUI.h"
#include "humanHand.h"
#include "body.h"

#include "BCI/handView.h"

#include "EGPlanner/searchState.h"
class SoQtRenderArea;
class SoQtExaminerViewer;
class SoCoordinate3;
class SoIndexedFaceSet;
class SoSeparator;
class QWidget;
class GWS;
class HandObjectState;
class Hand;

class HandViewWindow{

  //! viewsList
  std::vector<HandView *> views;

  //! The layout for the thumbnail views
  QGridLayout * grid;
  QHBoxLayout * hbox;
  QVBoxLayout * vbox1;
  QVBoxLayout * vbox2;
  //! The container for the layout for the preview (created so that we
  //   can use more than one preview window if so desired with different graphics applied)
  QGridLayout * previewGrid;
  QWidget * stageFrame_;
  //! MaxViewSize - protect against over access on number of loaded thumbnail views
  unsigned int maxViewSize;

  //! The copied hand we use as a temporary storage as we load thumbnail and preview - views
  Hand * cloneHand;

  //! Index of current Handview stored in the previewWindow
  int currentPreviewIndex;

  QRect geom_;

public:
  //! the window containing the thumbnail views
  QFrame * handViewWindow;
  QFrame * viewHolder;
  //! ConstructorNUL
  HandViewWindow(QWidget * parent, Hand * h, const QRect & geom, SoNode * IVRoot = NULL, QWidget * stageFrame = NULL);
  void initViews(Hand * h);
  ~HandViewWindow(){
    for (int i = 0; i < views.size(); ++i)
      delete views[i];
    delete cloneHand;
    delete handViewWindow;
  }
  //! Actually adds views at loop of initialization
  bool addView(HandObjectState & s, int i);
  void clearViews();
  //! Simple getter for the viewWindow associated with our object
  QFrame* getViewWindow(){
    return handViewWindow;
  }

 /* //! To determine which view appears in the preview window
  void updateList();
*/
  //! Skip directly to the desired thumbnail frame
  void setCurrentView(int num);

  //! Returns the current grasp (as seen in the preview window)
  int getCurrentGrasp();
};

#endif // HANDVIEWWINDOW_H
