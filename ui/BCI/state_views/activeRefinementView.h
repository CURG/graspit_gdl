#ifndef ACTIVEREFINEMENTVIEW_H
#define ACTIVEREFINEMENTVIEW_H

#include <QWidget>
#include "BCI/handView.h"

class GraspPlanningState;

namespace Ui {
class ActiveRefinementView;
}

class ActiveRefinementView : public QWidget
{
    Q_OBJECT
    
public:
    explicit ActiveRefinementView(QWidget *parent = 0);
    void showSelectedGrasp(Hand *hand,const GraspPlanningState *graspPlanningState);
    ~ActiveRefinementView();


public slots:
    void onOk();
    void onRotLat();
    void onRotLong();

    
private:
    HandView *handView;
    Ui::ActiveRefinementView *ui;
    void createHandView();
};

#endif // ACTIVEREFINEMENTVIEW_H
