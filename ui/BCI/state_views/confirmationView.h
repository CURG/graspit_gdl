#ifndef CONFIRMATIONVIEW_H
#define CONFIRMATIONVIEW_H

#include <QWidget>

class GraspPlanningState;
class HandView;
class Hand;

namespace Ui {
class ConfirmationView;
}

class ConfirmationView : public QWidget
{
    Q_OBJECT
    
public:
    explicit ConfirmationView(QWidget *parent = 0);
    void setCurrentGrasp(Hand *hand, const GraspPlanningState *graspPlanningState);
    ~ConfirmationView();
    
public slots:
    void onOk();
    void onBack();

private:
    HandView *handView;
    Ui::ConfirmationView *ui;
};

#endif // CONFIRMATIONVIEW_H
