#ifndef GRASPVIEW_H
#define GRASPVIEW_H

#include <QWidget>

class Hand;
class SoQtRenderArea;


namespace Ui {
class GraspView;
}

class GraspView : public QWidget
{
    Q_OBJECT
    
public:
    explicit GraspView(QWidget *parent = 0);
    ~GraspView();


    
private:
    Ui::GraspView *ui;
};

#endif // GRASPVIEW_H
