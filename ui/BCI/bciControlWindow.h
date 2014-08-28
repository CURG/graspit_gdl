#ifndef BCICONTROLWINDOW_H
#define BCICONTROLWINDOW_H




#include <QDialog>
#include "ui_BCIControlWindowBase.h"
#include "BCI/bciService.h"
#include "debug.h"
#include "BCI/onlinePlannerController.h"
class BCIControlWindow: public QDialog, public Ui::BCIControlWindowBase
{

    Q_OBJECT

public:


    BCIControlWindow(QWidget *parent = 0 )
        :QDialog(parent)
    {

        if(OnlinePlannerController::getInstance()->thread() != this->thread())
            DBGA("OnlinePlannerController not in same thread as BCIControlWindow");


        setupUi(this);
    }

};


#endif // BCICONTROLWINDOW_H
