#ifndef SINGLEBANDCONTROL_H
#define SINGLEBANDCONTROL_H

#include <QWidget>

namespace Ui {
class SingleBandControl;
}

class SingleBandControl : public QWidget
{
    Q_OBJECT
    
public:
    explicit SingleBandControl(QWidget *parent = 0);
    ~SingleBandControl();

protected:
    void paintEvent(QPaintEvent*);
    
private:
    Ui::SingleBandControl *ui;
    QPointF *cursorPosition;

};

#endif // SINGLEBANDCONTROL_H
