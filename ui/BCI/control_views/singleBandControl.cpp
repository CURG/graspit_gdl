#include "singleBandControl.h"
#include "ui_singleBandControl.h"
#include <Q3Painter>

SingleBandControl::SingleBandControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SingleBandControl)
{
    cursorPosition = new QPointF(10,10);
    ui->setupUi(this);
}

void SingleBandControl::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setBrush(Qt::SolidPattern);

    QRectF rect1(width()/2.0,height()/2.0, 10,10);
    QRectF rect2(10,10, 10,10);
    QRectF rect3(10,10, 10,10);

    painter.drawRect(rect1);
    //painter.drawRect(rect2);
    //painter.drawRect(rect3);

    painter.drawEllipse(cursorPosition->x(), cursorPosition->y(), 10,10);
}


SingleBandControl::~SingleBandControl()
{
    delete ui;
}
