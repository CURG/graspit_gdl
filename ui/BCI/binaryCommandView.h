#ifndef BINARYCOMMANDVIEW_H
#define BINARYCOMMANDVIEW_H

#include <QWidget>
#include <QPainter>
#include <map>
#include <string>
class DrawableFrame;
class ShapeDrawable;


namespace Ui {
class BinaryCommandView;
}

class BinaryCommandView : public QWidget
{
    Q_OBJECT
    
public:
    explicit BinaryCommandView(QWidget *parent = 0);
    virtual void updateCursor(double, double);
    ~BinaryCommandView();

public slots:
    virtual void updateFrame(DrawableFrame * drawing);

protected:
    void updateDrawableObjects(ShapeDrawable & shape);
    void rebuildDrawableObjectMap();
    void addToObjectMap(ShapeDrawable & shape);
    void drawShape(const ShapeDrawable & shape);
    double cursorX;
    double cursorY;
    DrawableFrame * currentFrame;
    std::map<std::string, ShapeDrawable *> drawable_object_map;
    void paintEvent(QPaintEvent*);
    
private:
    Ui::BinaryCommandView *ui;
};

#endif // BINARYCOMMANDVIEW_H
