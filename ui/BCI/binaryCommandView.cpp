#include "binaryCommandView.h"
#include "ui_binaryCommandView.h"


#include "Drawable.pb.h"
#include <algorithm>
#include <functional>
#include <QBrush>
#include <QPen>

#include "BCI/bciService.h"

BinaryCommandView::BinaryCommandView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BinaryCommandView)
{
    ui->setupUi(this);
    currentFrame = new DrawableFrame;
    connect(BCIService::getInstance() ,
            SIGNAL(drawShapes(DrawableFrame*)),
            this,SLOT(updateFrame(DrawableFrame*)));
}

void BinaryCommandView::paintEvent(QPaintEvent *)
{
    std::for_each(currentFrame->mutable_shapes()->begin(),
                  currentFrame->mutable_shapes()->end(),
                  std::bind(&BinaryCommandView::drawShape,
                            this,
                            std::placeholders::_1));
}

void BinaryCommandView::updateCursor(double x, double y)
{
    cursorX = x/7.5;
    cursorY = y/7.5;
    update();
}

Qt::BrushStyle shapeStyleToBrushStyle(const ShapeDrawable & shape)
{
    switch(shape.fill_pattern())
    {
    case ShapeDrawable::DiagonalLines:
        return Qt::BDiagPattern;
        break;
    case ShapeDrawable::FDiagonalLines:
        return Qt::FDiagPattern;
        break;
    case ShapeDrawable::HorizontalLines:
        return Qt::HorPattern;
        break;

    case ShapeDrawable::VerticalLines:
        return Qt::VerPattern;
        break;

    case ShapeDrawable::CrossedLines:
        return Qt::CrossPattern;
        break;
    case ShapeDrawable::XCrossedLines:
        return Qt::DiagCrossPattern;
        break;
    case ShapeDrawable::SolidFill:
        return Qt::SolidPattern;
        break;
    case ShapeDrawable::NoFill:
        return Qt::NoBrush;
        break;
    }
    return Qt::SolidPattern;
}


Qt::PenStyle shapeStyleToPenStyle(const ShapeDrawable & shape)
{
    switch(shape.line_style())
    {
    case ShapeDrawable::SolidLine:
        return Qt::SolidLine;
        break;
    case ShapeDrawable::Dotted:
        return Qt::DotLine;
        break;
    case ShapeDrawable::Dashed:
        return Qt::DashLine;
        break;
    }
    return Qt::SolidLine;
}


void BinaryCommandView::drawShape(const ShapeDrawable & shape)
{
    QPainter painter(this);
    Qt::BrushStyle fillStyle(shapeStyleToBrushStyle(shape));
    Qt::PenStyle outlineStyle(shapeStyleToPenStyle(shape));
    painter.setBrush(fillStyle);
    painter.setPen(outlineStyle);
    QPointF origin(shape.x(), shape.y());

    switch(shape.shape_type())
    {
    case ShapeDrawable::Circle:
    {
        painter.drawEllipse(origin,shape.width(), shape.height());
        break;
    }

    case ShapeDrawable::Rectangle:
    {
        QRectF rect(origin, QSizeF(shape.width(), shape.height()));
        painter.drawRect(rect);
        break;
    }
    case ShapeDrawable::Line:
    {
        QPointF lineEnding(shape.x() + shape.width(),
                           shape.y() + shape.width());

        painter.drawLine(origin, lineEnding);
        break;
    }
    case ShapeDrawable::Text:
    {
        QFont font;
        font = painter.font();
        font.setPixelSizeFloat(shape.height());

        painter.setFont(font);
        painter.drawText(origin,QString(shape.text().c_str()));
    break;
    }

    }
}





void BinaryCommandView::addToObjectMap(ShapeDrawable & shape)
{
    drawable_object_map[shape.name()] = &shape;
}

void BinaryCommandView::updateDrawableObjects(ShapeDrawable & shape)
{
    std::string shape_name = shape.name();
    std::map<std::string, ShapeDrawable*>::iterator lb = drawable_object_map.lower_bound(shape.name());
    if(lb != drawable_object_map.end() && !drawable_object_map.key_comp()(shape_name, lb->first))
    {
        lb->second->CopyFrom(shape);
    }
    else
    {
        ShapeDrawable * newShape = currentFrame->add_shapes();
        newShape->CopyFrom(shape);
        drawable_object_map[shape.name()] = newShape;
    }
}


void BinaryCommandView::rebuildDrawableObjectMap()
{
    drawable_object_map.clear();
    std::for_each(currentFrame->mutable_shapes()->begin(),
                  currentFrame->mutable_shapes()->end(),
                  std::bind(&BinaryCommandView::addToObjectMap,
                            this, std::placeholders::_1));
}



 void BinaryCommandView::updateFrame(DrawableFrame * drawing)
 {
     if(drawing->cleardrawables())
     {
         currentFrame->CopyFrom(*drawing);
         rebuildDrawableObjectMap();
     }
     else
     {
       std::for_each(drawing->mutable_shapes()->begin(),
                     drawing->mutable_shapes()->end(),
                     std::bind(&BinaryCommandView::updateDrawableObjects, this,
                               std::placeholders::_1));
     }
 }

BinaryCommandView::~BinaryCommandView()
{
    delete ui;
}
