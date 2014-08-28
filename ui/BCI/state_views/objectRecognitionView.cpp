#include "objectRecognitionView.h"
#include "ui_objectRecognitionView.h"

ObjectRecognitionView::ObjectRecognitionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ObjectRecognitionView)
{
    ui->setupUi(this);
}

ObjectRecognitionView::~ObjectRecognitionView()
{
    delete ui;
}
