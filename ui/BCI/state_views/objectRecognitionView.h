#ifndef OBJECTRECOGNITIONVIEW_H
#define OBJECTRECOGNITIONVIEW_H

#include <QWidget>

namespace Ui {
class ObjectRecognitionView;
}

class ObjectRecognitionView : public QWidget
{
    Q_OBJECT
    
public:
    explicit ObjectRecognitionView(QWidget *parent = 0);
    ~ObjectRecognitionView();
    
private:
    Ui::ObjectRecognitionView *ui;
};

#endif // OBJECTRECOGNITIONVIEW_H
