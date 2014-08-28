#ifndef EXECUTIONVIEW_H
#define EXECUTIONVIEW_H

#include <QWidget>

namespace Ui {
class ExecutionView;
}

class ExecutionView : public QWidget
{
    Q_OBJECT
    
public:
    explicit ExecutionView(QWidget *parent = 0);
    ~ExecutionView();

public slots:
    void onStopButtonPressed();
    
private:
    Ui::ExecutionView *ui;
};

#endif // EXECUTIONVIEW_H
