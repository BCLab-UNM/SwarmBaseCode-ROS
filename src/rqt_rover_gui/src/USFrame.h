#ifndef rtq_rover_gui_USFrame_H
#define rtq_rover_gui_USFrame_H

#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // For STL pair

using namespace std;

namespace rqt_rover_gui
{

class USFrame : public QFrame
{
    Q_OBJECT
public:
    USFrame(QWidget *parent, Qt::WFlags = 0);
    void setCenterRange(float r);
    void setLeftRange(float r);
    void setRightRange(float r);

signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    float center_range;
    float left_range;
    float right_range;

};

}

#endif // USFrame_H
