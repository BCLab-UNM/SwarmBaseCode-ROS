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
    void setCenterRange(float r, float min, float max);
    void setLeftRange(float r, float min, float max);
    void setRightRange(float r, float min, float max);

signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    float center_range;
    float left_range;
    float right_range;

    float center_max_range;
    float center_min_range;

    float left_max_range;
    float left_min_range;

    float right_max_range;
    float right_min_range;

};

}

#endif // USFrame_H
