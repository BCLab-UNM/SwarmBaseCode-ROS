#ifndef rtq_rover_gui_GPSFrame_H
#define rtq_rover_gui_GPSFrame_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // For STL pair

using namespace std;

namespace rqt_rover_gui
{

class GPSFrame : public QFrame
{
    Q_OBJECT
public:
    GPSFrame(QWidget *parent, Qt::WFlags = 0);


signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    QTime frame_rate_timer;
    int frames;

};

}

#endif // GPSFrame_H
