#ifndef rtq_rover_gui_MapFrame_H
#define rtq_rover_gui_MapFrame_H

#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // For STL pair

using namespace std;

namespace rqt_rover_gui
{

class MapFrame : public QFrame
{
    Q_OBJECT
public:
    MapFrame(QWidget *parent, Qt::WFlags = 0);

    void addToRoverPath(float x, float y);
    void addTargetLocation(float x, float y);
    void addCollectionPoint(float x, float y);
    void clearMap();

signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    vector< pair<float,float> > rover_path;
    vector< pair<float,float> > collection_points;
    vector< pair<float,float> > target_locations;
    mutable QMutex update_mutex;
    int frame_width;
    int frame_height;

    float max_seen_x;
    float max_seen_y;
    float min_seen_x;
    float min_seen_y;

    float max_seen_width;
    float max_seen_height;

};

}

#endif // MapFrame_H
