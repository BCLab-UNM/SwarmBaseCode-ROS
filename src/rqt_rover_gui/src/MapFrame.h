#ifndef rtq_rover_gui_MapFrame_H
#define rtq_rover_gui_MapFrame_H

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

class MapFrame : public QFrame
{
    Q_OBJECT
public:
    MapFrame(QWidget *parent, Qt::WFlags = 0);

    void setDisplayEncoderData(bool display);
    void setDisplayGPSData(bool display);
    void setDisplayEKFData(bool display);

    void addToGPSRoverPath(float x, float y);
    void addToEncoderRoverPath(float x, float y);
    void addToEKFRoverPath(float x, float y);


    void addTargetLocation(float x, float y);
    void addCollectionPoint(float x, float y);
    void clearMap();

signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    vector< pair<float,float> > gps_rover_path;
    vector< pair<float,float> > ekf_rover_path;
    vector< pair<float,float> > encoder_rover_path;

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

    bool display_gps_data;
    bool display_ekf_data;
    bool display_encoder_data;

    QTime frame_rate_timer;
    int frames;

};

}

#endif // MapFrame_H
