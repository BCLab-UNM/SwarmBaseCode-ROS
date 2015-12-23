/*!
 * \brief   This class visualizes the position output from the odometry, GPS, and IMU sensors. The extended Kalman filter (EKF)
 *          integrates the position data and transmits it on the appropraite ROS topics. The map view shows the path taken by
 *          the currently selected rover. In simulation, the encoder position data
 *          comes from the odometry topic being published by Gazebo's skid steer controller plugin.
 *          In the real robots, it is the encoder output. GPS points are shown as red dots.
 *          The EKF is the output of an extended Kalman filter which fuses data from the IMU, GPS, and encoder sensors.
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    Code works properly.
 * \class   MapFrame
 */

#ifndef MAPFRAME_H
#define MAPFRAME_H

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

    float max_gps_seen_x;
    float max_gps_seen_y;
    float min_gps_seen_x;
    float min_gps_seen_y;

    float max_encoder_seen_x;
    float max_encoder_seen_y;
    float min_encoder_seen_x;
    float min_encoder_seen_y;

    float max_ekf_seen_x;
    float max_ekf_seen_y;
    float min_ekf_seen_x;
    float min_ekf_seen_y;

    float max_gps_seen_width;
    float max_gps_seen_height;

    float max_ekf_seen_width;
    float max_ekf_seen_height;

    float max_encoder_seen_width;
    float max_encoder_seen_height;

    bool display_gps_data;
    bool display_ekf_data;
    bool display_encoder_data;

    QTime frame_rate_timer;
    int frames;

};

}

#endif // MapFrame_H
