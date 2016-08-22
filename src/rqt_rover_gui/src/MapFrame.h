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
#include <set>
#include <utility> // For STL pair
#include <map>
#include <QString>

// Forward declarations
class QMainWindow;

using namespace std;

namespace rqt_rover_gui
{

class MapFrame : public QFrame
{
    Q_OBJECT
public:
    MapFrame(QWidget *parent, Qt::WFlags = 0);

    void setWhetherToDisplay(string rover, bool yes);
    void createPopoutWindow();

    void setDisplayEncoderData(bool display);
    void setDisplayGPSData(bool display);
    void setDisplayEKFData(bool display);

    void addToGPSRoverPath(string rover, float x, float y);
    void addToEncoderRoverPath(string rover, float x, float y);
    void addToEKFRoverPath(string rover, float x, float y);

    // Set the map scale and translation using user mouse clicks
    // wheel for zooming in and out
    // press and move for panning
    // Excludes auto transform
    void setManualTransform();

    // Calculate scale and transform to keep all data in the map frame
    // Excludes manual trasform
    void setAutoTransform();
    void popout(); // Show a copy of the map in its own resizable window

    void addTargetLocation(string rover, float x, float y);
    void addCollectionPoint(string rover, float x, float y);
    void clearMap(string rover);
    void clearMap();

    ~MapFrame();

signals:

    void sendInfoLogMessage(QString msg);
    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *);

private:

    mutable QMutex update_mutex;
    int frame_width;
    int frame_height;

    bool display_gps_data;
    bool display_ekf_data;
    bool display_encoder_data;

    QTime frame_rate_timer;
    int frames;

    map<string, vector< pair<float,float> > > gps_rover_path;
    map<string, vector< pair<float,float> > >  ekf_rover_path;
    map<string, vector< pair<float,float> > >  encoder_rover_path;

    map<string, vector< pair<float,float> > >  collection_points;
    map<string, vector< pair<float,float> > >  target_locations;

    map<string, float> max_gps_seen_x;
    map<string, float> max_gps_seen_y;
    map<string, float> min_gps_seen_x;
    map<string, float> min_gps_seen_y;

    map<string, float> max_encoder_seen_x;
    map<string, float> max_encoder_seen_y;
    map<string, float> min_encoder_seen_x;
    map<string, float> min_encoder_seen_y;

    map<string, float> max_ekf_seen_x;
    map<string, float> max_ekf_seen_y;
    map<string, float> min_ekf_seen_x;
    map<string, float> min_ekf_seen_y;

    set<string> display_list;

    // For external pop out window
    QMainWindow* popout_window;
    MapFrame* popout_mapframe;

    // State for panning and zooming the map
    float scale;
    float scale_speed; // Amount to zoom as the mouse wheel angle changes

    QPoint previous_mouse_position;
    float translate_x;
    float translate_y;
    float translate_speed; // Amount to pan by as the mouse position changes

    bool auto_transform;

    float max_seen_width_when_manual_enabled;
    float max_seen_height_when_manual_enabled;
    float min_seen_x_when_manual_enabled;
    float min_seen_y_when_manual_enabled;
};

}

#endif // MapFrame_H
