/*!
 * \brief   This class visualizes the position output from the odometry, GPS,
 *          and IMU sensors. The extended Kalman filter (EKF) integrates the
 *          position data and transmits it on the appropraite ROS topics. The
 *          map view shows the path taken by the currently selected rover. In
 *          simulation, the encoder position data comes from the odometry topic
 *          being published by Gazebo's skid steer controller plugin. In the
 *          real robots, it is the encoder output. GPS points are shown as red
 *          dots. The EKF is the output of an extended Kalman filter which
 *          fuses data from the IMU, GPS, and encoder sensors.
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
class MapData;

using namespace std;

// Possible waypoint commands to send to the rover
// See custom ROS message type in swarmie_msgs package for options.
enum WaypointCmd {ADD, REMOVE};

namespace rqt_rover_gui
{

  class MapFrame : public QFrame
  {
    Q_OBJECT

    public:

      MapFrame(QWidget *parent, Qt::WindowFlags = 0);

      void setWhetherToDisplay(string rover, bool yes);
      void createPopoutWindow(MapData *map_data);

      void setDisplayEncoderData(bool display);
      void setDisplayGPSData(bool display);
      void setDisplayEKFData(bool display);
      void setGlobalOffset(bool display);
      void setGlobalOffsetForRover(std::string rover, float x, float y);
      void setDisplayUniqueRoverColors(bool display);
      void setUniqueRoverColor(std::string, QColor rover_color);

      void addToGPSRoverPath(std::string rover, float x, float y);
      void addToEncoderRoverPath(std::string rover, float x, float y);
      void addToEKFRoverPath(std::string rover, float x, float y);

      void setMapData(MapData* map_data);

      // Create a waypoint on map. Rovers will navigate to the waypoint.
      void addWaypoint(std::string rover, float x, float y);
      void removeWaypoint(std::string rover, int id );
      void resetAllWaypointPaths();
      void resetWaypointPathForSelectedRover(std::string rover);
     
      void clear();
      void clear(std::string rover);

      // Set the map scale and translation using user mouse clicks
      // wheel for zooming in and out
      // press and move for panning
      // Excludes auto transform
      void setManualTransform();

      // Calculate scale and transform to keep all data in the map frame
      // Excludes manual trasform
      void setAutoTransform();

      void enableWaypoints(std::string rover);
      void disableWaypoints(std::string rover);

      // Show a copy of the map in its own resizable window
      void popout();

      ~MapFrame();

    signals:

      void sendInfoLogMessage(QString msg);
      void sendWaypointCmd(WaypointCmd, int, float, float);
      void delayedUpdate();

    public slots:

        void receiveWaypointReached(int);
        void receiveCurrentRoverName(QString);
        
    protected:

      void paintEvent(QPaintEvent *event);
      void mouseReleaseEvent(QMouseEvent *event);
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
      bool display_global_offset;
      bool display_unique_rover_colors;

      QTime frame_rate_timer;
      int frames;

      set<string> display_list;
      std::map<std::string, QColor> unique_rover_colors;
      std::map<std::string, QColor> unique_simulated_rover_colors;
      QColor unique_physical_rover_colors[8] = { /* green         */ QColor(  0, 255,   0),
                                                 /* yellow        */ QColor(255, 255,   0),
                                                 /* white         */ QColor(255, 255, 255),
                                                 /* red           */ QColor(255,   0,   0),
                                                 /* deep sky blue */ QColor(  0, 191, 255),
                                                 /* hot pink      */ QColor(255, 105, 180),
                                                 /* chocolate     */ QColor(210, 105,  30),
                                                 /* indigo        */ QColor( 75,   0, 130) };

      // For external pop out window
      QMainWindow* popout_window;
      MapFrame* popout_mapframe;

      // State for panning and zooming the map
      int scale;
      float scale_speed; // Amount to zoom as the mouse wheel angle changes

      QPoint previous_clicked_position;
      float translate_x;
      float translate_y;
      float previous_translate_x;
      float previous_translate_y;
      float translate_speed; // Amount to pan by as the mouse position changes

      bool auto_transform;

      float max_seen_width_when_manual_enabled;
      float max_seen_height_when_manual_enabled;
      float min_seen_x_when_manual_enabled;
      float min_seen_y_when_manual_enabled;

      QPoint mouse_pointer_position = QPoint(0,0);
      
      MapData* map_data;

      // Map coordinate data
      // Calculate the axis positions
      int map_origin_x = 0;
      int map_origin_y = 0;
      
      int map_width = 0;
      int map_height = 0; 
      
      int map_center_x = 0; 
      int map_center_y = 0;

      float max_seen_x = -std::numeric_limits<float>::max();
      float max_seen_y = -std::numeric_limits<float>::max();
      
      float min_seen_x = std::numeric_limits<float>::max();
      float min_seen_y = std::numeric_limits<float>::max();
      
      float max_seen_width = -std::numeric_limits<float>::max();
      float max_seen_height = -std::numeric_limits<float>::max();

      std::string rover_currently_selected; // This is the rover selected in the main GUI.

      enum mode {
        AUTONOMOUS,
        MANUAL
      };
      std::map<std::string, bool> rover_mode;
  };

}

#endif // MapFrame_H
