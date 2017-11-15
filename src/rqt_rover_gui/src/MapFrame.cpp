#ifndef rqt_rover_gui_MapFrame
#define rqt_rover_gui_MapFrame

#include <iostream>
#include <cmath>
#include <limits>
#include <QMainWindow>
#include <QGridLayout>
#include <QLabel>
#include <QMouseEvent>
#include <MapData.h>
#include "MapFrame.h"

namespace rqt_rover_gui
{

MapFrame::MapFrame(QWidget *parent, Qt::WindowFlags flags) : QFrame(parent)
{
  connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

  // Scale coordinates
  frame_width = this->width();
  frame_height = this->height();

  // So we can keep track of relative mouse movements to make
  // panning feel natural
  previous_clicked_position = QPoint(0,0);

  auto_transform = true;
  scale = 10;

  translate_x = 0.0f;
  translate_y = 0.0f;
  previous_translate_x = 0.0f;
  previous_translate_y = 0.0f;

  scale_speed = 0.1; // The amount of zoom per mouse wheel angle change
  translate_speed = 1.5f;

  display_ekf_data = false;
  display_gps_data = false;
  display_encoder_data = false;
  display_global_offset = false;
  display_unique_rover_colors = false;

  frames = 0;
  popout_mapframe = NULL;
  popout_window = NULL;

  map_data = NULL;

  // Trigger mouseMoveEvent even when button not pressed
  setMouseTracking(true);

}

// This can't go in the constructor or there will be an infinite regression.
// Instead call from the main UI in it's startup routine.
void MapFrame::createPopoutWindow( MapData * map_data )
{
  popout_window = new QMainWindow();
  popout_mapframe = new MapFrame(popout_window, 0);
  popout_mapframe->setMapData(map_data);

  QGridLayout* layout = new QGridLayout();
  layout->addWidget(popout_mapframe);

  QWidget* central_widget = new QWidget();
  central_widget->setLayout(layout);

  popout_window->setGeometry(QRect(10, 10, 500, 500));
  popout_window->setStyleSheet("background-color: rgb(0, 0, 0); border-color: rgb(255, 255, 255);");
  popout_window->setCentralWidget(central_widget);

  connect(this, SIGNAL(delayedUpdate()), popout_mapframe, SLOT(update()), Qt::QueuedConnection);


  // Forward signals from the popout map to the parent map for handling. We only need to forward signals
  // triggered by the popout object - such as mouse clicks
  connect(popout_mapframe, SIGNAL(sendInfoLogMessage(QString)), this, SIGNAL(sendInfoLogMessage(QString)));
  connect(popout_mapframe, SIGNAL(sendWaypointCmd(WaypointCmd, int, float, float)), this, SIGNAL(sendWaypointCmd(WaypointCmd, int, float, float)));

}

void MapFrame::paintEvent(QPaintEvent* event) {
  // Begin drawing the map
  QPainter painter(this);
  painter.setPen(Qt::white);
  QFont font = painter.font();
  qreal font_size = font.pointSizeF();
  QFontMetrics fm(font);

  // Track the frames per second for development purposes
  QString frames_per_second;
  frames_per_second = QString::number(frames / (frame_rate_timer.elapsed() / 1000.0), 'f', 0) + " FPS";

  painter.drawText(this->width()-fm.width(frames_per_second), fm.height(), frames_per_second);

  frames++;

  if (!(frames % 100)) // time how long it takes to dispay 100 frames
  {
    frame_rate_timer.start();
    frames = 0;
  }

  // end frames per second

  // Check if any rovers have been selected for display
  if ( !map_data )
  {
    painter.drawText(QPoint(50,50), "No map data provided");
    return;
  }


  // Check if any rovers have been selected for display
  if ( display_list.empty() )
  {
    painter.drawText(QPoint(50,50), "No rover maps selected for display");
    return;
  }

  map_data->lock();

  QColor green(17, 192, 131);
  QColor red(255, 65, 30);

  max_seen_x = -std::numeric_limits<float>::max(); // std::numeric_limits<float>::max() is the max possible floating point value
  max_seen_y = -std::numeric_limits<float>::max();

  min_seen_x = std::numeric_limits<float>::max();
  min_seen_y = std::numeric_limits<float>::max();

  max_seen_width = -std::numeric_limits<float>::max();
  max_seen_height = -std::numeric_limits<float>::max();

  int no_data_offset = 0; // So the "no data" message is not overlayed if there are multiple rovers with no data.

  // Repeat the display code for each rover selected by the user - Using C++11 range syntax
  for(auto rover_to_display : display_list) {
    if (map_data->getEKFPath(rover_to_display)->empty() && map_data->getEncoderPath(rover_to_display)->empty() && map_data->getGPSPath(rover_to_display)->empty() && map_data->getTargetLocations(rover_to_display)->empty() && map_data->getCollectionPoints(rover_to_display)->empty()) {
      painter.drawText(QPoint(50,50+no_data_offset), QString::fromStdString(rover_to_display) + ": No data.");
      no_data_offset += 10;
    }
    // Check extended kalman filter has any values in it
    else if (map_data->getEKFPath(rover_to_display)->empty()) {
      painter.drawText(QPoint(50,50+no_data_offset), "Map Frame: No EKF data received.");
      no_data_offset += 10;
    }
  }

  // Calculate the map bounds if in auto transform mode. Iterate over the
  // rovers and get the min and max data values scale the map to include
  // these values
  if (auto_transform)
  {
    for(auto rover_to_display : display_list)
    {
      // Set the max and min seen values depending on which data the user
      // has selected to view

      // Check each of the display data options and choose the most
      // extreme value from those selected by the user

      // Always include the ekf data because that is what we are using to
      // position the current position marker for the rover

      if (display_ekf_data)
      {
      if (min_seen_x > map_data->getMinEKFX(rover_to_display)) min_seen_x = map_data->getMinEKFX(rover_to_display);
      if (min_seen_y > map_data->getMinEKFY(rover_to_display)) min_seen_y = map_data->getMinEKFY(rover_to_display);
      if (max_seen_x < map_data->getMaxEKFX(rover_to_display)) max_seen_x = map_data->getMaxEKFX(rover_to_display);
      if (max_seen_y < map_data->getMaxEKFY(rover_to_display)) max_seen_y = map_data->getMaxEKFY(rover_to_display);
      }

      if (display_gps_data)
      {
      if (min_seen_x > map_data->getMinGPSX(rover_to_display)) min_seen_x = map_data->getMinGPSX(rover_to_display);
      if (min_seen_y > map_data->getMinGPSY(rover_to_display)) min_seen_y = map_data->getMinGPSY(rover_to_display);
      if (max_seen_x < map_data->getMaxGPSX(rover_to_display)) max_seen_x = map_data->getMaxGPSX(rover_to_display);
      if (max_seen_y < map_data->getMaxGPSY(rover_to_display)) max_seen_y = map_data->getMaxGPSY(rover_to_display);
      }

      if (display_encoder_data)
      {
      if (min_seen_x > map_data->getMinEncoderX(rover_to_display)) min_seen_x = map_data->getMinEncoderX(rover_to_display);
      if (min_seen_y > map_data->getMinEncoderY(rover_to_display)) min_seen_y = map_data->getMinEncoderY(rover_to_display);
      if (max_seen_x < map_data->getMaxEncoderX(rover_to_display)) max_seen_x = map_data->getMaxEncoderX(rover_to_display);
      if (max_seen_y < map_data->getMaxEncoderY(rover_to_display)) max_seen_y = map_data->getMaxEncoderY(rover_to_display);
      }

      // Normalize the displayed coordinates to the largest coordinates
      // seen since we don't know the coordinate system.
      max_seen_width = max_seen_x-min_seen_x;
      max_seen_height = max_seen_y-min_seen_y;
    }
  }
  else
  {
    // Perform the manual zoom and pan transform

    max_seen_width = max_seen_width_when_manual_enabled * (scale * scale_speed);
    max_seen_height = max_seen_height_when_manual_enabled * (scale * scale_speed);

    min_seen_x = (min_seen_x_when_manual_enabled + translate_x) * (scale * scale_speed);
    min_seen_y = (min_seen_y_when_manual_enabled + translate_y) * (scale * scale_speed);

    // emit sendInfoLogMessage("MapFrame: paint event: manual transform: min_seen_x: " + QString::number(min_seen_x) + " min_seen_y: " + QString::number(min_seen_y));
  }

  // Maintain aspect ratio
  max_seen_height > max_seen_width ? max_seen_width = max_seen_height : max_seen_height = max_seen_width;

  // Calculate the axis positions
  map_origin_x = fm.width(QString::number(-max_seen_height, 'f', 1)+"m");
  map_origin_y = 2*fm.height();

  map_width = this->width()-1;// Minus 1 or will go off the edge
  map_height = this->height()-1;//

  int map_center_x = map_origin_x+((map_width-map_origin_x)/2);
  int map_center_y = map_origin_y+((map_height-map_origin_y)/2);

  // The map axes do not need to be redrawn for each rover so this code is
  // sandwiched between the two rover display list loops

  // Draw the scale bars
  //painter.setPen(Qt::gray);
  //painter.drawLine(QPoint(map_center_x, map_origin_y), QPoint(map_center_x, map_height));
  //painter.drawLine(QPoint(map_origin_x, map_center_y), QPoint(map_width, map_center_y));
  //painter.setPen(Qt::white);

  // Cross hairs at map display center
  QPoint axes_origin(map_origin_x,map_origin_y);
  QPoint x_axis(map_width,map_origin_y);
  QPoint y_axis(map_origin_x,map_height);
  painter.drawLine(axes_origin, x_axis);
  painter.drawLine(axes_origin, y_axis);
  painter.drawLine(QPoint(map_width, map_origin_y), QPoint(map_width, map_height));
  painter.drawLine(QPoint(map_origin_x, map_height), QPoint(map_width, map_height));

  // Draw north arrow
  QPoint northArrow_point(map_center_x, 0);
  QPoint northArrow_left(map_center_x - 5, 5);
  QPoint northArrow_right(map_center_x + 5, 5);
  QRect northArrow_textBox(northArrow_left.x(), northArrow_left.y(), 10, 15);
  painter.drawLine(northArrow_left, northArrow_right);
  painter.drawLine(northArrow_left, northArrow_point);
  painter.drawLine(northArrow_right, northArrow_point);
  painter.drawText(northArrow_textBox, QString("N"));

  // Draw rover origin crosshairs
  // painter.setPen(green);

  float initial_x = 0.0; //map_data->getEKFPath(rover_to_display).begin()->first;
  float initial_y = 0.001; //map_data->getEKFPath(rover_to_display).begin()->second;
  float rover_origin_x = map_origin_x+((initial_x-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
  float rover_origin_y = map_origin_y+((initial_y-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
  painter.setPen(Qt::gray);
  painter.drawLine(QPoint(rover_origin_x, map_origin_y), QPoint(rover_origin_x, map_height));
  painter.drawLine(QPoint(map_origin_x, rover_origin_y), QPoint(map_width, rover_origin_y));
  painter.setPen(Qt::white);

  int n_ticks = 6;
  float tick_length = 5;
  QPoint x_axis_ticks[n_ticks];
  QPoint y_axis_ticks[n_ticks];

  for (int i = 0; i < n_ticks-1; i++)
  {
    x_axis_ticks[i].setX(axes_origin.x()+(i+1)*map_width/n_ticks);
    x_axis_ticks[i].setY(axes_origin.y());

    y_axis_ticks[i].setX(axes_origin.x());
    y_axis_ticks[i].setY(axes_origin.y()+(i+1)*map_height/n_ticks);
  }

  for (int i = 0; i < n_ticks-1; i++)
  {
    painter.drawLine(x_axis_ticks[i], QPoint(x_axis_ticks[i].x(), x_axis_ticks[i].y()+tick_length));
    painter.drawLine(y_axis_ticks[i], QPoint(y_axis_ticks[i].x()+tick_length, y_axis_ticks[i].y()));
  }

  for (int i = 0; i < n_ticks-1; i++)
  {
    float fraction_of_map_to_rover_x = (rover_origin_x-map_origin_x)/map_width;
    float fraction_of_map_to_rover_y = (rover_origin_y-map_origin_y)/map_height;
    float x_label_f = (i+1)*max_seen_width/n_ticks-fraction_of_map_to_rover_x*max_seen_width;
    float y_label_f = (i+1)*max_seen_height/n_ticks-fraction_of_map_to_rover_y*max_seen_height;

    QString x_label = QString::number(x_label_f, 'f', 1) + "m";
    QString y_label = QString::number(-y_label_f, 'f', 1) + "m";

    int x_labels_offset_x = -(fm.width(x_label))/2;
    int x_labels_offset_y = 0;

    int y_labels_offset_x = -(fm.width(y_label));
    int y_labels_offset_y = fm.height()/3;

    painter.drawText(x_axis_ticks[i].x()+x_labels_offset_x, axes_origin.y()+x_labels_offset_y, x_label);
    painter.drawText(axes_origin.x()+y_labels_offset_x, y_axis_ticks[i].y()+y_labels_offset_y, y_label);
  }

  // End draw scale bars

  int hardware_rover_color_index = 0;

  // Repeat the display code for each rover selected by the user - Using C++11 range syntax
  for(auto rover_to_display : display_list)
  {
    // scale coordinates
    std::vector<QPoint> scaled_target_locations;
    for(std::vector< pair<float,float> >::iterator it = map_data->getTargetLocations(rover_to_display)->begin(); it < map_data->getTargetLocations(rover_to_display)->end(); ++it)
    {
      pair<float,float> coordinate  = *it;
      QPoint point;
      point.setX(map_origin_x+coordinate.first*map_width);
      point.setY(map_origin_y+coordinate.second*map_height);
      scaled_target_locations.push_back(point);
    }
    
    std::vector<QPoint> scaled_collection_points;
    for(std::vector< pair<float,float> >::iterator it = map_data->getCollectionPoints(rover_to_display)->begin(); it < map_data->getCollectionPoints(rover_to_display)->end(); ++it)
    {
      pair<float,float> coordinate  = *it;
      QPoint point;
      point.setX(map_origin_x+coordinate.first*map_width);
      point.setY(map_origin_y+coordinate.second*map_height);
      scaled_collection_points.push_back(point);
    }

    std::vector<QPoint> scaled_gps_rover_points;
    for(std::vector< pair<float,float> >::iterator it = map_data->getGPSPath(rover_to_display)->begin(); it < map_data->getGPSPath(rover_to_display)->end(); ++it)
    {
      pair<float,float> coordinate  = *it;
      
      float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
      float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
      scaled_gps_rover_points.push_back( QPoint(x,y) );
    }


    // Maintain aspect ratio
    max_seen_height > max_seen_width ? max_seen_width = max_seen_height : max_seen_height = max_seen_width;

    // Calculate the axis positions
    map_origin_x = fm.width(QString::number(-max_seen_height, 'f', 1)+"m");
    map_origin_y = 2*fm.height();
    
    map_width = this->width()-1;// Minus 1 or will go off the edge
    map_height = this->height()-1;//
    
    map_center_x = map_origin_x+((map_width-map_origin_x)/2);
    map_center_y = map_origin_y+((map_height-map_origin_y)/2);

    // The map axes do not need to be redrawn for each rover so this code is
    // sandwiched between the two rover display list loops

    // Draw the scale bars
    //painter.setPen(Qt::gray);
    //painter.drawLine(QPoint(map_center_x, map_origin_y), QPoint(map_center_x, map_height));
    //painter.drawLine(QPoint(map_origin_x, map_center_y), QPoint(map_width, map_center_y));
    //painter.setPen(Qt::white);

    // Cross hairs at map display center
    QPoint axes_origin(map_origin_x,map_origin_y);
    QPoint x_axis(map_width,map_origin_y);
    QPoint y_axis(map_origin_x,map_height);
    painter.drawLine(axes_origin, x_axis);
    painter.drawLine(axes_origin, y_axis);
    painter.drawLine(QPoint(map_width, map_origin_y), QPoint(map_width, map_height));
    painter.drawLine(QPoint(map_origin_x, map_height), QPoint(map_width, map_height));

    // Draw north arrow
    QPoint northArrow_point(map_center_x, 0);
    QPoint northArrow_left(map_center_x - 5, 5);
    QPoint northArrow_right(map_center_x + 5, 5);
    QRect northArrow_textBox(northArrow_left.x(), northArrow_left.y(), 10, 15);
    painter.drawLine(northArrow_left, northArrow_right);
    painter.drawLine(northArrow_left, northArrow_point);
    painter.drawLine(northArrow_right, northArrow_point);
    painter.drawText(northArrow_textBox, QString("N"));

    // Draw rover origin crosshairs
    // painter.setPen(green);

    float initial_x = 0.0; //map_data->getEKFPath(rover_to_display).begin()->first;
    float initial_y = 0.001; //map_data->getEKFPath(rover_to_display).begin()->second;
    float rover_origin_x = map_origin_x+((initial_x-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
    float rover_origin_y = map_origin_y+((initial_y-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
    painter.setPen(Qt::gray);
    painter.drawLine(QPoint(rover_origin_x, map_origin_y), QPoint(rover_origin_x, map_height));
    painter.drawLine(QPoint(map_origin_x, rover_origin_y), QPoint(map_width, rover_origin_y));
    painter.setPen(Qt::white);

    QPainterPath scaled_ekf_rover_path;
    for(std::vector< pair<float,float> >::iterator it = map_data->getEKFPath(rover_to_display)->begin(); it < map_data->getEKFPath(rover_to_display)->end(); ++it)
    {
      pair<float,float> coordinate  = *it;
      QPoint point;
      float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
      float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
      
      // Move to the starting point of the path without drawing a line
      if (it == map_data->getEKFPath(rover_to_display)->begin()) scaled_ekf_rover_path.moveTo(x, y);
      scaled_ekf_rover_path.lineTo(x, y);
    }

    QPainterPath scaled_encoder_rover_path;
    for(std::vector< pair<float,float> >::iterator it = map_data->getEncoderPath(rover_to_display)->begin(); it < map_data->getEncoderPath(rover_to_display)->end(); ++it)
    { 
      pair<float,float> coordinate  = *it;
      QPoint point;
      float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
      float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
      
      // Move to the starting point of the path without drawing a line
      if (it == map_data->getEncoderPath(rover_to_display)->begin()) scaled_encoder_rover_path.moveTo(x, y);

      scaled_encoder_rover_path.lineTo(x, y);
    }
    
    QColor rover_color = QColor(255, 255, 255); // white

    // if we have properly set a color for simulated rovers initialise the color here
    // also make the popout map frame aware of the colors we use here
    if(unique_simulated_rover_colors.find(rover_to_display) != unique_simulated_rover_colors.end())
    {
      rover_color = unique_simulated_rover_colors[rover_to_display];
      
      if(popout_mapframe)
      {
        popout_mapframe->setUniqueRoverColor(rover_to_display, unique_simulated_rover_colors[rover_to_display]);
      }
    }
    // caveat in the case that
    //     1) we haven't set sim rover colors properly
    //     2) we are using hardware rovers
    else if(display_unique_rover_colors)
    {
      rover_color = unique_physical_rover_colors[hardware_rover_color_index];
    }

    painter.setPen(rover_color);

    if(!display_unique_rover_colors) painter.setPen(red);

    if (display_gps_data) painter.drawPoints(&scaled_gps_rover_points[0], scaled_gps_rover_points.size());
    // if (display_gps_data) painter.drawPath(scaled_gps_rover_path);

    if(!display_unique_rover_colors) painter.setPen(Qt::white);

    if (display_ekf_data) painter.drawPath(scaled_ekf_rover_path);

    if(!display_unique_rover_colors) painter.setPen(green);

    if (display_encoder_data) painter.drawPath(scaled_encoder_rover_path);

    painter.setPen(red);
    QPoint* point_array = &scaled_collection_points[0];
    painter.drawPoints(point_array, scaled_collection_points.size());

    painter.setPen(green);
    point_array = &scaled_target_locations[0];
    painter.drawPoints(point_array, scaled_target_locations.size());
      
    // Draw the waypoints for the current rover
    painter.setPen(Qt::cyan);
    
    QPainterPath scaled_waypoint_rover_path;
    for(map< int, std::tuple<float,float,bool> >::iterator it = map_data->getWaypointPath(rover_to_display)->begin(); it != map_data->getWaypointPath(rover_to_display)->end(); ++it)
    {	
      tuple<float,float,bool> coordinate  = it->second; // Get the value from the map
      
      float x = map_origin_x+((get<0>(coordinate)-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
      float y = map_origin_y+((get<1>(coordinate)-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
      
      
      QPoint point(x,y);
      int default_pen_width = painter.pen().width();
      QPen previous_pen = painter.pen();
      QPen pen;
      if(!get<2>(coordinate))
      {
	pen.setColor(Qt::cyan);
      }
      else
      {
	pen.setColor(Qt::yellow);
      }
      pen.setWidth(5);
      painter.setPen(pen);
      painter.drawPoint(point);
      
      // Draw lines connecting waypoints
      painter.setPen(previous_pen);
      
      painter.setPen(Qt::blue);
      
      // Move to the starting point of the path without drawing a line
      if (it == map_data->getWaypointPath(rover_to_display)->begin())
      {
	scaled_waypoint_rover_path.moveTo(x, y);
      }
      else
      {
	scaled_waypoint_rover_path.lineTo(x, y);
      }
    }

    painter.drawPath(scaled_waypoint_rover_path);
    

    if(display_unique_rover_colors) painter.setPen(rover_color);
    else painter.setPen(Qt::yellow);
    
    pair<float,float> current_coordinate;
    if(! map_data->getEKFPath(rover_to_display)->empty() )
    {
       current_coordinate = map_data->getEKFPath(rover_to_display)->back();
    }
    
    float x = map_origin_x+((current_coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
    float y = map_origin_y+((current_coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
    float radius = 2.5;
    
    painter.drawEllipse(QPointF(x,y), radius, radius);
    painter.drawText(QPoint(x,y), QString::fromStdString(rover_to_display));

    hardware_rover_color_index = (hardware_rover_color_index + 1) % 8;

    painter.setPen(Qt::white);
  } // End rover display list set iteration

  map_data->unlock();
  
  // Diagnostic output
  /*
  font.setPointSizeF( 12 );
  painter.drawText(QPoint(0,15), "min_seen_x: " + QString::number(min_seen_x));
  painter.drawText(QPoint(0,30), "min_seen_y: " + QString::number(min_seen_y));
  painter.drawText(QPoint(0,45), "max_width_seen: " + QString::number(max_seen_width));
  painter.drawText(QPoint(0,60), "max_height_seen: " + QString::number(max_seen_height));
  painter.drawText(QPoint(0,75), "map_origin_x: " + QString::number(map_origin_x));
  painter.drawText(QPoint(0,90), "map_origin_y: " + QString::number(map_origin_y));
  painter.drawText(QPoint(0,105), "map_width: " + QString::number(map_width));
  painter.drawText(QPoint(0,120), "map_height: " + QString::number(map_height));
  painter.drawText(QPoint(0,135), "map_center_x: " + QString::number(map_center_x));
  */

}


void MapFrame::setDisplayEncoderData(bool display)
{
  display_encoder_data = display;

  if(popout_mapframe) popout_mapframe->setDisplayEncoderData(display);
}

void MapFrame::setDisplayGPSData(bool display)
{
  display_gps_data = display;

  if(popout_mapframe) popout_mapframe->setDisplayGPSData(display);
}

void MapFrame::setDisplayEKFData(bool display)
{
  display_ekf_data = display;

  if(popout_mapframe) popout_mapframe->setDisplayEKFData(display);
}

void MapFrame::setGlobalOffset(bool display)
{
    display_global_offset = display;
    map_data->setGlobalOffset(display);

    if(popout_mapframe) popout_mapframe->setGlobalOffset(display);
}

void MapFrame::setGlobalOffsetForRover(string rover, float x, float y)
{
    map_data->setGlobalOffsetForRover(rover, x, y);
}

void MapFrame::setDisplayUniqueRoverColors(bool display)
{
    display_unique_rover_colors = display;

    if(popout_mapframe) popout_mapframe->setDisplayUniqueRoverColors(display);
}

void MapFrame::setUniqueRoverColor(string rover, QColor rover_color)
{
    unique_rover_colors[rover] = rover_color;
}

void MapFrame::setWhetherToDisplay(string rover, bool yes)
{
  map_data->lock();

  if (yes)
  {
    display_list.insert(rover);
  }
  else
  {
    display_list.erase(rover);
  }

  map_data->unlock();

  if(popout_mapframe) popout_mapframe->setWhetherToDisplay(rover, yes);
}

void MapFrame::mouseReleaseEvent(QMouseEvent *event)
{
  previous_translate_x = translate_x;
  previous_translate_y = translate_y;
}

void MapFrame::mousePressEvent(QMouseEvent *event)
{
  std::set<std::string>::iterator it = display_list.find(rover_currently_selected);

  // Failure condition: a valid rover is not selected.
  if(it == display_list.end())
  {
    emit sendInfoLogMessage("Waypoints Error: a valid rover is not selected!");
    return;
  }
  // Failure condition: don't accept waypoints for rovers in autonomous mode
  else if(! map_data->inManualMode(rover_currently_selected))
  {
    return;
  }

  float waypoint_click_tolerance = 0.25*(scale/10);
  
  if ( event->buttons() == Qt::RightButton )
  {
    // Solve for map coordinates in terms of frame coordinates
    float mouse_map_x = ((event->pos().x() - map_origin_x*1.0f)/(map_width-map_origin_x))*max_seen_width + min_seen_x;
    float mouse_map_y = ((event->pos().y() - map_origin_y*1.0f)/(map_height-map_origin_y))*max_seen_height + min_seen_y;

    // We must "adjust" the clicked position when "global frame" is selected in the GUI
    // If we do not do this here, then the rover will NOT move to where a user clicks,
    //     but to a position offset by the selected rover's global offset.
    if(map_data->isDisplayingGlobalOffset())
    {
      std::pair<float,float> offset = map_data->getGlobalOffsetForRover(rover_currently_selected);
      mouse_map_x -= offset.first;
      mouse_map_y -= offset.second;
    }

    emit sendInfoLogMessage("MOX: " + QString::number(map_origin_x) + " map_width: " + QString::number(map_width) + " max_seen_width: " +  QString::number(min_seen_x));
    
    // If click is within eplison of an existing waypoint remove the waypoint
    bool waypoint_removed = false;
    for(map< int, std::tuple<float,float,bool> >::iterator it = map_data->getWaypointPath(rover_currently_selected)->begin(); it != map_data->getWaypointPath(rover_currently_selected)->end(); ++it)
    {
      // Get the distance between the mouse click and the coordinates of the existing waypoints
      tuple<float,float,bool> coordinate  = it->second;
      float x1 = get<0>(coordinate);
      float x2 = mouse_map_x;
      float y1 = get<1>(coordinate);
      float y2 = mouse_map_y;

emit sendInfoLogMessage(" x1: " + QString::number(x1)
                              + " x2: " + QString::number(x2)
                              + " y1: " + QString::number(y1)
                              + " y2: " + QString::number(y2)
                        + " dist: " + QString::number(sqrt( pow( x1 - x2, 2 ) + pow( y1 - y2, 2 ) ) ) + " scale: " + QString::number(scale) );
      
      if ( sqrt( pow( x1 - x2, 2 ) + pow( y1 - y2, 2 ) ) < waypoint_click_tolerance ) 
      {
        int waypoint_id = it->first;
        removeWaypoint( rover_currently_selected, waypoint_id );
        waypoint_removed = true;
      }
    }
    
    if ( !waypoint_removed )
    {
      emit sendInfoLogMessage(" Adding waypoint at x: " + QString::number(mouse_map_x) + " y: " + QString::number(mouse_map_y));
      addWaypoint(rover_currently_selected, mouse_map_x, mouse_map_y);
    }
  }
  else if ( event->buttons() == Qt::LeftButton )
  {
    previous_clicked_position = event->pos();
  }
  
    // emit sendInfoLogMessage("MapFrame: mouse press. x: " + QString::number(mouse_event->pos().x()) + ", y: " + QString::number(mouse_event->pos().y()));

}

void MapFrame::mouseMoveEvent(QMouseEvent *event)
{

  // Get the mouse pointer position to print on the map
  mouse_pointer_position = event->pos();

  // Do not adjust panning in auto-transform mode.. the changes will not be reflected
    // and will be applied only when the user clicks on manual panning mode which will
    // cause undesired results.
    if (auto_transform == true) return;

    if (event->type() == QEvent::MouseMove )
    {
      // Check whether only the left button is down
      if ( event->buttons() == Qt::LeftButton )
      {
        QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);
        float max_width = this->width();
        float max_height = this->height();
        
        // start with the previous translate
        translate_x = previous_translate_x;
        translate_y = previous_translate_y;
        
        // add the scaled translation based on the previous mouse click
        // and the current mouse position while dragging; multiply the translation
        // by the given translate speed to keep the map lined up with mouse movement
        translate_x += translate_speed * (previous_clicked_position.x() - mouse_event->pos().x()) / max_width;
        translate_y += translate_speed * (previous_clicked_position.y() - mouse_event->pos().y()) / max_height;
        
        // debug info log messages
        // emit sendInfoLogMessage("MapFrame: mouse move: translate_x: " + QString::number(translate_x) + " translate_y: " + QString::number(translate_y) + "\n");
        // emit sendInfoLogMessage("MapFrame: mouse move: frame_width: " + QString::number(this->width()) + " frame_height: " + QString::number(this->height()));
        // emit sendInfoLogMessage("MapFrame: mouse move: x: " + QString::number(mouse_event->pos().x()) + " y: " + QString::number(mouse_event->pos().y()));
        // emit sendInfoLogMessage("MapFrame: mouse move: xp: " + QString::number(previous_clicked_position.x()) + " yp: " + QString::number(previous_clicked_position.y()));

        // Store the current mouse position for use in the map
      }
   }
}

void MapFrame::wheelEvent(QWheelEvent *event)
{
  // Do not adjust the zoom in auto-transform mode.. the changes will not be reflected
  // and will be applied only when the user clicks on manual panning mode which will
  // cause undesired results.
  if (auto_transform == true) return;

  // 100% map zoom is set when scale = 10; 10% adjustments to 
  // the zoom occur with each mouse wheel adjustment
  if (event->delta() < 0) {
    scale++;
  } else {
    scale--;
  }

  // limit the lower bound of the scale so we do not invert the map and have
  // negative zoom values
  if (scale <= 0) {
    scale = 1;
    // emit sendInfoLogMessage("Map Zoom Set: " + QString::number(scale * 10) + "% (Minimum Zoom)");
  } else {
    // emit sendInfoLogMessage("Map Zoom Set: " + QString::number(scale * 10) + "%");
  }

  // debug info log messages
  // emit sendInfoLogMessage("MapFrame: mouse wheel. Degrees: " + QString::number(num_degrees) + " Scale: " + QString::number(scale));
  // emit sendInfoLogMessage("MapFrame: mouse wheel. x: " + QString::number(event->pos().x()) + " y: " + QString::number(event->pos().y()));
}

void MapFrame::setManualTransform()
{
  if (popout_mapframe) popout_mapframe->setManualTransform();
  auto_transform = false;

  // Calculate and store the max and min values seen so far for use my the manual transform
  float max_seen_x = -std::numeric_limits<float>::max(); // std::numeric_limits<float>::max() is the max possible floating point value
  float max_seen_y = -std::numeric_limits<float>::max();
  float min_seen_x = std::numeric_limits<float>::max();
  float min_seen_y = std::numeric_limits<float>::max();

  for (auto rover_to_display : display_list)
  {
      if (display_ekf_data)
      {
        if (min_seen_x > map_data->getMinEKFX(rover_to_display)) min_seen_x = map_data->getMinEKFX(rover_to_display);
        if (min_seen_y > map_data->getMinEKFY(rover_to_display)) min_seen_y = map_data->getMinEKFY(rover_to_display);
        if (max_seen_x < map_data->getMaxEKFX(rover_to_display)) max_seen_x = map_data->getMaxEKFX(rover_to_display);
        if (max_seen_y < map_data->getMaxEKFY(rover_to_display)) max_seen_y = map_data->getMaxEKFY(rover_to_display);
      }

      if (display_gps_data)
      {
        if (min_seen_x > map_data->getMinGPSX(rover_to_display)) min_seen_x = map_data->getMinGPSX(rover_to_display);
        if (min_seen_y > map_data->getMinGPSY(rover_to_display)) min_seen_y = map_data->getMinGPSY(rover_to_display);
        if (max_seen_x < map_data->getMaxGPSX(rover_to_display)) max_seen_x = map_data->getMaxGPSX(rover_to_display);
        if (max_seen_y < map_data->getMaxGPSY(rover_to_display)) max_seen_y = map_data->getMaxGPSY(rover_to_display);
      }

      if (display_encoder_data)
      {
        if (min_seen_x > map_data->getMinEncoderX(rover_to_display)) min_seen_x = map_data->getMinEncoderX(rover_to_display);
        if (min_seen_y > map_data->getMinEncoderY(rover_to_display)) min_seen_y = map_data->getMinEncoderY(rover_to_display);
        if (max_seen_x < map_data->getMaxEncoderX(rover_to_display)) max_seen_x = map_data->getMaxEncoderX(rover_to_display);
        if (max_seen_y < map_data->getMaxEncoderY(rover_to_display)) max_seen_y = map_data->getMaxEncoderY(rover_to_display);
      }
  }

  // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
  float max_seen_width = max_seen_x-min_seen_x;
  float max_seen_height = max_seen_y-min_seen_y;
  min_seen_x_when_manual_enabled = min_seen_x;
  min_seen_y_when_manual_enabled = min_seen_y;
  max_seen_width_when_manual_enabled = max_seen_width;
  max_seen_height_when_manual_enabled = max_seen_height;

  /* scale the translate speed with the max seen width and height */
  translate_speed = (max_seen_width * 0.75) + (max_seen_height * 0.75);
}

void MapFrame::setAutoTransform()
{
  if (popout_mapframe) popout_mapframe->setAutoTransform();
  auto_transform = true;
  scale = 10;
  translate_x = 0.0f;
  translate_y = 0.0f;
  previous_translate_x = 0.0f;
  previous_translate_y = 0.0f;
  previous_clicked_position = QPoint(0,0);
}

void MapFrame::clear()
{
  map_data->lock();
  display_list.clear();
  map_data->unlock();
}

void MapFrame::clear(string rover)
{
  map_data->lock();
  display_list.erase(rover);
  map_data->unlock();
}

void MapFrame::popout()
{
  if (popout_window) popout_window->show();
}

 void MapFrame::setMapData(MapData* data)
 {
   map_data = data;
 }

 void MapFrame::addToGPSRoverPath(std::string rover, float x, float y)
 {
   if (map_data)
   {
      map_data->addToGPSRoverPath(rover, x, y);
      emit delayedUpdate();
   }
 }

void MapFrame::addToEncoderRoverPath(std::string rover, float x, float y)
{
  if (map_data)
  {
    map_data->addToEncoderRoverPath(rover, x, y);
    emit delayedUpdate();
  }
}

void MapFrame::addToEKFRoverPath(std::string rover, float x, float y)
{
  if (map_data)
  {
    map_data->addToEKFRoverPath(rover, x, y);
    emit delayedUpdate();
  }
}

void MapFrame::addWaypoint( string rover, float x, float y ) {
  if (map_data)
  {
    int id = map_data->addToWaypointPath(rover, x, y);
    cout << "Waypoint created. There are now " << map_data->getWaypointPath(rover_currently_selected)->size() << " waypoints" << endl;
    emit delayedUpdate();
    // The y coordinate must be negated to translatAe between the
    // map frame and the rover frame.
    emit sendWaypointCmd(ADD, id, x, -y);
  }
}

void MapFrame::removeWaypoint( string rover, int id ) {
  if (map_data)
  {
    map_data->removeFromWaypointPath(rover, id );
    cout << "Waypoint removed. There are now " << map_data->getWaypointPath(rover_currently_selected)->size() << " waypoints" << endl;
    emit delayedUpdate();
    emit sendWaypointCmd(REMOVE, id, 0, 0); // x and y are 0 here because they are unused in a remove command
  }
}

void MapFrame::resetAllWaypointPaths()
{
  map_data->resetAllWaypointPaths();
}

void MapFrame::resetWaypointPathForSelectedRover(string rover)
{
  map_data->resetWaypointPathForSelectedRover(rover);
}

void MapFrame::receiveWaypointReached(int waypoint_id)
{
  map_data->reachedWaypoint(waypoint_id);
}

void MapFrame::receiveCurrentRoverName( QString rover_name )
{
  this->rover_currently_selected = rover_name.toStdString();

  // Forward the change to the popout map if it exists
  if (popout_mapframe)
  {
      popout_mapframe->receiveCurrentRoverName(rover_name);
  }
}

void MapFrame::enableWaypoints(string rover_name)
{
  map_data->setManualMode(rover_name);
}

void MapFrame::disableWaypoints(string rover_name)
{
  map_data->setAutonomousMode(rover_name);
}

MapFrame::~MapFrame()
{
  // Safely erase map data - locks to make sure a frame isnt being drawn
  // clearMap();
  if (popout_window) delete popout_window;
}

}
#endif
