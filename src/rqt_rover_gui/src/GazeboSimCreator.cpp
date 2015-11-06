#include "GazeboSimCreator.h"
#include <string>
#include <unistd.h>
#include <iostream>
#include <utility> // For pair

using namespace std;

GazeboSimCreator::GazeboSimCreator()
{
    gazebo_process = NULL;
    command_process = NULL;
}

QString GazeboSimCreator::startGazebo()
{
    if (gazebo_process != NULL)
        return "Gazebo already running";

    gazebo_process = new QProcess();
    gazebo_process->startDetached("rosrun gazebo_ros gazebo");//QStringList() << "/home/oDx/Documents/a.txt");

    gazebo_process->waitForStarted();
    QString return_msg = ""; //command_process->readAll();
    return return_msg;
}

QString GazeboSimCreator::stopGazebo()
{
    // Close rover processes
    for(map<QString, QProcess*>::iterator it = rover_processes.begin(); it != rover_processes.end(); it++)
    {
        it->second->close();
        it->second->waitForFinished();
    }

    rover_processes.clear(); // calls destructors for us.

    if (gazebo_process == NULL) return "Gazebo is not running";

    QString argument = "~/rover_workspace/src/rover_driver_gazebo_launch/cleanup.sh";

    QProcess sh;

    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    cout << return_msg.toStdString() << endl;

    gazebo_process->close();
//    gazebo_process->waitForFinished();
//    gazebo_process->terminate();
//    delete gazebo_process;
    delete gazebo_process;
    gazebo_process = NULL;
      return return_msg;
}

QString GazeboSimCreator::startRoverNode( QString rover_name )
{
    QString argument = "roslaunch ~/rover_workspace/src/rover_driver_gazebo_launch/launch_files/"+rover_name+".launch name:="+rover_name;

    QProcess* rover_process = new QProcess();

    rover_processes.insert(pair<QString, QProcess*>(rover_name, rover_process));

    rover_process->start("sh", QStringList() << "-c" << argument);

    return "rover process spawned";
}

QString GazeboSimCreator::addGroundPlane( QString ground_name )
{
    model_locations.insert(make_tuple(0,0)); // Nest location

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/" + ground_name + "/model.sdf -model " + ground_name;
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addRover(QString rover_name, float x, float y, float z)
{
    model_locations.insert(make_tuple(x, y));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/" + rover_name + "/model.sdf -model " + rover_name + " -x " + QString::number(x) + " -y " + QString::number(y)+ " -z " + QString::number(z);
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeRover( QString rover_name)
{
    QString argument;
    argument = "rosservice call gazebo/delete_model '{model_name: "+rover_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeGroundPlane( QString ground_name )
{
    QString argument = "rosservice call gazebo/delete_model '{model_name: "+ground_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addModel(QString model_name, float x, float y, float z)
{
    model_locations.insert(make_tuple(x, y));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/" + model_name + "/model.sdf -model " + model_name + " -x " + QString::number(x) + " -y " + QString::number(y)+ " -z " + QString::number(z);
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeModel( QString model_name )
{
    QString argument = "rosservice call gazebo/delete_model '{model_name: "+model_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::moveRover(QString rover_name, float x, float y, float z)
{
            QString x_str = QString::number(x,'f',1);
            QString y_str = QString::number(y,'f',1);
            QString z_str = QString::number(z,'f',1);
           // QString duration_str = QString::number(duration,'f',1);

            QString argument = "rosservice call /gazebo/set_model_state '{model_state: { model_name: alpha, pose: { position: { x: 5, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 1, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'";

            QProcess sh;
            sh.start("sh", QStringList() << "-c" << argument);

            sh.waitForFinished();
            QByteArray output = sh.readAll();
            sh.close();

            QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

            return return_msg;
}

QString GazeboSimCreator::applyForceToRover(QString rover_name, float x, float y, float z, float duration)
{
            QString x_str = QString::number(x,'f',1);
            QString y_str = QString::number(y,'f',1);
            QString z_str = QString::number(z,'f',1);
           // QString duration_str = QString::number(duration,'f',1);

            QString argument = "rosservice call /gazebo/apply_body_wrench '{ body_name: \"beta::base_link\", reference_frame: \"beta::base_link\", wrench: { force: { x: 1000, y: 0, z: 0 } }, start_time: 0, duration: 2 }'";

            QProcess sh;
            sh.start("sh", QStringList() << "-c" << argument);

            sh.waitForFinished();
            QByteArray output = sh.readAll();
            sh.close();

            QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

            return return_msg;
}

bool GazeboSimCreator::isLocationOccupied(float x, float y, float clearance)
{
    set<tuple<float, float>>::iterator it;
    for (it = model_locations.begin(); it != model_locations.end(); it++)
    {
        float used_x = get<0>(*it);
        float used_y = get<1>(*it);

        if (fabs(used_x - x) < clearance || fabs(used_y - y) < clearance)
        {
            return true;
        }
    }

    return false;
}

bool GazeboSimCreator::isGazeboRunning()
{
    return gazebo_process != NULL;
}

GazeboSimCreator::~GazeboSimCreator()
{
    stopGazebo();
    if (gazebo_process) gazebo_process->close();
    if (command_process) command_process->close();
    delete gazebo_process;
    delete command_process;
}

