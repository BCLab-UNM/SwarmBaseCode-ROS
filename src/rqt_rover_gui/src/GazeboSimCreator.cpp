#include "GazeboSimCreator.h"
#include <string>
#include <unistd.h>
#include <iostream>

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

    QString return_msg = ""; //command_process->readAll();
    return return_msg;
}

QString GazeboSimCreator::addGroundPlane( QString ground_name )
{
    QStringList arguments;

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/mars_ground_plane/model.sdf -model mars_ground_plane");

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addRover( QString rover_name, float x, float y )
{

    QStringList arguments;

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/alpha/model.sdf -model alpha" << "-x" << 0  << "-y" << 0);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeRover( QString rover_name)
{

    QStringList arguments;

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << "rosservice call gazebo/delete_model '{model_name: alpha}'");

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

GazeboSimCreator::~GazeboSimCreator()
{

    gazebo_process->close();
    command_process->close();
    delete gazebo_process;
    delete command_process;
}

