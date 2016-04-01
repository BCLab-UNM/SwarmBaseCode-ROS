#include "GazeboSimManager.h"
#include <QDir>
#include <string>
#include <unistd.h>
#include <iostream>
#include <utility> // For pair

using namespace std;

GazeboSimManager::GazeboSimManager()
{
    gazebo_client_process = NULL;
    gazebo_server_process = NULL;
    command_process = NULL;

    // Set the app_root by reading the evvironment variable SWARMATHON_APP_ROOT ideally set by the run.sh script.
    const char *name = "SWARMATHON_APP_ROOT";
    char *app_root_cstr;
    app_root_cstr = getenv(name);
    app_root = QString(app_root_cstr);
}

QProcess* GazeboSimManager::startGazeboServer()
{
    if (gazebo_server_process != NULL) return gazebo_server_process;

    gazebo_server_process = new QProcess();

    QString command = QString("rosrun gazebo_ros gzserver ") + app_root + "/simulation/worlds/swarmathon.world";

    gazebo_server_process->startDetached(command);

    gazebo_server_process->waitForStarted();

    return gazebo_server_process;
}

QProcess* GazeboSimManager::startGazeboClient()
{
    if (gazebo_client_process != NULL) return gazebo_client_process;

    gazebo_client_process = new QProcess();

    QString command = QString("rosrun gazebo_ros gzclient");

    gazebo_client_process->start(command);

   // gazebo_client_process->waitForStarted();

    return gazebo_client_process;
}

QString GazeboSimManager::stopGazeboServer()
{
    if (gazebo_server_process == NULL) return "Gazebo server is not running";

    QString argument = "pkill gzserver";
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    QString output = sh.readAll();
    sh.close();

    gazebo_server_process->close();
    cleanUpGazeboServer();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

void GazeboSimManager::cleanUpGazeboServer()
{
    // Use delete later here because this function is called by a slot connected to a signal from this object which will segfault if delete directely with "delete gazebo_client_process;"
    gazebo_server_process->deleteLater();
    gazebo_server_process = NULL;
    model_locations.clear();
}

void GazeboSimManager::cleanUpGazeboClient()
{
    // Use delete later here because this function is called by a slot connected to a signal from this object which will segfault if delete directely with "delete gazebo_client_process;"
    gazebo_client_process->deleteLater();
    gazebo_client_process = NULL;
}

QString GazeboSimManager::stopGazeboClient()
{
    if (gazebo_client_process == NULL) return "Gazebo client is not running";

    QString argument = "pkill gzclient";

    QProcess sh;

    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    gazebo_client_process->close();

    cleanUpGazeboClient();

    return "Simulation visualization ended";
}


QString GazeboSimManager::stopRoverNode( QString rover_name )
{
    if (rover_processes.find(rover_name) == rover_processes.end()) return "Could not stop " + rover_name + " rover process since it does not exist.";

    rover_processes[rover_name]->terminate();
    rover_processes[rover_name]->waitForFinished();
    rover_processes.erase(rover_name);

    // Kill nodes
    QString argument = "rosnode kill "+rover_name+"_MOBILITY";
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    QString output = sh.readAll();
    sh.close();

    argument = "rosnode kill "+rover_name+"_NAVSAT";
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    output += "<br>" + sh.readAll();
    sh.close();

    argument = "rosnode kill "+rover_name+"_TARGET";
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    output += "<br>" + sh.readAll();
    sh.close();

    argument = "rosnode kill "+rover_name+"_OBSTACLE";
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    output += "<br>" + sh.readAll();
    sh.close();

    argument = "rosnode kill "+rover_name+"_EKF";
    sh.start("sh", QStringList() << "-c" << argument);
    sh.waitForFinished();
    output += "<br>" + sh.readAll();
    sh.close();

    return output+"<br>rover process " + rover_name + " terminated along with <font color='green'>navsat, target, obstacle, and mobility</font> nodes";
}

QString GazeboSimManager::startRoverNode( QString rover_name )
{
    QString argument = "roslaunch "+app_root+"/launch/"+rover_name+".launch name:="+rover_name;

    QProcess* rover_process = new QProcess();

    rover_processes[rover_name] = rover_process;

    rover_process->start("sh", QStringList() << "-c" << argument);

    return "rover process spawned";
}

QString GazeboSimManager::addGroundPlane( QString ground_name )
{
    QString argument = QString("rosrun gazebo_ros spawn_model -sdf -file ")+app_root+"/simulation/models/" + ground_name + "/model.sdf -model " + ground_name;
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimManager::addRover(QString rover_name, float x, float y, float z)
{
    float rover_clearance = 0.45; //meters
    model_locations.insert(make_tuple(x, y, rover_clearance));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file "+app_root+"/simulation/models/" + rover_name + "/model.sdf "
               + "-model " + rover_name
               + " -x " + QString::number(x)
               + " -y " + QString::number(y)
               + " -z " + QString::number(z);
               + " -Y " + QString::number(M_PI);

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimManager::removeRover( QString rover_name)
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

QString GazeboSimManager::removeGroundPlane( QString ground_name )
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

QString GazeboSimManager::addModel(QString model_name, QString unique_id, float x, float y, float z, float clearance)
{
    return addModel(model_name, unique_id, x, y, z, 0, 0, 0, clearance);
}

QString GazeboSimManager::addModel(QString model_name, QString unique_id, float x, float y, float z, float roll, float pitch, float yaw, float clearance)
{
    model_locations.insert(make_tuple(x, y, clearance));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file "+app_root+"/simulation/models/" + model_name + "/model.sdf "
            + "-model " + unique_id
            + " -x "    + QString::number(x)
            + " -y "    + QString::number(y)
            + " -z "    + QString::number(z)
            + " -R "    + QString::number(roll)
            + " -P "    + QString::number(pitch)
            + " -Y "    + QString::number(yaw);

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimManager::removeModel( QString model_name )
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

QString GazeboSimManager::moveRover(QString rover_name, float x, float y, float z)
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

QString GazeboSimManager::applyForceToRover(QString rover_name, float x, float y, float z, float duration)
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

// Takes the center x and center y positions of an object along with its clearance and checks if any objects are within that area
bool GazeboSimManager::isLocationOccupied(float x, float y, float clearance)
{
    set<tuple<float, float, float>>::iterator it;
    for (it = model_locations.begin(); it != model_locations.end(); it++)
    {
        float used_x = get<0>(*it);
        float used_y = get<1>(*it);
        float used_clearance = get<2>(*it);

        // Distance between circle centers
        float d = sqrt(pow(x-used_x,2)+pow(y-used_y,2));
        if (d < clearance+used_clearance)
        {
            return true;
        }
    }

    return false;
}

bool GazeboSimManager::isGazeboServerRunning()
{
    return gazebo_server_process != NULL;
}


bool GazeboSimManager::isGazeboClientRunning()
{
    return gazebo_server_process != NULL;
}

GazeboSimManager::~GazeboSimManager()
{
    stopGazeboServer();
    stopGazeboClient();
    if (gazebo_server_process) gazebo_server_process->close();
    if (gazebo_client_process) gazebo_server_process->close();
    if (command_process) command_process->close();
    delete gazebo_client_process;
    delete gazebo_server_process;
    delete command_process;
}

