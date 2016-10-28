#include "ScorePlugin.h"

using namespace gazebo;
using namespace std;

// required overloaded function from ModelPlugin class
void ScorePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    score = 0;
    model = _model;
    sdf = _sdf;

    // set the update period (number of updates per second) for this plugin
    previousUpdateTime = model->GetWorld()->GetSimTime();
    loadUpdatePeriod();

    // Create a ros node
    rosNode.reset(new ros::NodeHandle(string(model->GetName()) + "_score"));

    // Create publisher so we can send info messages to the UI
    scorePublisher = rosNode->advertise<std_msgs::String>(loadPublisherTopic(), 1, true);

    // Connect the updateWorldEventHandler function to Gazebo;
    // ConnectWorldUpdateBegin sets our handler to be called at the beginning of
    // each physics update iteration
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ScorePlugin::updateWorldEventHandler, this)
    );
}

// Gazebo actuation function
void ScorePlugin::updateWorldEventHandler() {
    common::Time currentTime = model->GetWorld()->GetSimTime();

    if((currentTime - previousUpdateTime).Float() < updatePeriodInSeconds) {
        return;
    }
    previousUpdateTime = currentTime;

    updateScore();

    std_msgs::String msg;
    msg.data = std::to_string(score);
    scorePublisher.publish(msg);
}

/**
 * This function loads the publisher topic to be used by this plugin from the
 * configuration XML file.
 */
std::string ScorePlugin::loadPublisherTopic() {
    string topic;

    if(sdf->HasElement("scoreTopic")) {
        topic = sdf->GetElement("scoreTopic")->Get<std::string>();
    } else {
        ROS_ERROR_STREAM("[Score Plugin : " << model->GetName()
            << "]: In ScorePlugin.cpp: loadPublisherTopic(): No <scoreTopic> "
            << "tag is defined in the model SDF file");
        exit(1);
    }

    return topic;
}

/**
 * This function loads the update rate from the SDF configuration file and uses
 * that value to set the update period. Effectively, the updatePeriod variable
 * defines how many times per second the plugin will publish.
 */
void ScorePlugin::loadUpdatePeriod() {
  float updateRate = 1.0;

  if(!sdf->HasElement("updateRate")) {
    ROS_INFO_STREAM("[Score Plugin : " << model->GetName()
      << "]: In ScorePlugin.cpp: loadUpdatePeriod(): "
      << "missing <updateRate> tag, defaulting to 0.1");
    updateRate = 0.1;
  } else {
    updateRate = sdf->GetElement("updateRate")->Get<float>();

    // fatal error: the update cannot be <= 0 and especially cannot = 0
    if(updateRate <= 0) {
      ROS_ERROR_STREAM("[Score Plugin : " << model->GetName()
        << "]: In ScorePlugin.cpp: loadUpdatePeriod(): "
        << "updateRate = " << updateRate << ", updateRate cannot be <= 0.0");
      exit(1);
    }
  }

  // set the update period for this plugin: the plugin will refresh at a rate
  // of "updateRate" times per second, i.e., at "updatePeriodInSeconds" hertz
  updatePeriodInSeconds = 1.0 / updateRate;
}

/**
 * This function updates the score (an int from 0 - 256) indicating how many
 * tags have been returned to the nest.
 */
void ScorePlugin::updateScore() {
    // instead of what I have here, replace this with a subscriber event handler
    // getting contact info with the nest

    /*
    modelList = model->GetWorld()->GetModels();
    math::Pose pose;

    for(int i = 0; i < modelList.size(); i++) {
        pose = modelList[i]->GetWorldPose();
        //cout << pose << endl;
    }
    */
}

ScorePlugin::~ScorePlugin() {
    rosNode->shutdown(); // Shutdown the ROS node

    // Stop the multi-threaded ROS spinner
    gazebo::shutdown();
}
