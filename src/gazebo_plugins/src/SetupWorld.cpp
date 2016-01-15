#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <iostream>

using namespace std;

namespace gazebo
{
  class SetupWorld : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      cout << "Setting up world..." << flush;

      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/physics topic
      transport::PublisherPtr physicsPub =
        node->Advertise<msgs::Physics>("~/physics");

      msgs::Physics physicsMsg;
      physicsMsg.set_type(msgs::Physics::ODE);

      // Set the step time
      physicsMsg.set_max_step_size(0.01);

      // Change gravity
      //msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1));
      
      physicsPub->Publish(physicsMsg);

      cout << " done." << endl;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(SetupWorld)
}
