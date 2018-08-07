#ifndef _ROBOT_INTERFACE_HPP
#define _ROBOT_INTERFACE_HPP

#include <memory>

#include "Sensors.hpp"
#include "Action.hpp"

namespace core {
   class RobotInterface
   {
   public:

      /**
       * Get the current values of the core sensors.
       */
      virtual const core::Sensors& GetSensors() = 0;

      /**
       * Instruct the robot to exectute the given action.
       */
      virtual void DoAction(const core::Action& action) = 0;
   };
}

namespace mission {
   class RobotInterface
   {
   public:

      /**
       * Get the current values of the mission sensors.
       */
      virtual const mission::Sensors& GetSensors() = 0;

      /**
       * Instruct the robot to execute the given action using the
       * mission actuators.
       */
      virtual void DoAction(const mission::Action& action) = 0;
   };
}

#endif // _ROBOT_INTERFACE_HPP