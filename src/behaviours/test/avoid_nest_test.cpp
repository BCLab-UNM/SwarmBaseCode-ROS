#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AvoidNest.hpp"
#include "SwarmieSensors.hpp"

TEST(AvoidNest, noTagsNoAction) {
   SwarmieSensors sensors;
   AvoidNest avoid(&sensors);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST(AvoidNest, nonCenterTagNoAction) {
   SwarmieSensors sensors;
   AvoidNest avoid(&sensors);
   avoid.Update();
   Tag t(0, 1, 1, 1, boost::math::quaternion<double>(1.2, 1.2, 1.2, 2.1));
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST(AvoidNest, centerTagTriggersMovement) {
   SwarmieSensors sensors;
   AvoidNest avoid(&sensors);
   avoid.Update();
   Tag t(256, 1, 1, 1, boost::math::quaternion<double>(1.2, 1.2, 1.2, 2.1));
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_moving(a));
}
