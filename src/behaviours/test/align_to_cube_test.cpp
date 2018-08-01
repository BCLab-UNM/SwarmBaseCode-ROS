#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AlignToCube.hpp"
#include "SwarmieSensors.hpp"
#include "SwarmieInterface.hpp"
#include "Tag.hpp"

class AlignToCubeTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   AlignToCube align;
   boost::math::quaternion<double> defaultOrientation;

   AlignToCubeTest() : align(&sensors), defaultOrientation(1.2, 1.2, 1.2, 2.1) { align.Update(); }
};

TEST_F(AlignToCubeTest, noCubeNoMovement) {
   SwarmieAction a = align.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AlignToCubeTest, nonCubeTagNoMovement) {
   Tag t = tag_top_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   align.Update();
   SwarmieAction a = align.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AlignToCubeTest, cubeOnLeftTurnLeft) {
   sensors.DetectedTag(tag_top_left(Tag::CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_left(align.GetAction())) << "Yaw: " << align.GetAction().GetVelocity().GetYaw();

   sensors.ClearDetections();
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction()));

   sensors.DetectedTag(tag_bottom_left(Tag::CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 50 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_left(align.GetAction())) << "Yaw: " << align.GetAction().GetVelocity().GetYaw();
}

TEST_F(AlignToCubeTest, cubeOnRightTurnRight) {
   sensors.DetectedTag(tag_top_right(Tag::CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_right(align.GetAction()));

   sensors.ClearDetections();
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction()));

   sensors.DetectedTag(tag_bottom_right(Tag::CUBE_TAG_ID));
   align.Update();
   // accumulate integral term in the PI controller
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_right(align.GetAction()));
}

TEST_F(AlignToCubeTest, nestAndCubeVisible) {
   sensors.DetectedTag(tag_top_left(Tag::CUBE_TAG_ID));
   sensors.DetectedTag(tag_bottom_left(Tag::NEST_TAG_ID));
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction()));
}

TEST_F(AlignToCubeTest, movementStopsWhenTagVanishes)
{
   sensors.DetectedTag(tag_top_left(Tag::CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_moving(align.GetAction()));
   sensors.ClearDetections();
   // Update many times to test that the integral term is no longer
   // having an effect.
   for(int i = 0; i < 1000; i++) {
      align.Update();
   }
   EXPECT_FALSE(is_moving(align.GetAction()));
}

TEST_F(AlignToCubeTest, noMovementWhenAligned)
{
   sensors.DetectedTag(Tag(Tag::CUBE_TAG_ID, -0.023, -0.12, 0.5, defaultOrientation));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_FALSE(is_moving(align.GetAction()));
}