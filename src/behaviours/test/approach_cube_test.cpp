#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "ApproachCube.hpp"
#include "SwarmieSensors.hpp"

class ApproachCubeTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   ApproachCube approach;
   boost::math::quaternion<double> defaultOrientation;
   ApproachCubeTest() : approach(&sensors), defaultOrientation(1.2, 1.2, 1.2, 2.1) { approach.Update(); }
};

TEST_F(ApproachCubeTest, noCubeNoMovement)
{
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, nestTagOnlyNoMovement)
{
   Tag t = tag_top_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, nestTagAlignedNoMovement)
{
   Tag t(Tag::NEST_TAG_ID, -0.023, -0.12, 0.5, defaultOrientation);
   ASSERT_TRUE(fabs(t.Alignment()) < 0.01) << "Tag not aligned: " << t.Alignment() << std::endl;
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, tagMisalignedNoMovement)
{
   Tag t = tag_top_left(Tag::CUBE_TAG_ID);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, tagAlignedMovement)
{
   Tag t(Tag::CUBE_TAG_ID, -0.022, -0.12, 0.4, defaultOrientation);
   ASSERT_TRUE(fabs(t.Alignment()) < 0.01) << "Tag not aligned: " << t.Alignment() << std::endl;
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, movementStopsWhenTagVanishes)
{
   Tag t(Tag::CUBE_TAG_ID, -0.022, -0.12, 0.4, defaultOrientation);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
   sensors.ClearDetections();
   // Update many times to test that the integral term is no longer
   // having an effect.
   for(int i = 0; i < 1000; i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

// This is the actual position of cubes when this bug is triggered.
// Tag[0]{ alignment: -0.0582302 | position: (-0.0812302, -0.0454901, 0.426023) | Orientation: (0.954242,-0.0389091,0.160215,-0.24948)}n
// Tag[0]{ alignment: 0.00210978 | position: (-0.0208902, -0.0438522, 0.423104) | Orientation: (0.0378738,0.948888,0.2493,0.189805)}
TEST_F(ApproachCubeTest, approachWhenBackgroundTagMisalignedToLeft)
{
   Tag aligned(Tag::CUBE_TAG_ID, -0.0208902, -0.0438522, 0.423104,
               boost::math::quaternion<double>(0.0378738,0.948888,0.2493,0.189805));
   Tag background(Tag::CUBE_TAG_ID, -0.0812302, -0.0454901, 0.426023,
                  boost::math::quaternion<double>(0.954242,-0.0389091,0.160215,-0.24948));

   sensors.DetectedTag(background);
   sensors.DetectedTag(aligned);

   approach.Update();

   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));

   sensors.ClearDetections();
   approach.Update();

   sensors.DetectedTag(aligned);
   sensors.DetectedTag(background);

   approach.Update();

   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, approachWhenBackgroundTagMisalignedToRight)
{
   Tag aligned(Tag::CUBE_TAG_ID, -0.0208902, -0.0438522, 0.423104,
               boost::math::quaternion<double>(0.0378738,0.948888,0.2493,0.189805));
   Tag background(Tag::CUBE_TAG_ID, 0.1012302, -0.0454901, 0.426023,
                  boost::math::quaternion<double>(0.954242,-0.0389091,0.160215,-0.24948));

   sensors.DetectedTag(background);
   sensors.DetectedTag(aligned);

   approach.Update();

   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));

   sensors.ClearDetections();
   approach.Update();

   sensors.DetectedTag(aligned);
   sensors.DetectedTag(background);

   approach.Update();

   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
}
