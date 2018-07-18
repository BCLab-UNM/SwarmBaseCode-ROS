#include <gtest/gtest.h>

#include "test_util.h"
#include "PickUpCube.hpp"
#include "MockTimer.hpp"

class PickUpCubeTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   MockTimer timer;
   PickUpCube pickup;

   PickUpCubeTest() : pickup(&sensors, &timer) {
      pickup.Update();
   }
};

TEST_F(PickUpCubeTest, noActionWhenNoCubes)
{
   ASSERT_FALSE(is_moving(pickup.GetAction()));
}

TEST_F(PickUpCubeTest, noActionWhenMisaligned)
{
   sensors.DetectedTag(tag_top_left(Tag::CUBE_TAG_ID));
   pickup.Update();
   EXPECT_FALSE(is_moving(pickup.GetAction()));
   sensors.ClearDetections();

   sensors.DetectedTag(tag_top_right(Tag::CUBE_TAG_ID));
   pickup.Update();
   EXPECT_FALSE(is_moving(pickup.GetAction()));
   sensors.ClearDetections();

   sensors.DetectedTag(tag_bottom_right(Tag::CUBE_TAG_ID));
   pickup.Update();
   EXPECT_FALSE(is_moving(pickup.GetAction()));
   sensors.ClearDetections();

   sensors.DetectedTag(tag_bottom_left(Tag::CUBE_TAG_ID));
   pickup.Update();
   EXPECT_FALSE(is_moving(pickup.GetAction()));
   sensors.ClearDetections();
}

#if 0 // not ready to implement these tests yet.
TEST_F(PickUpCubeTest, noActionWhenOnlyDetectingNest)
{
   Tag t = tag_aligned_close(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   pickup.Update();
   ASSERT_FALSE(is_moving(pickup.GetAction()));
}

TEST_F(PickUpCubeTest, moveForwardWhenCubeAlignedAndClose)
{
   // the timer should be set on entering the last inch state
   EXPECT_CALL(timer, StartOnce).Times(1);

   Tag t = tag_aligned_close(Tag::CUBE_TAG_ID);
   sensors.DetectedTag(t);
   pickup.Update();
   ASSERT_TRUE(is_moving_forward());
}
#endif // 0
