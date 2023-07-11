#include <gtest/gtest.h>
#include <quaternion.hpp>


TEST(Quaternion, ClassInitialisation) {
  
	yadq::quaternion<double> q;

    
    EXPECT_TRUE(q.w() == 1);
    EXPECT_TRUE(q.x() == 0);
    EXPECT_TRUE(q.y() == 0);
    EXPECT_TRUE(q.z() == 0);
    
}