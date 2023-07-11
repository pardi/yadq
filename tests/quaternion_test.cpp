#include <gtest/gtest.h>
#include <quaternion.hpp>

#define TOLERANCE (1e-5)

TEST(Quaternion, DefaultClassInitialisation) {
  
	yadq::quaternion<double> q;

    
    EXPECT_TRUE(q.w() == 1);
    EXPECT_TRUE(q.x() == 0);
    EXPECT_TRUE(q.y() == 0);
    EXPECT_TRUE(q.z() == 0);
    
}


TEST(Quaternion, ClassInitialisation) {
  
	yadq::quaternion<double> q(1, 2, 3, 4);

    
    EXPECT_TRUE(q.w() == 1);
    EXPECT_TRUE(q.x() == 2);
    EXPECT_TRUE(q.y() == 3);
    EXPECT_TRUE(q.z() == 4);
    
}


TEST(Quaternion, Normalisation) {
  
	yadq::quaternion<double> q(1, 2, 3, 4);

    q.normalise();

    
    EXPECT_NEAR(q.w(), (1.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.x(), (2.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.y(), (3.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.z(), (4.0 / 5.47722557505), TOLERANCE);
    
}