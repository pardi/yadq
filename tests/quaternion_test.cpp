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

TEST(Quaternion, MemberNormalisation) {
  
	yadq::quaternion<double> q(1, 2, 3, 4);

    q.normalise();

    EXPECT_NEAR(q.w(), (1.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.x(), (2.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.y(), (3.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.z(), (4.0 / 5.47722557505), TOLERANCE);
    
}

TEST(Quaternion, MemberOperatorPlus) {
  
	yadq::quaternion<double> q1(1, 2, 3, 4);
    yadq::quaternion<double> q2(1, 2, 3, 4);

    q1 += q2;

    EXPECT_NEAR(q1.w(), 2.0, TOLERANCE);
    EXPECT_NEAR(q1.x(), 4.0, TOLERANCE);
    EXPECT_NEAR(q1.y(), 6.0, TOLERANCE);
    EXPECT_NEAR(q1.z(), 8.0, TOLERANCE);
    
}

TEST(Quaternion, MemberOperatorMultiplication) {
  
	yadq::quaternion<double> q1(2, -1, 3, 1);
    yadq::quaternion<double> q2(5, -4, 0, 1);

    q1 *= q2;

    EXPECT_NEAR(q1.w(), 5.0, TOLERANCE);
    EXPECT_NEAR(q1.x(), -10.0, TOLERANCE);
    EXPECT_NEAR(q1.y(), 12.0, TOLERANCE);
    EXPECT_NEAR(q1.z(), 19.0, TOLERANCE);
    
}

TEST(Quaternion, MemberEmpty) {
  
	yadq::quaternion<double> q(0, 0, 0, 0);

    EXPECT_TRUE(q.empty());    
}

TEST(Quaternion, MemberDividedByValue) {
  
	yadq::quaternion<double> q(1, 2, 3, 4);

    const double value = 2.0;

    q /= value;

    EXPECT_NEAR(q.w(), (1.0 / value), TOLERANCE);
    EXPECT_NEAR(q.x(), (2.0 / value), TOLERANCE);
    EXPECT_NEAR(q.y(), (3.0 / value), TOLERANCE);
    EXPECT_NEAR(q.z(), (4.0 / value), TOLERANCE); 
}

TEST(Quaternion, MemberConjugate) {
  
	yadq::quaternion<double> q(1, 2, 3, 4);

    q.conjugate();

    EXPECT_NEAR(q.w(), 1, TOLERANCE);
    EXPECT_NEAR(q.x(), -2, TOLERANCE);
    EXPECT_NEAR(q.y(), -3, TOLERANCE);
    EXPECT_NEAR(q.z(), -4, TOLERANCE); 
}

TEST(QuaternionUnitary, DefaultConstructor) {
  
	yadq::quaternionU<double> q;

    EXPECT_TRUE(q.w() == 1);
    EXPECT_TRUE(q.x() == 0);
    EXPECT_TRUE(q.y() == 0);
    EXPECT_TRUE(q.z() == 0);
    
}

TEST(QuaternionUnitary, Constructor) {
  
	yadq::quaternionU<double> q(1, 2, 3, 4);

    EXPECT_NEAR(q.w(), (1.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.x(), (2.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.y(), (3.0 / 5.47722557505), TOLERANCE);
    EXPECT_NEAR(q.z(), (4.0 / 5.47722557505), TOLERANCE);
}

TEST(QuaternionUnitary, ConstructorAxisAngle) {
  
	yadq::quaternionU<double> q({0.1690309, 0.8451543, 0.5070926}, 0.1745328);

    EXPECT_NEAR(q.w(), 0.9961947, TOLERANCE);
    EXPECT_NEAR(q.x(), 0.014732, TOLERANCE);
    EXPECT_NEAR(q.y(), 0.07366, TOLERANCE);
    EXPECT_NEAR(q.z(), 0.044196, TOLERANCE);
}

TEST(QuaternionUnitary, MemberOperatorEqual) {
  
	yadq::quaternionU<double> q1(1, 0, 1, 0);
    yadq::quaternionU<double> q2 = q1;

    EXPECT_NEAR(q2.w(), q1.w(), TOLERANCE);
    EXPECT_NEAR(q2.x(), q1.x(), TOLERANCE);
    EXPECT_NEAR(q2.y(), q1.y(), TOLERANCE);
    EXPECT_NEAR(q2.z(), q1.z(), TOLERANCE);
}

TEST(QuaternionUnitary, MemberOperatorPlus) {
  
	yadq::quaternionU<double> q1(0, 0.7071068, 0, 0.7071068);
    yadq::quaternionU<double> q2(0, 0.7071068, 0, 0.7071068);

    q1 += q2;

    EXPECT_NEAR(q1.w(), 0, TOLERANCE);
    EXPECT_NEAR(q1.x(), 0.7071068, TOLERANCE);
    EXPECT_NEAR(q1.y(), 0, TOLERANCE);
    EXPECT_NEAR(q1.z(), 0.7071068, TOLERANCE);
}