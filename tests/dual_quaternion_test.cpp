#include <gtest/gtest.h>
#include <yadq/quaternion.hpp>
#include <yadq/dual_quaternion.hpp>

#define TOLERANCE (1e-5)

TEST(DualQuaternion, Constructor) {
  
    yadq::quaternionU<double> qr(0, 0.7071068, 0, 0.7071068);
    yadq::quaternion<double> qd(1, 2, 3, 4);
	yadq::dualquaternion<double> dq(qr, qd);

    EXPECT_NEAR(dq.qr_.w(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq.qr_.x(), 0.7071068, TOLERANCE);
    EXPECT_NEAR(dq.qr_.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq.qr_.z(), 0.7071068, TOLERANCE);

    EXPECT_NEAR(dq.qd_.w(), 1.0, TOLERANCE);
    EXPECT_NEAR(dq.qd_.x(), 2.0, TOLERANCE);
    EXPECT_NEAR(dq.qd_.y(), 3.0, TOLERANCE);
    EXPECT_NEAR(dq.qd_.z(), 4.0, TOLERANCE);

}

TEST(DualQuaternion, ConstructorRotTransl) {
  
    yadq::quaternionU<double> qr(0, 0.7071068, 0, 0.7071068);
	yadq::dualquaternion<double> dq(qr, {1, 0, 0});

    EXPECT_NEAR(dq.qr_.w(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq.qr_.x(), 0.7071068, TOLERANCE);
    EXPECT_NEAR(dq.qr_.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq.qr_.z(), 0.7071068, TOLERANCE);

    EXPECT_NEAR(dq.qd_.w(), -0.353553, TOLERANCE);
    EXPECT_NEAR(dq.qd_.x(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq.qd_.y(), -0.353553, TOLERANCE);
    EXPECT_NEAR(dq.qd_.z(), 0.0, TOLERANCE);
}

TEST(DualQuaternion, Conjugate) {
  
    yadq::quaternionU<double> qr(0, 0.7071068, 0, 0.7071068);
    std::array<double, 3> vec = {1, 1, 1};
	yadq::dualquaternion<double> dq(qr, vec);

    yadq::dualquaternion<double> dq_res = conjugate(dq);

    EXPECT_NEAR(dq_res.qr_.w(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq_res.qr_.x(), -0.7071068, TOLERANCE);
    EXPECT_NEAR(dq_res.qr_.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(dq_res.qr_.z(), -0.7071068, TOLERANCE);

    EXPECT_NEAR(dq_res.qd_.w(), -0.707107, TOLERANCE);
    EXPECT_NEAR(dq_res.qd_.x(), -0.353553, TOLERANCE);
    EXPECT_NEAR(dq_res.qd_.y(), -0.0, TOLERANCE);
    EXPECT_NEAR(dq_res.qd_.z(), 0.353553, TOLERANCE);
}
