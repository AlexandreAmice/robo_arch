#include "robo_arch/core/controllers/joint_pd/joint_pd.h"

#include <array>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace robo_arch {
namespace {

TEST(JointPd, IndependentlyLinkableNumericalControl) {
  const std::array q{0.2, -0.3};
  const std::array v{0.1, 0.2};
  const std::array q_des{0.5, 0.4};
  const std::array v_des{0.0, -0.1};
  const std::array ff{2.0, -3.0};
  const std::array kp{10.0, 20.0};
  const std::array kd{2.0, 4.0};
  const auto result = JointPd(q, v, q_des, v_des, ff, kp, kd);
  EXPECT_DOUBLE_EQ(result[0], 4.8);
  EXPECT_DOUBLE_EQ(result[1], 9.8);
}

TEST(JointPd, RejectsOverflow) {
  const std::array huge{std::numeric_limits<double>::max()};
  const std::array zero{0.0};
  const std::array two{2.0};
  EXPECT_THROW(JointPd(zero, zero, huge, zero, zero, two, zero),
               std::overflow_error);
}

}  // namespace
}  // namespace robo_arch
