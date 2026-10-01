#pragma once

#include <span>
#include <vector>

namespace robo_arch {

// Stateless joint effort feedback. q is in rad, v in rad/s, and torque in N m.
// kp has units N m/rad; kd has units N m s/rad. All spans must have equal,
// nonzero length and contain finite values. Gains must be nonnegative.
std::vector<double> JointPd(std::span<const double> q,
                            std::span<const double> v,
                            std::span<const double> q_des,
                            std::span<const double> v_des,
                            std::span<const double> feedforward,
                            std::span<const double> kp,
                            std::span<const double> kd);

}  // namespace robo_arch
