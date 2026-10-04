#pragma once

#include <span>
#include <vector>

namespace robo_arch {

/// Compute stateless joint PD feedback plus feedforward effort.
///
/// Input vectors must have equal, nonzero length and contain finite values.
/// Gains must be nonnegative. The caller supplies consistent joint ordering.
/// @param q Measured joint positions in rad.
/// @param v Measured joint velocities in rad/s.
/// @param q_des Desired joint positions in rad.
/// @param v_des Desired joint velocities in rad/s.
/// @param feedforward Feedforward joint efforts in N m.
/// @param kp Proportional gains in N m/rad.
/// @param kd Derivative gains in N m s/rad.
/// @returns Joint efforts: feedforward + kp * (q_des - q) + kd * (v_des - v).
std::vector<double> JointPd(std::span<const double> q,
                            std::span<const double> v,
                            std::span<const double> q_des,
                            std::span<const double> v_des,
                            std::span<const double> feedforward,
                            std::span<const double> kp,
                            std::span<const double> kd);

}  // namespace robo_arch
