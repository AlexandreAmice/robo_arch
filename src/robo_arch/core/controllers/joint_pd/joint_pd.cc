#include "robo_arch/core/controllers/joint_pd/joint_pd.h"

#include <cmath>
#include <stdexcept>

namespace robo_arch {

std::vector<double> JointPd(std::span<const double> q,
                            std::span<const double> v,
                            std::span<const double> q_des,
                            std::span<const double> v_des,
                            std::span<const double> feedforward,
                            std::span<const double> kp,
                            std::span<const double> kd) {
  for (const auto values : {q, v, q_des, v_des, feedforward, kp, kd}) {
    if (values.empty() || values.size() != q.size()) {
      throw std::invalid_argument(
          "Joint vectors must have equal nonzero length");
    }
    for (double value : values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Joint vectors must be finite");
      }
    }
  }
  std::vector<double> result(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (kp[i] < 0 || kd[i] < 0) {
      throw std::invalid_argument("Gains must be nonnegative");
    }
    result[i] =
        feedforward[i] + kp[i] * (q_des[i] - q[i]) + kd[i] * (v_des[i] - v[i]);
    if (!std::isfinite(result[i])) {
      throw std::overflow_error("Joint effort overflow");
    }
  }
  return result;
}

}  // namespace robo_arch
