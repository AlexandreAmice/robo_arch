#include <memory>
#include <span>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "robo_arch/core/controllers/joint_pd/joint_pd.h"

namespace nb = nanobind;
using Vector =
    nb::ndarray<const double, nb::ndim<1>, nb::c_contig, nb::device::cpu>;

NB_MODULE(_joint_pd, module) {
  module.def(
      "compute",
      [](Vector q, Vector v, Vector q_des, Vector v_des, Vector feedforward,
         Vector kp, Vector kd) {
        const auto span = [](Vector value) {
          return std::span<const double>(value.data(), value.size());
        };
        auto result = std::make_unique<std::vector<double>>(
            robo_arch::JointPd(span(q), span(v), span(q_des), span(v_des),
                               span(feedforward), span(kp), span(kd)));
        auto* storage = result.get();
        nb::capsule owner(storage, [](void* pointer) noexcept {
          delete static_cast<std::vector<double>*>(pointer);
        });
        result.release();
        return nb::ndarray<nb::numpy, double>(storage->data(),
                                              {storage->size()}, owner);
      },
      nb::arg("q").noconvert(), nb::arg("v").noconvert(),
      nb::arg("q_des").noconvert(), nb::arg("v_des").noconvert(),
      nb::arg("feedforward").noconvert(), nb::arg("kp").noconvert(),
      nb::arg("kd").noconvert(),
      "Compute PD plus feedforward. CPU float64 vectors; GIL held; owned "
      "output.");
}
