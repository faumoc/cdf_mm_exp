
#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ostream>
#include <vector>

namespace mm_ros_control {
    using scalar_t = double;
    using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
}  // namespace mm_ros_control