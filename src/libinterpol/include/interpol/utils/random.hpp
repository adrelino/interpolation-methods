/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_RANDOM_HPP
#define INTERPOL_RANDOM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace interpol {

namespace random {
    double numberUniform();
    double numberNormal();

    Eigen::Vector3d vectorUniform(double scale=1.0);
    Eigen::Vector3d vectorNormal(double sigma=1.0);

    enum class QuaternionSampling {Hypersphere, AngleAxis, Shoemake, NormalDistribution};
    Eigen::Quaterniond quaternionUniform(QuaternionSampling method=QuaternionSampling::Hypersphere);
} // ns random

} // ns interpol

#endif // INTERPOL_RANDOM_HPP
