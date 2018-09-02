/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_R3xSO3_HPP
#define INTERPOL_R3xSO3_HPP

#include <Eigen/Geometry>

namespace interpol {

class R3xSO3
{
public:
    Eigen::Quaterniond quat;
    Eigen::Vector3d tra;

    R3xSO3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) : quat(q), tra(t) {}

    R3xSO3& operator *=(const R3xSO3& other){
        quat *= other.quat;
        tra += other.tra;
        return *this;
    }
};

static Eigen::Vector3d tra(const R3xSO3& rr){
    return rr.tra;
}
static Eigen::Quaterniond rot(const R3xSO3& rr){
    return rr.quat;
}

} // ns interpol

#endif // INTERPOL_R3xSO3_HPP
