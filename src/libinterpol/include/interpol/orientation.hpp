/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_ORIENTATION_HPP
#define INTERPOL_ORIENTATION_HPP

#include "euclidean.hpp"
#include "orientation/SU2.hpp"
#include <iomanip>

namespace interpol {

template <typename T>
static T LERPN(const T& lhs, double u, const T& rhs) {
    return LERP(lhs,u,rhs).normalized();
}

static auto QLB = LERPN<Eigen::Quaterniond>;

template <typename T>
static T SLERP(const T& lhs, double u, const T& rhs) {
    return lhs*expq(logq(lhs.conjugate()*rhs)*u);
}


} // ns interpol

#endif // INTERPOL_ORIENTATION_HPP
