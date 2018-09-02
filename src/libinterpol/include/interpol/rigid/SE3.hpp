/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_SE3_HPP
#define INTERPOL_SE3_HPP

#include "sophus/se3.hpp"

namespace interpol {

static Sophus::SE3d SE3Up(const Sophus::SE3d& c0,double u,const Sophus::SE3d& c1){
    Sophus::SE3d rel =c0.inverse()*c1;
    return c0*Sophus::SE3d::exp(rel.log() * u);
}

} // ns interpol

#endif // INTERPOL_SE3_HPP
