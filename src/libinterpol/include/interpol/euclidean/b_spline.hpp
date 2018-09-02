/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_B_SPLINE_HPP
#define INTERPOL_B_SPLINE_HPP

#include <Eigen/Dense>

namespace interpol {

namespace b_spline {

//important: do not use c++11 auto with Eigen expressions: https://eigen.tuxfamily.org/dox/TopicPitfalls.html

/**
 * Basis form
 */
const Eigen::Matrix4d C_basis = (Eigen::Matrix4d() << \
        -1/6.0,  3/6.0, -3/6.0, 1/6.0,
        3/6.0, -6/6.0,  3/6.0, 0/6.0,
        -3/6.0,  0/6.0,  3/6.0, 0/6.0,
        1/6.0,  4/6.0,  1/6.0, 0/6.0).finished();

template<typename T>
Eigen::Matrix<T, 1, 4> spline_B_basis(double u) {
    return (Eigen::RowVector4d(u*u*u,u*u,u,1)*C_basis).cast<T>();
}

template<typename T>
Eigen::Matrix<T, 1, 4> spline_B_basis_dot(double u) {
    return (Eigen::RowVector4d(3*u*u,2*u,1,0)*C_basis).cast<T>();
}

template<typename T>
Eigen::Matrix<T, 1, 4> spline_B_basis_dotdot(double u) {
    return (Eigen::RowVector4d(6*u,2,0,0)*C_basis).cast<T>();
}


/**
 * Cumulative form
 */
const Eigen::Matrix4d C_cumul = (Eigen::Matrix4d() << \
        6.0 / 6.0 , 0.0       , 0.0        , 0.0        ,
        5.0 / 6.0 , 3.0 / 6.0 , -3.0 / 6.0 , 1.0 / 6.0  ,
        1.0 / 6.0 , 3.0 / 6.0 , 3.0 / 6.0  , -2.0 / 6.0 ,
        0.0       , 0.0       , 0.0        ,  1.0 / 6.0).finished();

template<typename T>
Eigen::Matrix<T, 4, 1> spline_B(double u) {
    return (C_cumul*Eigen::Vector4d(1,u,u*u,u*u*u)).cast<T>();
}

template<typename T>
Eigen::Matrix<T, 4, 1> spline_B_dot(double u, double dt) {
    return (C_cumul*Eigen::Vector4d(0,1/dt,(2*u)/dt,(3*u*u)/dt)).cast<T>();
}

template<typename T>
Eigen::Matrix<T, 4, 1> spline_B_dotdot(double u, double dt) {
    double dt2 = dt * dt;
    return (C_cumul*Eigen::Vector4d(0,0,2/dt2,(6*u)/dt2)).cast<T>();
}

} // ns b_spline

} // ns interpol

#endif //INTERPOL_B_SPLINE_HPP