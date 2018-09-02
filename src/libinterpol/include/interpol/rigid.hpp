/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_RIGID_HPP
#define INTERPOL_RIGID_HPP

#include "orientation.hpp"
#include "rigid/R3xSO3.hpp"
#include "rigid/DH.hpp"
#include "rigid/SE3.hpp"

namespace interpol {

//SPLIT Interpolation in R3xSO3
static R3xSO3 SPLIT(const R3xSO3& c0, double u, const R3xSO3& c1){
    return R3xSO3(SLERP(c0.quat,u,c1.quat),c0.tra*(1-u) + c1.tra*u);
}

static R3xSO3 SPAPP(const R3xSO3& c0, double u, const R3xSO3& c1){
    return R3xSO3(QLB(c0.quat,u,c1.quat),c0.tra*(1-u) + c1.tra*u);
}


//JOINT INTERPOLATION


//Taken from Ben Kenwright 2012: Dual-Quaternions: From Classical Mechanics to Computer Graphics and Beyond
static DH1d ScLERPGeometricCosSin(const DH1d& from, double u, const DH1d& toO){
    DH1d to(toO);
    // Shortest path
    double dot = from.real().dot(to.real());
    if ( dot < 0 ) to = to * -1.0;

    // ScLERP = qa(qa^-1 qb)^t
    auto diff = from.conjugate() * to;
    Eigen::Vector3d vr(diff.real().x(), diff.real().y(), diff.real().z());
    Eigen::Vector3d vd(diff.dual().x(), diff.dual().y(), diff.dual().z());
    double invr = 1.0 / vr.norm(); // == 1.0 / std::sqrt(vr.dot(vr));

    // Screw parameters
    double  angle = 2.0 * std::acos( diff.real().w() );
    double  pitch = -2.0 * diff.dual().w() * invr;
    Eigen::Vector3d direction = vr * invr;
    Eigen::Vector3d moment = (vd - direction*pitch*diff.real().w()*0.5)*invr;

    //cout<<"directionNorm="<<direction.norm()<<"momentNorm="<<moment.norm()<<endl; //why is moment.norm 1.15 but it still works?

    // Exponential power
    angle *= u;
    pitch *= u;

    // Convert back to dual-quaternion
    double sinAngle = std::sin(0.5*angle);
    double cosAngle = std::cos(0.5*angle);

    Eigen::Vector3d axisReal = direction * sinAngle;
    Eigen::Quaterniond real(cosAngle, axisReal.x(),axisReal.y(),axisReal.z());

    Eigen::Vector3d axisDual = sinAngle*moment+pitch*0.5* cosAngle *direction;
    Eigen::Quaterniond dual(-pitch*0.5*sinAngle, axisDual.x(),axisDual.y(),axisDual.z());

    // Complete the multiplication and return the interpolated value
    return from * DH1d( real, dual );

}

//This gives same result as ScLERPGeometricCosSin, but uses Eigen::AngleAxis for some conversions instead
static DH1d ScLERPAnalyticExpLogScrewTangent(const DH1d& from, double u, const DH1d& to, bool useBenKenwrightImpl = false){
    DH1Tangentd omega8x = (from.conjugate()*to).logT(useBenKenwrightImpl);
    //Tangent * scalar only multiplies the w of the real and dual quaternions!
    omega8x = omega8x * u;
    return from*(omega8x).expT();
}

//Kavan dual quaternion blending (2007 Skinning with dual quaternions)
/*static DualQuaterniond DLUp(double u, const DualQuaterniond& from, const DualQuaterniond& to){
    return ((1-u)*from + u*to).normalized();
}*/
static auto DLUp = LERPN<DH1d>;



static Eigen::Vector3d tra(const Sophus::SE3d& rr){
    return rr.translation();
}
static Eigen::Quaterniond rot(const Sophus::SE3d& rr){
    return rr.unit_quaternion();
}
static Eigen::Vector3d tra(const DH1d& rr){
    return rr.translation();
}
static Eigen::Quaterniond rot(const DH1d& rr){
    return rr.rotation();
}

} // ns interpol

#endif // INTERPOL_RIGID_HPP
