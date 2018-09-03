/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYSPLINEDUALQUATERNION_H
#define INTERPOL_TRAJECTORYSPLINEDUALQUATERNION_H

#include "TrajectorySpline.h"
#include "interpol/rigid/DH.hpp"
#include "interpol/euclidean/b_spline.hpp"
#include <Eigen/Dense>

namespace interpol {

const static bool useBenKenwrightImpl = false;

template<typename T>
DH1<T> expd(DH1Tangent<T> t){
    return t.expT(useBenKenwrightImpl);
}

template<typename T>
DH1Tangent<T> logd(DH1<T> t){
    return t.logT(useBenKenwrightImpl);
}

/**
 * Dualquaternion Kavan / Ben Kenwright 2013 style spline
 */
class TrajectorySplineDualQuaternion : public TrajectorySpline {

public:

    TrajectorySplineDualQuaternion(Poses3d& poses, bool posesAreWithPhantomPoses, bool approx);

    //overwritten
    Pose3d getPoseAt(uint64_t time) const;

    bool approx;

    template<typename T>
    static Pose3<T> evaluate(double u,const DH1<T>& c0,const DH1<T>& c1,const DH1<T>& c2,const DH1<T>& c3, uint64_t time, uint64_t deltaT){

        Pose3<T> result;

        result.time = time;

        Eigen::Matrix<T,4,1> B_tilde = b_spline::spline_B<T>(u);

        DH1Tangent<T> w1 = logd(c0.conjugate() * c1);
        DH1Tangent<T> w2 = logd(c1.conjugate() * c2);
        DH1Tangent<T> w3 = logd(c2.conjugate() * c3);;

        DH1<T> resultq = c0 * expd(w1*B_tilde[1]) * expd(w2*B_tilde[2]) * expd(w3*B_tilde[3]);


        //Derivatives
//        Eigen::Matrix<T,4,1> B_tilde_dot  = b_spline::spline_B_dot<T>(u,((double)deltaT)*1e-9);

//        //inertial_to_local_frame
//        DualQuaternion<T> q_inv = resultq.conjugate();

//        //rotation 1st deriv (kimkimShin p. 373) in inertial frame
//        DualQuaternion<T> q_dash =  c0 * expq(B_tilde[1] * w1) * (w1 * B_tilde_dot[1]).toDH() * expq(w2 * B_tilde[2]) * expq(w3*B_tilde[3])  //this wont work since we multiply tangent by dualQuat directly but have wrong tangent impl
//                                   +c0 * expq(B_tilde[1] * w1) * expq(w2 * B_tilde[2]) * (w2 * B_tilde_dot[2]).toDH() * expq(w3*B_tilde[3])
//                                   +c0 * expq(B_tilde[1] * w1) * expq(w2 * B_tilde[2]) * expq(w3*B_tilde[3]) * (w3 * B_tilde_dot[3]).toDH();

//        //angular velocity (wx,wy,wz) is  2*q⁻¹*q'
//        DualQuaternion<T> whatever = T(2.0)*(q_inv*q_dash);

//        result.angularVel = whatever.m_real.vec();


        //position 2nd deriv
        //Eigen::Matrix<T,4,1> B_tilde_dotdot  = b_spline::spline_B_dotdot<T>(u,((double)deltaT)*1e-9);
        //result.linearAcc = q_inv*(a1*B_tilde_dotdot[1] + a2*B_tilde_dotdot[2] + a3*B_tilde_dotdot[3]);

        //put in local frame with q_inv   as in  Poses3d Trajectory::kreisBewegung(double incr, int n, double r){



        result.fromDualQuat(resultq);

        return result;
    }

    template<typename T>
    static Pose3<T> evaluateApprox(double u,const DH1<T>& c0,const DH1<T>& c1,const DH1<T>& c2,const DH1<T>& c3, uint64_t time, uint64_t deltaT){

        Pose3<T> result;

        result.time = time;

        Eigen::Matrix<T,4,1> B = b_spline::spline_B_basis<T>(u);

        DH1<T> resultq =
                (c0*B[0]) +
                (c1*B[1]) +
                (c2*B[2]) +
                (c3*B[3]);

        result.fromDualQuat(resultq.normalized());
        return result;
    }


//    template<typename T>
//    static Pose3<T> evaluate(double u,const T* const c0,const T* const c1,const T* const c2,const T* const c3, uint64_t time, uint64_t deltaT){
//        return evaluate(u,toSophusSE3(c0),toSophusSE3(c1),toSophusSE3(c2),toSophusSE3(c3),time, deltaT);
//    }


    template<typename T>
    static Pose3<T> evaluate(double u,Pose3<T>& c0,Pose3<T>& c1, Pose3<T>& c2,Pose3<T>& c3, uint64_t time, uint64_t deltaT, bool approx){
        return approx ? evaluateApprox(u,c0.toDualQuat(),c1.toDualQuat(),c2.toDualQuat(),c3.toDualQuat(),time, deltaT) :
                        evaluate(u,c0.toDualQuat(),c1.toDualQuat(),c2.toDualQuat(),c3.toDualQuat(),time, deltaT);
    }
};

} // ns interpol

#endif // INTERPOL_TRAJECTORYSPLINEDUALQUATERNION_H
