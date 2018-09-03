/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYSPLINEKIMKIMSHIN_H
#define INTERPOL_TRAJECTORYSPLINEKIMKIMSHIN_H

#include "TrajectorySpline.h"
#include "interpol/rigid/R3xSO3.hpp"
#include "interpol/euclidean/b_spline.hpp"

namespace interpol {

/**
 * Kim, Kim, Shin 95' style spline with uniform knot spacing
 */
class TrajectorySplineKimKimShin : public TrajectorySpline {

public:
    //poses must be equally spaced, times are used for basePoses
    TrajectorySplineKimKimShin(Poses3d& poses, bool posesAreWithPhantomPosesFirstAndLast = false);

    //factory method to ease construction
    static Poses3d getUniformKnots(Poses3d& posesToInterpolate, double taus, uint64_t timeOffset);

    //overwritten
    Pose3d getPoseAt(uint64_t time) const;

    template<typename T>
    static Pose3<T> evaluate(double u,const Pose3<T>& c0,const Pose3<T>& c1,const Pose3<T>& c2,const Pose3<T>& c3, uint64_t time, uint64_t deltaT){
        Pose3<T> result;

        result.time = time;
        //cout<<time<<" u:"<<u<<endl;

        Eigen::Matrix<T,4,1> B_tilde = b_spline::spline_B<T>(u);
        //Eigen::Matrix<T,4,1> B_tilde_dot  = b_spline::spline_B_dot<T>(u,((double)deltaT)*1e-9);


        const Eigen::Quaternion<T>& q0(c0.orientation);
        const Eigen::Quaternion<T>& q1(c1.orientation);
        const Eigen::Quaternion<T>& q2(c2.orientation);
        const Eigen::Quaternion<T>& q3(c3.orientation);

        //angular velocities given in local frame //kimkimshin 4.2
        Eigen::Quaternion<T> w1 = logq(q0.conjugate() * q1);
        Eigen::Quaternion<T> w2 = logq(q1.conjugate() * q2);
        Eigen::Quaternion<T> w3 = logq(q2.conjugate() * q3);

        //angular velocities given in global frame //kimkimshin 4.2
      /*Eigen::Quaternion<T> w1 = logq(q1*q0.conjugate());
        Eigen::Quaternion<T> w2 = logq(q2*q1.conjugate());
        Eigen::Quaternion<T> w3 = logq(q3*q2.conjugate());*/

        //rotation q
        result.orientation = q0 * expq(B_tilde[1] * w1) \
                                * expq(B_tilde[2] * w2) \
                                * expq(B_tilde[3] * w3);

        //inertial_to_local_frame
//        Quaternion<T> q_inv = result.orientation.conjugate();

//        //rotation 1st deriv (kimkimShin p. 373) in inertial frame
//        Quaternion<T> q_dash =   q0 * expq(B_tilde[1] * w1) * (w1 * B_tilde_dot[1]) * expq(w2 * B_tilde[2]) * expq(w3*B_tilde[3])
//                                   +q0 * expq(B_tilde[1] * w1) * expq(w2 * B_tilde[2]) * (w2 * B_tilde_dot[2]) * expq(w3*B_tilde[3])
//                                   +q0 * expq(B_tilde[1] * w1) * expq(w2 * B_tilde[2]) * expq(w3*B_tilde[3]) * (w3 * B_tilde_dot[3]);

        //angular velocity (wx,wy,wz) is  2*q⁻¹*q'
        //result.angularVel = T(2.0)*(q_inv*q_dash).vec();


        Eigen::Matrix<T,3,1> a1 = c1.position - c0.position;
        Eigen::Matrix<T,3,1> a2 = c2.position - c1.position;
        Eigen::Matrix<T,3,1> a3 = c3.position - c2.position;

        //position
        result.position = c0.position*B_tilde[0] + a1*B_tilde[1]
                                                 + a2*B_tilde[2]
                                                 + a3*B_tilde[3];

//        //position 1st deriv
//        result.linearVel =  q_inv*(a1*B_tilde_dot[1] + a2*B_tilde_dot[2] + a3*B_tilde_dot[3]);

//        //position 2nd deriv
//        Eigen::Matrix<T,4,1> B_tilde_dotdot  = b_spline::spline_B_dotdot<T>(u,((double)deltaT)*1e-9);
//        result.linearAcc = q_inv*(a1*B_tilde_dotdot[1] + a2*B_tilde_dotdot[2] + a3*B_tilde_dotdot[3]);

        //put in local frame with q_inv   as in  Poses3d Trajectory::kreisBewegung(double incr, int n, double r){


        //////////////////
        // non cumulative following

        // Eigen::Matrix<T,1,4> B = b_spline::spline_B_basis<T>(u);
                            //u * v
      //   result.position = B[0] * c0.position + B[1] * c1.position + B[2] * c2.position + B[3] * c3.position;

/*        result.linearVel = c0.position*B_tilde[0]  *a1*B_tilde[1] * a1*B_tilde_dot[1] * a2*B_tilde[2] * a3*B_tilde[3]
                          +c0.position*B_tilde[0] * a1*B_tilde[1] * a2*B_tilde[2] * a2*B_tilde_dot[2] * a3*B_tilde[3]
                          +c0.position*B_tilde[0] * a1*B_tilde[1] * a2*B_tilde[2] * a3*B_tilde[3] * a3*B_tilde_dot[3];*/
/*        //vel
        Eigen::Matrix<T,1,4> B_dot = se3_spline::spline_B_basis_dot<T>(u);

        result.linearVel = B_dot[1] * c1.position + B_dot[2] * c2.position + B_dot[3] * c3.position//u'*v
                         + B[0] * c0.position + B[1] * a1 + B[2] * a2 + B[3] * a3; //u*v'
        //acc
        Eigen::Matrix<T,1,4> B_dotdot = se3_spline::spline_B_basis_dotdot<T>(u);
        result.linearAcc = B_dotdot[2] * (a1-a2) + B_dotdot[3] * (a2-a3);*/



        return result;
    }
};

}  // ns interpol

#endif // INTERPOL_TRAJECTORYSPLINEKIMKIMSHIN_H
