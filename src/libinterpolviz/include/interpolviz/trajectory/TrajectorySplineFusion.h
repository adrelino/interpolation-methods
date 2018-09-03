/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYSPLINEFUSION_H
#define INTERPOL_TRAJECTORYSPLINEFUSION_H

#include "TrajectorySpline.h"
#include "interpol/rigid/SE3.hpp"
#include "interpol/euclidean/b_spline.hpp"
#include <Eigen/Dense>

using namespace Sophus;

namespace interpol {

    /**
     * Lovegrove et al BMCV 2013 style spline
     */
class TrajectorySplineFusion : public TrajectorySpline {

public:

    TrajectorySplineFusion(Poses3d& poses, bool posesAreWithPhantomPoses);

    //overwritten
    Pose3d getPoseAt(uint64_t time) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    //const double deltaTSeconds = 0.1;

    template<typename T>
    static Pose3<T> evaluate(double u,const SE3<T>& c0,const SE3<T>& c1,const SE3<T>& c2,const SE3<T>& c3, uint64_t time, uint64_t deltaT){
//        Eigen::Matrix<T,4,1> B = se3_spline::spline_B<T>(u);
//
//        return c0 * SE3Group<T>::exp(B[1] * SE3Group<T>::log(c0.inverse()*c1)) \
//                  * SE3Group<T>::exp(B[2] * SE3Group<T>::log(c1.inverse()*c2)) \
//                  * SE3Group<T>::exp(B[3] * SE3Group<T>::log(c2.inverse()*c3));

        Eigen::Matrix<T,4,1> B          = b_spline::spline_B<T>(u);
        //Eigen::Matrix<T,4,1> B_dot      = b_spline::spline_B_dot<T>(u,((double)deltaT)*1e-9);
        //Eigen::Matrix<T,4,1> B_dotdot   = b_spline::spline_B_dotdot<T>(u,((double)deltaT)*1e-9);

        //http://stackoverflow.com/questions/1164266/why-arrays-of-references-are-illegal
        //const SE3Group<T>& c[4] = {c0,c1,c2,c3};
        std::reference_wrapper<const SE3<T> > c[4] = {c0,c1,c2,c3};

        SE3<T> A[3];
        //Eigen::Matrix<T, 4, 4> Am[3];
        //Eigen::Matrix<T, 4, 4> Af[3];
        //Eigen::Matrix<T, 4, 4> As[3];



        for(int j : {1, 2, 3}) {
            auto omega = (c[j-1].get().inverse()*c[j].get()).log();

            //0: position and orientation
            A[j-1] = SE3<T>::exp(omega * B[j]);
            //Am[j-1] = A[j-1].matrix();

            //1: first derivative
//            Eigen::Matrix<T, 4, 4> omega_hat = SE3Group<T>::hat(omega);
//            Af[j-1] = Am[j-1] * omega_hat * B_dot[j];
//            As[j-1] = Af[j-1] * omega_hat * B_dot[j] + Am[j-1] * omega_hat * B_dotdot[j];
        }

//        Eigen::Matrix<T, 4, 4> Mf =   Af[0] * Am[1] * Am[2] +
//                                      Am[0] * Af[1] * Am[2] +
//                                      Am[0] * Am[1] * Af[2];

//        Eigen::Matrix<T, 4, 4> Ms =   As[0] * Am[1] * Am[2] +          Am[0] * As[1] * Am[2] +
//                                      Am[0] * Am[1] * As[2] + T(2.0) * Af[0] * Af[1] * Am[2] +
//                             T(2.0) * Af[0] * Am[1] * Af[2] + T(2.0) * Am[0] * Af[1] * Af[2];


        SE3<T> Tz = c0 * A[0] * A[1] * A[2];

//        Eigen::Matrix<T, 4, 4> Tzm = c0.matrix() * Am[0] * Am[1] * Am[2];
//        Eigen::Matrix<T, 4, 4> Tf = c0.matrix() * Mf;
//        Eigen::Matrix<T, 4, 4> Ts = c0.matrix() * Ms;

        Pose3<T> pose(time,Tz.translation(),Tz.unit_quaternion());

//        Eigen::Matrix<T,3,3> Rw;
//        Rw <<   Tf(0,0), Tf(0,1), Tf(0,2),
//                Tf(1,0), Tf(1,1), Tf(1,2),
//                Tf(2,0), Tf(2,1), Tf(2,2);

//        Eigen::Matrix<T,3,3> Rzm;
//        Rzm <<   Tzm(0,0), Tzm(0,1), Tzm(0,2),
//                Tzm(1,0), Tzm(1,1), Tzm(1,2),
//                Tzm(2,0), Tzm(2,1), Tzm(2,2);

//        //Rw = pose.orientation.inverse().matrix() * Rw;

//        //def (7) in splinefusion
//        Eigen::Matrix<T,3,3> inSensorFrame = Rw.transpose() * Rzm;

//       // pose.angularVel << -inSensorFrame(1,3), inSensorFrame(0,3), -inSensorFrame(0,2);



//               // Eigen::Quaternion<T> timesQuat(inSensorFrame);
//        //auto timesQuat = pose.orientation.inverse()*derivQuat;
//        //pose.angularVel << 2*timesQuat.x(), 2*timesQuat.y(),2*timesQuat.z();


//        //Eigen::Matrix<T,3,3> rf(Tf.topLeftCorner<3,3>());

//       Eigen::AngleAxis<T> angeAxis(inSensorFrame);
//       pose.angularVel = angeAxis.axis().array() * angeAxis.angle();

        return pose;
    }


    template<typename T>
    static Pose3<T> evaluate(double u,const T* const c0,const T* const c1,const T* const c2,const T* const c3, uint64_t time, uint64_t deltaT){
        return evaluate(u,toSophusSE3(c0),toSophusSE3(c1),toSophusSE3(c2),toSophusSE3(c3),time, deltaT);
    }


    template<typename T>
    static Pose3<T> evaluate(double u,Pose3<T>& c0,Pose3<T>& c1, Pose3<T>& c2,Pose3<T>& c3, uint64_t time, uint64_t deltaT){
        return evaluate(u,c0.toSophusSE3(),c1.toSophusSE3(),c2.toSophusSE3(),c3.toSophusSE3(),time, deltaT);
    }

};

}  // ns interpol

#endif // INTERPOL_TRAJECTORYSPLINEFUSION_H
