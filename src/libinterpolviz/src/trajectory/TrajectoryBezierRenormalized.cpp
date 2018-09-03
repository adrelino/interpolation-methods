/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectoryBezierRenormalized.h"
#include <iostream>

namespace interpol {

TrajectoryBezierRenormalized::TrajectoryBezierRenormalized(Poses3d& posess, bool renormalize, bool fr)
    : Trajectory(posess), renormalize(renormalize), firstDerivBound(fr)
{
    name = "RQBez";
    controlPosesUpdated();
}

void TrajectoryBezierRenormalized::controlPosesUpdated(bool sampleAgain){
    //cout<<"TrajectorySplineRenormalized controlPosesUpdated "<<endl;
    Trajectory::controlPosesUpdated(false);

    vector_a<Eigen::Vector3d> ts;
    vector_a<Eigen::Vector4d> qs2;
    std::vector<double> times;
    for (Pose3d& supportPoint : basePoses) {
        times.push_back(supportPoint.time*1e-9);
        ts.push_back(supportPoint.position);
        qs2.push_back(supportPoint.orientation.coeffs());
    }

    Eigen::Vector3d left(0,12,-12);
    Eigen::Vector3d right(0,-12,-12);

    if(firstDerivBound){
        left = Eigen::Vector3d(0,0,6);
        right = Eigen::Vector3d(0,0,-6);
    }



    tspline = bezier::Spline3d(ts,times,firstDerivBound,left,right);
    q2spline = bezier::Spline4d(qs2,times);

    Trajectory::controlPosesUpdated();
}

Poses3d TrajectoryBezierRenormalized::getControlPolygon() {
    auto foo = tspline.getBasisPoints();
    auto foo2 = q2spline.getBasisPoints();

    Poses3d controlPolygon;

    assertCustom(foo.size() == foo2.size());
    for (int i = 0; i < foo.size(); ++i) {
        auto orientation = Eigen::Quaterniond(foo2[i]);
        if(renormalize) orientation.normalize();
        controlPolygon.push_back(Pose3d(i,foo[i],orientation));
    }

    return controlPolygon;
}

Pose3d TrajectoryBezierRenormalized::getPoseAt(uint64_t time) const{
    double tGlobal = time*1e-9;// (time - timeOfFirstSupportPose)*1.0/(duration);
    Pose3d interp;
    interp.orientation = (Eigen::Quaterniond(q2spline.eval(tGlobal)));
    //if(renormalize)
    interp.orientation.normalize();
    interp.position = tspline.eval(tGlobal);
    return interp;
}

}  // ns interpol
