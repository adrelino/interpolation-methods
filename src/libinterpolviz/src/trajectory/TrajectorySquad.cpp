#include "TrajectorySquad.h"
/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include <iostream>

namespace interpol {

TrajectorySquad::TrajectorySquad(Poses3d& posess, bool initialVelocityNotZero) : Trajectory(posess)
{
    name = "SQUAD";
    controlPosesUpdated();
}


void TrajectorySquad::controlPosesUpdated(bool sampleAgain){
    Trajectory::controlPosesUpdated(false);

    //cout<<"TrajectorySplineSquad controlPosesUpdated "<<endl;
    int n = basePoses.size();
    ts.resize(n);
    qs.resize(n);
    for (int i = 0; i < n; ++i) {
        ts[i] = basePoses[i].position;
        qs[i] = basePoses[i].orientation;
    }

    tspline = squad::VSpline(ts);
    qspline = squad::QSpline(qs);

/*    cout<<"Squad make BasePoses"<<endl;
    int n = qspline->segments.size(); //==poses.size() -1
    for (int i = 0; i < n; ++i) {
        uint64_t startTime = basePoses[i].time;
        uint64_t endTime = basePoses[i+1].time;

        if(i==0) basePoses.push_back(Pose3d(startTime, Vector3d(0, 0, 0), qspline->segments[i].q1));
        basePoses.push_back(Pose3d(startTime+deltaT/3.0, Vector3d(0, 0, 0), qspline->segments[i].t1));
        basePoses.push_back(Pose3d(startTime+2*deltaT/3.0, Vector3d(0, 0, 0), qspline->segments[i].t2));
        basePoses.push_back(Pose3d(endTime, Vector3d(0, 0, 0), qspline->segments[i].q2));
    }*/

    Trajectory::controlPosesUpdated();
}

Poses3d TrajectorySquad::getControlPolygon(){
    auto foo = tspline.getControlPolygon();
    auto foo2 = qspline.getControlPolygon();

    Poses3d controlPolygon;

    assertCustom(foo.size() == foo2.size());
    for (int i = 0; i < foo.size(); ++i) {
        controlPolygon.push_back(Pose3d(i,foo[i],foo2[i]));
    }

    return controlPolygon;
}


Pose3d TrajectorySquad::getPoseAt(uint64_t time) const{
    double tGlobal = (time - timeOfFirstSupportPose)*1.0/(duration);
    Pose3d interp;
    interp.orientation = qspline.eval(tGlobal);
    interp.position = tspline.eval(tGlobal);
    return interp;
}

Poses3d TrajectorySquad::getBasePoses() {
    return basePoses;
}

}  // ns interpol
