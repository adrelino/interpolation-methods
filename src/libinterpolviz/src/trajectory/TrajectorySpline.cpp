/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectorySpline.h"

namespace interpol {

TrajectorySpline::TrajectorySpline(Poses3d& poses, bool posesAreWithPhantomPosesFirstAndLast)
    : Trajectory(poses, posesAreWithPhantomPosesFirstAndLast) {}

void TrajectorySpline::getBasePoseIndexAndSegmentScalar(uint64_t time, int &i, double& u) const {
    bool found = false;
    int n = basePoses.size();
    found = time >= basePoses[1].time && time <= basePoses[n-2].time;
    if(!found){
        std::cout<<"can't find "<<time<<" between "<<basePoses[1].time <<" and " <<basePoses[n-2].time<<std::endl;
    }
    assertCustom(found);

    uint64_t timeRelative = time-basePoses[0].time;

    i = timeRelative/deltaT;
    u = fmod(timeRelative,deltaT)/deltaT;

    if(time==timeOfLastSupportPose){//basePoses[basePoses.size()-2].time){
        u=1;
        i-=1;
    }

    assertCustom(i >= 1 && i <= n - 2);
    assertCustom(i+2 < basePoses.size());
}


} // ns interpol
