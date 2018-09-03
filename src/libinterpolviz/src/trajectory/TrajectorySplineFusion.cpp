/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectorySplineFusion.h"
#include <iostream>

namespace interpol {

TrajectorySplineFusion::TrajectorySplineFusion(Poses3d& posess, bool posesAreWithPhantomPoses)
    : TrajectorySpline(posess, posesAreWithPhantomPoses)
{
    name = "SpFus";
}

Pose3d TrajectorySplineFusion::getPoseAt(uint64_t time) const{
    double u;
    int i;
    getBasePoseIndexAndSegmentScalar(time,i,u);
    return evaluate(u,basePoses[i-1],basePoses[i],basePoses[i+1],basePoses[i+2],time,deltaT);
}

} // ns interpol
