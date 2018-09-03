/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectorySplineDualQuaternion.h"

namespace interpol {

TrajectorySplineDualQuaternion::TrajectorySplineDualQuaternion(Poses3d& posess, bool posesAreWithPhantomPoses, bool approx) : TrajectorySpline(posess, posesAreWithPhantomPoses), approx(approx)
{
    name = approx ? "DLFus" : "ScFus";
}

Pose3d TrajectorySplineDualQuaternion::getPoseAt(uint64_t time) const{
    double u;
    int i;
    getBasePoseIndexAndSegmentScalar(time,i,u);
    return evaluate(u,basePoses[i-1],basePoses[i],basePoses[i+1],basePoses[i+2],time,deltaT,approx);
}

} // ns interpol
