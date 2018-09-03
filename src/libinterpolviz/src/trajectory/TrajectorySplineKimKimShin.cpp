/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectorySplineKimKimShin.h"
#include "TrajectoryLinear.h"
#include <iostream>

namespace interpol {

TrajectorySplineKimKimShin::TrajectorySplineKimKimShin(Poses3d& posess, bool posesAreWithPhantomPosesFirstAndLast)
    : TrajectorySpline(posess,posesAreWithPhantomPosesFirstAndLast)
{
    name="CuBsp";
}

Pose3d TrajectorySplineKimKimShin::getPoseAt(uint64_t time) const{
    int i;
    double u;
    getBasePoseIndexAndSegmentScalar(time,i,u);
    return evaluate(u,basePoses[i-1],basePoses[i],basePoses[i+1],basePoses[i+2],time,deltaT);
    //cout<<p<<" i: "<<i<<" u: "<<u<<" deltaT: "<<deltaT<<endl;
}

Poses3d TrajectorySplineKimKimShin::getUniformKnots(Poses3d& posesOriginal, double taus, uint64_t timeOffset){
    uint64_t spacing = taus*1e9;
    TrajectoryLinear linear(posesOriginal);
    Poses3d uniformKnots = linear.sampleRegular(spacing); //still missing phantom knots
    return uniformKnots;
}

} // ns interpol
