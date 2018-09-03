/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYLINEAR_H
#define INTERPOL_TRAJECTORYLINEAR_H

#include "Trajectory.h"
#include "interpol/rigid.hpp"

namespace interpol {

/**
 * @brief The ContiniousTrajectory class
 *
 * Gets passed in a number of high frequency (100Hz) poses and constructs a continious trajectory.
 * Each pose query at timt t is interpolated linearly between the 2 closest poses (t: lerp, q: slerp)
 */
class TrajectoryLinear : public Trajectory{
public:
    //lerpMode:
    // 0 SLERP_G + LERP as in SPLIT
    // 1 SE3Up
    // 2 DualQuat ScLERPAnalyticExpLog
    // 3 DualQuat ScLERPGeometricCosSin
    // 4 DLUP Approx
    // 5 QLP Approx SPLIT
    // 6 SLERP_A + LERP as in SPLIT
    TrajectoryLinear(Poses3d& poses, int lerpMode = 0);

    Pose3d getPoseAt(uint64_t time) const;
    void getBasePoseIndexAndSegmentScalar(uint64_t time, int& idx, double& u) const;


    Poses3d getBasePoses();

    int cachedIdx = 0;

    //template<typename Scalar>
    static Pose3d evaluate(double u,const Pose3d& left,const Pose3d& right, uint64_t time=50, int type=0);

    static Pose3d evaluateSE3LERP(double u,const Pose3d& left,const Pose3d& right, uint64_t time=50);

    static Pose3d evaluateDualQuatLERP(double u,const Pose3d& left,const Pose3d& right, uint64_t time=50, int type=2);

    static TrajectoryLinear init(Poses3d& posesToInterpolate, double taus, uint64_t timeOffset);
private:
    int lerpMode;
};

} // ns interpol

#endif // CONTINIOUSTRAJECTORY_H
