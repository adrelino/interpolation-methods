/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYSPLINE_H
#define INTERPOL_TRAJECTORYSPLINE_H

#include "Trajectory.h"

namespace interpol {

/**
 * @brief The TrajectorySpline class
 *
 * Gets passed in a high number of poses and constructs a cubic
 * spline to interpolate between small number of support poses
 */
class TrajectorySpline : public Trajectory {

public:
    /**
     * @brief ContiniousTrajectorySpline
     * @param poses high frequency poses
     */
    TrajectorySpline(Poses3d& posess, bool posesAreWithPhantomPosesFirstAndLast=false);

    virtual Pose3d getPoseAt(uint64_t time) const = 0;

    void getBasePoseIndexAndSegmentScalar(uint64_t time, int& idx, double& u) const;

    static void fixEndsAndAddNoiseOthers(Poses3d &poses);

};

} // ns interpol

#endif // INTERPOL_TRAJECTORYSPLINE_H
