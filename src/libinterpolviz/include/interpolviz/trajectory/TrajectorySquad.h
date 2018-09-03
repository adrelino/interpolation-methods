/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYSQUAD_H
#define INTERPOL_TRAJECTORYSQUAD_H

#include "TrajectorySpline.h"
#include "interpol/orientation/squad.hpp"

namespace interpol {

/**
 * Shoemake 87' style spline
 */
class TrajectorySquad : public Trajectory {

public:

    TrajectorySquad(Poses3d& poses, bool initialVelocityNotZero=false);

    //overwritten
    Pose3d getPoseAt(uint64_t time) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    //overwritten. return intermediate poses t1, t2 as well!
    void controlPosesUpdated(bool sampleAgain=false);
    Poses3d getBasePoses();

    Poses3d getControlPolygon();

    void getBasePoseIndexAndSegmentScalar(uint64_t time, int& idx, double& u) const{};

private:
    vector_a<Eigen::Quaterniond> qs;
    vector_a<Eigen::Vector3d> ts;
    squad::VSpline tspline;
    squad::QSpline qspline;

};

} // ns interpol

#endif // INTERPOL_TRAJECTORYSQUAD_H
