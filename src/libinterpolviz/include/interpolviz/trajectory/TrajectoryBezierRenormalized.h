/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORYBEZIERRENORMALIZED_H
#define INTERPOL_TRAJECTORYBEZIERRENORMALIZED_H

#include "TrajectorySpline.h"
#include "interpol/euclidean/bezier.hpp"

namespace interpol {

/**
 * @brief uses 4d spline on quaternion coeffs and renormalizes them.
 */
class TrajectoryBezierRenormalized : public Trajectory {

public:

    TrajectoryBezierRenormalized(Poses3d& poses, bool renormalize=true, bool firstDerivBoundary=false);

    //overwritten
    Pose3d getPoseAt(uint64_t time) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //owerwritten
    void controlPosesUpdated(bool sampleAgain=false);
    Poses3d getControlPolygon();

    bool firstDerivBound;


    void getBasePoseIndexAndSegmentScalar(uint64_t time, int& idx, double& u) const{};

private:
    bezier::Spline3d tspline;
    bezier::Spline4d q2spline;
    bool renormalize;
};

}  // ns interpol

#endif // INTERPOL_TRAJECTORYBEZIERRENORMALIZED_H
