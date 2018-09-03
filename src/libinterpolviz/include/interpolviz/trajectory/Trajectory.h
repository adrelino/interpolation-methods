/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TRAJECTORY_H
#define INTERPOL_TRAJECTORY_H

#include "Pose3.h"
#include <vector>
#include <string>

namespace interpol {

/**
 * Abstract class that defines trajectory interface
 */
class Trajectory{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Trajectory(Poses3d& posess, bool posesAreWithPhantomPosesFirstAndLast=false);

    bool isUniform() const{
        return deltaT != UINT64_MAX;
    }

    virtual Poses3d getBasePoses(){
        return basePoses;
    }

    uint64_t timeOffset = 0;

    uint64_t deltaT = UINT64_MAX;

    virtual Pose3d getPoseAt(uint64_t time) const = 0;
    virtual void getBasePoseIndexAndSegmentScalar(uint64_t time, int& idx, double& u) const = 0;

    //Poses3d poses; //incoming poses
    Poses3d& basePoses; //might be padded with phantom points
    Poses3d regularSampledPoses; //times between poses[0] and poses[n-1]
    uint64_t incrLast; //regularSampledPoses are spaced incrLast apart;

    /*
     * need to be called after control poses changed (e.g. after ceres optimized them)
     * in order to clear member variable caches used in visualisation
     */
    virtual void controlPosesUpdated(bool sampleAgain=true);

    virtual Poses3d getControlPolygon(){
        return basePoses;
    }

    /**
     * @brief sampleRegular
     * @param incr number of nanoseconds (e.g. 0.2*1e9) for which to generate poses from trajectory
     * @return
     */
    Poses3d sampleRegular();
    Poses3d sampleRegular(uint64_t incrNanoSec);

    Poses3d* groundTruth;


    std::vector<double> posComp(int idx) const;
    std::vector<double> linearVel() const;
    std::vector<double> linearAcc() const;
    std::vector<double> angularVelocity() const;
    std::vector<double> angularAcceleration() const;

    std::vector<double> timeInSeconds() const; //xaxis
    std::vector<double> getDeriv(int test) const; //yaxis

    static void initDisplay();
    void draw(int colorChoice=-1, bool drawBasePoses=true);
    void draw2d(int colorChoice=-1);

    uint64_t getStartTime(){return timeOfFirstSupportPose;}
    uint64_t getEndTime(){return timeOfLastSupportPose;}
    uint64_t getDuration(){return duration;}

    void setEndTime(uint64_t endTime);

    //for visualization and saving
    std::string name;
    int key;
    bool show=true;
    int color;
    int modifiers;

    void toggle(int key_, int modifiers, bool show_, std::string name_, int color);

protected:
    uint64_t timeOfFirstSupportPose;
    uint64_t timeOfLastSupportPose;
    uint64_t duration;

    bool posesAreWithPhantomPosesFirstAndLast;
};
} //end ns

#endif // INTERPOL_TRAJECTORY_H
