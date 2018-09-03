/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "TrajectoryLinear.h"
#include <algorithm>

namespace interpol {

TrajectoryLinear::TrajectoryLinear(Poses3d& poses, int lerpMode)
    : Trajectory(poses), lerpMode(lerpMode){}

void TrajectoryLinear::getBasePoseIndexAndSegmentScalar(uint64_t time, int &idx, double &u) const{
    if(time==basePoses[basePoses.size()-1].time){
        u=1;
        idx=basePoses.size()-2; //2nd last pose, because we call evaluate with basePoses[idx], basePoses[idx+1]
    }else if(time==timeOfFirstSupportPose){
        u=0;
        idx=0;
    }else if(isUniform()){
        int n = basePoses.size();

        if(timeOfFirstSupportPose <= time && time <= timeOfLastSupportPose){

        }else{
            std::cout<<"not ok timeOfFirstSupportPose:"<<timeOfFirstSupportPose<<" time: "<<time<<" timeOfLastSupportPose: "<<timeOfLastSupportPose<<std::endl;
        }
        //assertCustom(timeOfFirstSupportPose <= time && time <= timeOfLastSupportPose);

        uint64_t timeRelative = time-timeOfFirstSupportPose;

        idx = (timeRelative/deltaT);
        u = fmod(timeRelative,deltaT)/deltaT;
    }else{
        idx = linearInterpolateTimestampToClosest2TrajectorySamples(basePoses,time,u);
    }

    if(idx>=0 && idx < basePoses.size()-1){

    }else{
        std::cout<<"not ok time:"<<time<<" idx: "<<idx<<" u: "<<u<<std::endl;
    }

    //assertCustom(idx>=0 && idx< basePoses.size()-1);
}

Pose3d TrajectoryLinear::evaluateSE3LERP(double u,const Pose3d& left,const Pose3d& right, uint64_t time){
    Sophus::SE3d co = left.toSophusSE3();
    Sophus::SE3d c1 = right.toSophusSE3();

    Sophus::SE3d c = SE3Up(co,u,c1);

    Pose3d res;
    res.time=time;
    res.fromSophusSE3(c);

    return res;
}

Pose3d TrajectoryLinear::evaluate(double u,const Pose3d& left,const Pose3d& right, uint64_t time, int mode){
    Pose3d p;
    if(mode==0){
        p.orientation = SLERP(left.orientation,u, right.orientation);
    }else if(mode==5){
        p.orientation = QLB(left.orientation,u,right.orientation);
    }
    p.position = left.position*(1-u) + right.position*u;
    return p;
}

Pose3d TrajectoryLinear::evaluateDualQuatLERP(double u,const Pose3d& left,const Pose3d& right, uint64_t time, int type){
    DH1d c0(left.orientation,left.position);
    DH1d c1(right.orientation,right.position);
    DH1d c;
    if(type==2){
        //c = ScLERPGeometricCosSin(u,c0,c1);//should be equal
        c = ScLERPAnalyticExpLogScrewTangent(c0,u,c1);
    }else if(type==3){
        c = ScLERPGeometricCosSin(c0,u,c1);//should be equal
        //c = ScLERPAnalyticExpLogWrongDualPart(u,c0,c1);
    }else if(type==4){
        c = DLUp(c0,u,c1);
    }
    //cout<<"w="<<c.translationQuaternion().w()<<endl;
    return Pose3d(time,c.translation(),c.rotation());
}

Pose3d TrajectoryLinear::getPoseAt(uint64_t time) const{
    int i;
    double u;
    getBasePoseIndexAndSegmentScalar(time,i,u);
    if(lerpMode==1) return evaluateSE3LERP(u,basePoses[i],basePoses[i+1],time);
    else if(lerpMode >= 2 && lerpMode <=4) return evaluateDualQuatLERP(u,basePoses[i],basePoses[i+1],time,lerpMode);
    else return evaluate(u,basePoses[i],basePoses[i+1],time, lerpMode);//0, 5 or 6
}

Poses3d TrajectoryLinear::getBasePoses() {
    return basePoses;
}

TrajectoryLinear TrajectoryLinear::init(Poses3d &posesToInterpolate, double taus, uint64_t timeOffset) {
    TrajectoryLinear T_highFreq(posesToInterpolate);
    Poses3d poses_subsampled = T_highFreq.sampleRegular(taus*1e9);
    poses_subsampled[0].fixed=true;
    auto T= TrajectoryLinear(poses_subsampled);
    T.timeOffset = timeOffset;
    return T;
}

} // ns interpol
