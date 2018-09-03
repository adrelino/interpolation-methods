/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

#include <Eigen/Geometry>
#include <vector>

namespace interpol {

static Eigen::Quaterniond angleAxis2Quaternion(double angle, double x, double y, double z) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d(x, y, z).normalized()));
}

static std::vector<Eigen::Quaterniond> getQuaternions() {
    std::vector<Eigen::Quaterniond> qs;

    qs.push_back(angleAxis2Quaternion(1.0, 1, 3, 0));
    qs.push_back(angleAxis2Quaternion(1.9, -1, 0, 0));
    qs.push_back(angleAxis2Quaternion(0.0, -2, 1, 0));
    qs.push_back(angleAxis2Quaternion(-2.0, 3, 4, 0));
    qs.push_back(angleAxis2Quaternion(-1.0, -1, 4, 0));
    qs.push_back(angleAxis2Quaternion(1.0, 2, 3, 0)); //note: different than in paper, which does not correspond to the visualization
    // (in the  paper it is 1.0, 1, 3 which would be same as 1st quaternion)

    return qs;
}

static Poses3d getPoses(){
    auto orientations = getQuaternions();
    Poses3d poses1;
    int n = orientations.size();
    for (int i = 0; i < orientations.size(); ++i) {
        uint64_t time = 10000000000 + i * 1 * 1e9;
        Eigen::Vector3d pos = Vector3d(0, 0, -1.0 + (2.0 * i) / (n-1)); //: orientations[i].vec();//+Vector3d(0.5,0.5,0.5);
        auto p = Pose3d(time, pos, orientations[i]);
        p.fixed=true;
        poses1.push_back(p);
    }

    poses1[1].position.y()=1;
    poses1[3].position.y()=1;

    return poses1;
}

static void flipQuaternionsToSameHemisphere(Poses3d& quats){
    for(int i=0; i<quats.size()-1;i++){
        Eigen::Quaterniond currentOtherSide(quats[i + 1].orientation);
        currentOtherSide.coeffs() *= -1;
        double dist1 = quats[i].orientation.dot(quats[i + 1].orientation);
        double dist2 = quats[i].orientation.dot(currentOtherSide);
        //cout<<"dist1="<<dist1<<"  dist2="<<dist2<<endl;
        if (dist1 < dist2) {
            quats[i + 1].orientation = currentOtherSide;
        }
    }
}

} // ns interpol
