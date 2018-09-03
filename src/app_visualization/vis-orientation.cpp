/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include <interpol/orientation.hpp>
#include <interpol/utils/timer.hpp>
#include <interpolviz/Visualize.h>
#include <interpolviz/trajectories.h>
#include <memory>
#include "sampleposes.h"
#include "common.hpp"

using namespace std;
using namespace interpol;

int main(int argc, char * argv[]){

    Poses3d poses = getPoses();
    Poses3d posesWithPhantoms = Pose3d::addPhantomPoses(poses);

    vector<std::shared_ptr<Trajectory>> trajectories;

    int modifer=0;

    //pairwise
    auto pSLERP = make_shared<TrajectoryLinear>(poses);
    auto pQLERP = make_shared<TrajectoryLinear>(poses,5);
    auto slerpSE3 = make_shared<TrajectoryLinear>(poses, 1);
    auto slerpDualQuat = make_shared<TrajectoryLinear>(poses, 2);
    //auto slerpDualQuatGeometric = make_shared<TrajectoryLinear>(poses, 3);
    auto slerpDualQuatDLup = make_shared<TrajectoryLinear>(poses, 4);
    trajectories.push_back(pSLERP);
    trajectories.push_back(pQLERP);
    trajectories.push_back(slerpDualQuatDLup);
    trajectories.push_back(slerpSE3);
    trajectories.push_back(slerpDualQuat);
    //trajectories.push_back(slerpDualQuatGeometric);
    pSLERP->toggle(GLFW_KEY_L,modifer, true, "SLERP", ORANGE);
    pQLERP->toggle(GLFW_KEY_M,modifer, true, "QLB", BLUE);
    slerpSE3->toggle(GLFW_KEY_F,modifer, true, "SE3Up", MAG);
    slerpDualQuat->toggle(GLFW_KEY_J,modifer, false, "ScLERP", GREEN);
    //slerpDualQuatGeometric->toggle(GLFW_KEY_K,modifer, false, "ScLERP_G", RED);
    slerpDualQuatDLup->toggle(GLFW_KEY_N,modifer, true, "DLUp", RED);

    //higher order
    auto squad = make_shared<TrajectorySquad>(poses);
    auto renormalized = make_shared<TrajectoryBezierRenormalized>(poses);
    //auto renormalizedFirstDerivBound = make_shared<TrajectoryBezierRenormalized>(poses,true,true);
    auto kimKimShin = make_shared<TrajectorySplineKimKimShin>(posesWithPhantoms, true);
    auto fusion = make_shared<TrajectorySplineFusion>(posesWithPhantoms, true);
    auto dualQuat = make_shared<TrajectorySplineDualQuaternion>(posesWithPhantoms, true, false);
    auto dualQuatApprox = make_shared<TrajectorySplineDualQuaternion>(posesWithPhantoms, true, true);
    trajectories.push_back(squad);
    trajectories.push_back(renormalized);
    //trajectories.push_back(renormalizedFirstDerivBound);
    trajectories.push_back(kimKimShin);
    trajectories.push_back(fusion);
    trajectories.push_back(dualQuat);
    trajectories.push_back(dualQuatApprox);
    squad->toggle(GLFW_KEY_I,modifer, true, "SQUAD", BLUE);
    renormalized->toggle(GLFW_KEY_N,modifer, true, "RQBez", RED);
    //renormalizedFirstDerivBound->toggle(GLFW_KEY_M,modifer, true, "RQBez2", ORANGE);
    kimKimShin->toggle(GLFW_KEY_K,modifer, true, "CuBsp", GREEN);
    fusion->toggle(GLFW_KEY_F,modifer, true, "SpFus", CYAN);
    dualQuat->toggle(GLFW_KEY_J,modifer, false, "ScFus", RED);
    dualQuatApprox->toggle(GLFW_KEY_M,modifer, false, "DLFus", BLUE);


    TrajectoryRuntimeExperiments(trajectories,poses[0]);


    Visualize *V = Visualize::getInstance();
    TrajectoryVisualization(trajectories,V,poses,posesWithPhantoms);

    V->setToggleState(GLFW_KEY_R,true);
    //V->setPressState(GLFW_KEY_5);
    //V->setToggleState(GLFW_KEY_F5,GLFW_MOD_SHIFT,true);
    V->setToggleState(GLFW_KEY_B,3);
    //V->setToggleState(GLFW_KEY_B,GLFW_MOD_SHIFT,3);
    V->setToggleState(GLFW_KEY_C,false);//control polygon
    V->setToggleState(GLFW_KEY_F11,true);//lightning
    V->setToggleState(GLFW_KEY_F10,false);//blending
    V->setToggleState(GLFW_KEY_F9,false);//black
    V->setToggleState(GLFW_KEY_U,GLFW_MOD_SHIFT,true);//QUATERNION UNIT SPHERe
    V->setToggleState(GLFW_KEY_F7,false);//translation curves
    V->setToggleState(GLFW_KEY_F8,true);//orientation curves
    V->setToggleState(GLFW_KEY_F5,GLFW_MOD_SHIFT,0);//5 regular sampled poses on trajectory
    V->setToggleState(GLFW_KEY_F8,GLFW_MOD_SHIFT,true);//pfeiltasten to move quat

    std::vector<double> cam = {0,0,-4.82,-450,-203,0,-1.8,3};
    V->setCam(cam);
    V->setToggleState(GLFW_KEY_W,true);

    Visualize::spin();
}
