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
#include <interpolviz/trajectories.h>
#include <interpolviz/Visualize.h>
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

    //higher order
    auto squad = make_shared<TrajectorySquad>(poses);
    auto kimKimShin = make_shared<TrajectorySplineKimKimShin>(posesWithPhantoms, true);
    auto fusion = make_shared<TrajectorySplineFusion>(posesWithPhantoms, true);
    auto dualQuat = make_shared<TrajectorySplineDualQuaternion>(posesWithPhantoms, true, false);
    auto dualQuatApprox = make_shared<TrajectorySplineDualQuaternion>(posesWithPhantoms, true, true);
    trajectories.push_back(squad);
    trajectories.push_back(kimKimShin);
    trajectories.push_back(fusion);
    trajectories.push_back(dualQuat);
    trajectories.push_back(dualQuatApprox);
    squad->toggle(GLFW_KEY_I,modifer, true, "SQUAD", BLUE);
    kimKimShin->toggle(GLFW_KEY_K,modifer, true, "CuBsp", GREEN);
    fusion->toggle(GLFW_KEY_F,modifer, true, "SpFus", CYAN);
    dualQuat->toggle(GLFW_KEY_J,modifer, true, "ScFus", RED);
    dualQuatApprox->toggle(GLFW_KEY_M,modifer, true, "DLFus",MAG);

    TrajectoryRuntimeExperiments(trajectories,poses[0]);



    Visualize *V = Visualize::getInstance();
    TrajectoryVisualization(trajectories,V,poses,posesWithPhantoms);

    //V->setToggleState(GLFW_KEY_R,true);
    //V->setPressState(GLFW_KEY_5);
    //V->setToggleState(GLFW_KEY_F5,GLFW_MOD_SHIFT,true);
    V->setToggleState(GLFW_KEY_B,3);
    //V->setToggleState(GLFW_KEY_B,GLFW_MOD_SHIFT,3);
    V->setToggleState(GLFW_KEY_C,false);//control polygon
    V->setToggleState(GLFW_KEY_F11,true);//lightning
    V->setToggleState(GLFW_KEY_F10,false);//blending
    V->setToggleState(GLFW_KEY_F9,true);//black
    V->setToggleState(GLFW_KEY_U,GLFW_MOD_SHIFT,false);//QUATERNION UNIT SPHERe
    V->setToggleState(GLFW_KEY_F7,true);//translation curves
    V->setToggleState(GLFW_KEY_F8,false);//orientation curves
    V->setToggleState(GLFW_KEY_F5,GLFW_MOD_SHIFT,0);//5 regular sampled poses on trajectory
    V->setToggleState(GLFW_KEY_F8,GLFW_MOD_SHIFT,true);//pfeiltasten to move quat

    Visualize::spin();
}
