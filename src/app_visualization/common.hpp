/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace interpol {

static void TrajectoryRuntimeExperiments(std::vector<std::shared_ptr<Trajectory>>& trajectories, Pose3d first){
    auto start = trajectories[0]->getStartTime();
    auto end = trajectories[0]->getEndTime();
    auto timespan = end-start;

    int numSamples = 10000;


    auto incr = timespan / numSamples;
    //cout<<"incr "<<incr<<endl;

    Timer    t3;

    int numOpsPerMeas = 0;
    for(uint64_t j=start; j<=end; j+=incr){
        numOpsPerMeas++;
    }
    int numTrials = 10;

    Pose3d accum = first;

    for(auto t : trajectories){
        Pose3d a = first;
        for(int j=0; j<numTrials; j++){
        t3.tic();
        for(uint64_t j=start; j<=end; j+=incr){
            auto b = t->getPoseAt(j);
            a.position += b.position;
            a.orientation *= b.orientation;

        }
        t3.toc(t->name);
        }
        //auto a = t->getPoseAt(half);
        accum.position += a.position;
        accum.orientation *= a.orientation;
        //cout<<t->name<<"\t"<<a.position<<" ori:"<<a.orientation<<endl;
    }


    t3.printAllTimings("Trajectories",numOpsPerMeas,"CuBsp");
    std::cout<<accum.position.norm()+accum.orientation.norm()<<std::endl;
}


static void TrajectoryVisualization(std::vector<std::shared_ptr<Trajectory>>& trajectories, Visualize* V, Poses3d& poses2, Poses3d& posesWithPhantoms2){
    V->setTimespan(trajectories[0]->getStartTime(), trajectories[0]->getEndTime());

    int whichQuat=0;
    V->press(GLFW_KEY_COMMA,"choose keyfraem",[&](int mods){
        std::cout<<"pressed minus"<<std::endl;
        whichQuat++;
        if(whichQuat>=poses2.size()) whichQuat=0;
    });



    for (int j = GLFW_KEY_RIGHT; j <= GLFW_KEY_UP; ++j) {
        V->press(j,"move quaternion",[&,j](int mods){

            if(V->toggled(GLFW_KEY_F7,GLFW_MOD_SHIFT)){
                Vector3d wxy = ((j>GLFW_KEY_LEFT) ? Vector3d(0,1,0) : Vector3d(0,0,-1));
                wxy *= (j%2==0) ? -0.1 : 0.1;
                poses2[whichQuat].position += wxy;
                posesWithPhantoms2[whichQuat+1].position += wxy;
            }
            if(V->toggled(GLFW_KEY_F8,GLFW_MOD_SHIFT)){
                auto& q = poses2[whichQuat].orientation;
                Eigen::Vector3d wxy(q.w(),q.x(),q.y());
                wxy = Eigen::AngleAxisd((j%2==0) ? -0.1 : 0.1,(j<=GLFW_KEY_LEFT) ? Eigen::Vector3d(0,1,0) : Eigen::Vector3d(0,0,1))*wxy;
                q.w() = wxy[0];
                q.x() = wxy[1];
                q.y() = wxy[2];
                posesWithPhantoms2[whichQuat+1].orientation = q;
                std::cout<<"q: "<<q.coeffs().transpose()<<std::endl;
            }

            if(whichQuat==0){
                posesWithPhantoms2[whichQuat]=Pose3d::getPoseBefore(posesWithPhantoms2[whichQuat+1],posesWithPhantoms2[whichQuat+2]);
            }

            if(whichQuat==poses2.size()-1){
                posesWithPhantoms2[whichQuat+2]=Pose3d::getPoseBefore(posesWithPhantoms2[whichQuat+1],posesWithPhantoms2[whichQuat]);
            }

            for(auto& p: poses2){
                std::cout<<p<<std::endl;
            }


            for (std::shared_ptr<Trajectory>& t : trajectories) {
                t->controlPosesUpdated(true);
            }
        });
    }

    V->displayFunctions2d.push_back([&trajectories]() {
        for (std::shared_ptr<Trajectory>& t : trajectories) {
            t->draw2d(t->color);
        }
    });
    V->displayFunctions.push_back([&trajectories]() {
        for (std::shared_ptr<Trajectory>& t : trajectories) {
            t->draw(t->color);
        }
    });
}

} // ns interpol
