/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "Trajectory.h"
#include "Visualize.h"

namespace interpol {

Trajectory::Trajectory(Poses3d& poses, bool posesAreWithPhantomPosesFirstAndLast)
    : basePoses(poses), posesAreWithPhantomPosesFirstAndLast(posesAreWithPhantomPosesFirstAndLast) {
    controlPosesUpdated(false);
}

Poses3d Trajectory::sampleRegular(uint64_t incrNanoSec) {
    //if(regularSampledPoses.size()==0 || incrNanoSec != incrLast) {
        regularSampledPoses.clear();
        //size_t nOriginal = poses.size();
        //uint64_t incrNanoSec = 1000000000 * incr;
        // TODO: we miss end pose if regular sampled poses dont fall exactly on it

        for (uint64_t t = timeOfFirstSupportPose; t <=timeOfLastSupportPose; t += incrNanoSec) {
            Pose3d supportPoint = getPoseAt(t);
            regularSampledPoses.push_back(supportPoint);
            //cout<<supportPoint<<endl;
        }
        //cout<<"sampled regular every: "<<incrNanoSec<<"ns, giving "<<regularSampledPoses.size()<<" poses:"<<endl;

        incrLast=incrNanoSec;
    //}
    return regularSampledPoses;
}

Poses3d Trajectory::sampleRegular() {
    return sampleRegular(incrLast);
}

void Trajectory::controlPosesUpdated(bool sampleAgain){

    int offset = posesAreWithPhantomPosesFirstAndLast ? 1 : 0;
    int l = basePoses.size();


    for (int i = 0; i < l-1; ++i) {
        uint64_t deltaTc = basePoses[i+1].time - basePoses[i].time;
        if(i==0) deltaT=deltaTc;
        if(deltaTc != deltaT){
            deltaT = UINT64_MAX;
        }
    }


    for (int i = 0+offset; i < l-1-offset; ++i) {
        Eigen::Quaterniond currentOtherSide(basePoses[i + 1].orientation);
        currentOtherSide.coeffs() *= -1;
        double dist1 = basePoses[i].orientation.dot(basePoses[i + 1].orientation);
        double dist2 = basePoses[i].orientation.dot(currentOtherSide);
        //cout<<"dist1="<<dist1<<"  dist2="<<dist2<<endl;
        if (dist1 < dist2) {
            basePoses[i + 1].orientation = currentOtherSide;
        }
    }


    timeOfFirstSupportPose = basePoses[0+offset].time;
    timeOfLastSupportPose = basePoses[basePoses.size()-1-offset].time;
    duration = timeOfLastSupportPose-timeOfFirstSupportPose;
    incrLast = 1e9/25; //1/100st second, same as imu frequency 100Hz

/*        std::cout<<"Trajectory::controlPosesUpdated with "<<basePoses.size()<<" poses with phantom poses? "<<posesAreWithPhantomPosesFirstAndLast;
    std::cout<<" from "<<timeOfFirstSupportPose;
    std::cout<<" to "<<timeOfLastSupportPose<<" duration (ns):"<<duration;
    std::cout<<" sampleRegular (ns):"<<incrLast;
    if(isUniform()) cout<<" uniform times of "<<deltaT<<endl;
    else cout<<" nonuniform"<<endl;*/

    if(sampleAgain) {
        regularSampledPoses.clear();
        regularSampledPoses = sampleRegular();
        //cout<<"controlPosesUpdated "<<name<<endl;
    }
}


void Trajectory::setEndTime(uint64_t end){
    timeOfLastSupportPose = end;
    duration = timeOfLastSupportPose-timeOfFirstSupportPose;
    controlPosesUpdated();
}

static void interpolateLinearAtEnds(std::vector<double>& vel){
    int n = vel.size();
    vel.push_back(vel[n-1]+(vel[n-1]-vel[n-2]));
    vel[0] = vel[1]+(vel[1]-vel[2]);
}

std::vector<double> Trajectory::angularVelocity() const{
    std::vector<double> vel;
    vel.push_back(0);
    for (int i = 1; i < regularSampledPoses.size()-1; ++i) {
        auto q0 = regularSampledPoses[i-1].orientation;
        auto q1 = regularSampledPoses[i].orientation;
        auto q2 = regularSampledPoses[i+1].orientation;

        double v = ((q1.coeffs()-q0.coeffs()).norm() + (q1.coeffs() - q2.coeffs()).norm());
        v /=incrLast*1e-9;
        vel.push_back(v);
    }
    interpolateLinearAtEnds(vel);
    return vel;
}

//F=m*a, so it is proportional to torque
std::vector<double> Trajectory::angularAcceleration() const{
    std::vector<double> vel;
    vel.push_back(0);

    for (int i = 1; i < regularSampledPoses.size()-1; ++i) {
        auto q0 = regularSampledPoses[i-1].orientation.coeffs();
        auto q1 = regularSampledPoses[i].orientation.coeffs();
        auto q2 = regularSampledPoses[i+1].orientation.coeffs();

        //f(x) = q1
        //backward difference
        Eigen::Vector4d diff = q1-q0;
        diff.array() /=(incrLast*1e-9);
        //forward difference
        Eigen::Vector4d diff2 = q2-q1;
        diff2.array() /=(incrLast*1e-9);

        Eigen::Vector4d difdiff = diff2-diff;
        double v1 = difdiff.norm()/(incrLast*1e-9);
        //vel.push_back(v1);

        //2nd order central differnce
        Eigen::Vector4d difdiff2 = q2 - 2*q1 + q0;
        double v2 = difdiff2.norm() /((incrLast*1e-9)*(incrLast*1e-9)); //h²
        //assertCustom(abs(v1-v2)<0.00000001);
        vel.push_back(v2);
    }
    interpolateLinearAtEnds(vel);

    return vel;
}

std::vector<double> Trajectory::posComp(int idx) const{
    std::vector<double> vel;

    for (int i = 0; i < regularSampledPoses.size(); ++i) {
        auto q1 = regularSampledPoses[i].position[idx];
        vel.push_back(q1);
    }

    return vel;
}


std::vector<double> Trajectory::linearVel() const{
    std::vector<double> vel;

    vel.push_back(0);

    for (int i = 1; i < regularSampledPoses.size()-1; ++i) {
        auto q0 = regularSampledPoses[i-1].position;
        auto q1 = regularSampledPoses[i].position;
        auto q2 = regularSampledPoses[i+1].position;

        //backward difference
        Eigen::Vector3d diff = q1-q0;
        diff.array() /=(incrLast*1e-9);

        //vel.push_back(diff.norm()/(incrLast*1e-9));

        //forward difference
        Eigen::Vector3d diff2 = q2-q1;
        diff2.array() /=(incrLast*1e-9);

        Eigen::Vector3d meanOfBoth = diff + diff2;
        meanOfBoth.array() /= 2.0;

        vel.push_back(meanOfBoth.norm());
    }
    interpolateLinearAtEnds(vel);


    return vel;
}


std::vector<double> Trajectory::linearAcc() const{
    std::vector<double> vel;
    vel.push_back(0);

    for (int i = 1; i < regularSampledPoses.size()-1; ++i) {
        auto q0 = regularSampledPoses[i-1].position;
        auto q1 = regularSampledPoses[i].position;
        auto q2 = regularSampledPoses[i+1].position;

        //f(x) = q1
        //backward difference
        Eigen::Vector3d diff = q1-q0;
        diff.array() /=(incrLast*1e-9);
        //forward difference
        Eigen::Vector3d diff2 = q2-q1;
        diff2.array() /=(incrLast*1e-9);

        Eigen::Vector3d difdiff = diff2-diff;
        double v1 = difdiff.norm()/(incrLast*1e-9);
        //vel.push_back(v1);

        //2nd order central differnce
        Eigen::Vector3d difdiff2 = q2 - 2*q1 + q0;
        double v2 = difdiff2.norm() /((incrLast*1e-9)*(incrLast*1e-9)); //h²
        //assertCustom(abs(v1-v2)<0.00000001);
        vel.push_back(v2);
    }
    interpolateLinearAtEnds(vel);

    return vel;
}

std::vector<double> Trajectory::timeInSeconds() const{
    std::vector<double> times;

    for (int i = 0; i < regularSampledPoses.size(); ++i) {
        times.push_back((regularSampledPoses[i].time-timeOfFirstSupportPose)*1e-9);
    }

    return times;
}

std::vector<double> Trajectory::getDeriv(int test) const{
    if(test<=PosZ) return posComp(test);
    if(test==AngVel) return angularVelocity();
    if(test==LinAcc) return linearAcc();
    if(test==LinVel) return linearVel();
    if(test==AngAcc) return angularAcceleration();
}

void Trajectory::draw2d(int colorChoice){
    if(!show) return;
    if(colorChoice<0) colorChoice=color;

    if(basePoses.size()==0) return;

    if(regularSampledPoses.size()==0){
        sampleRegular();
    }

    //getPoseAt(Visualize::getInstance()->now).draw2d(colorChoice,Visualize::getInstance()->nowX());

    double max2d = Visualize::getInstance()->maxScale2D;

    int enums[] = {AngVel,LinAcc,LinVel,AngAcc};
    int keys[] = {GLFW_KEY_R,GLFW_KEY_A,GLFW_KEY_V,GLFW_KEY_T};//T for torque

    for(int i=0; i<4; i++){
        if(Visualize::Toggled(keys[i])) {
            if(Visualize::Toggled(GLFW_KEY_F6)) visualize::draw2dPoints(regularSampledPoses, colorChoice,enums[i]);
            visualize::draw2dLine(getDeriv(enums[i]), colorChoice);
        }
    }
}

void Trajectory::draw(int colorChoice, bool drawBasePoses){
    if(!show) return;
    if(colorChoice<0) colorChoice=color;

    if(basePoses.size()==0) return;

    if(regularSampledPoses.size()==0){
        sampleRegular();
    }

    //realtime pose is always big
    if(Visualize::Toggled(GLFW_KEY_F5)) getPoseAt(Visualize::getInstance()->now).draw(colorChoice,false);

    bool smallPose = Visualize::Toggled(GLFW_KEY_F4);

    int lineFromOrigin = Visualize::ToggledMulti(GLFW_KEY_F4,GLFW_MOD_SHIFT,2);
    if(!Visualize::Toggled(GLFW_KEY_F8)) lineFromOrigin=0;


    double scale=2.0;
    if(this->name=="DLUp") scale=1.0;


    //colored by segment
    int mode = Visualize::ToggledMulti(GLFW_KEY_F5,GLFW_MOD_SHIFT,3);
    if(mode){
        int segments = 5;
        auto inc = duration/segments;
        for (int i = 1; i < segments; ++i) {
            getPoseAt(timeOfFirstSupportPose+i*inc).draw((mode==1) ? (i-1) : ((mode==2) ? colorChoice : GRAY),smallPose,scale);
        }

        if(lineFromOrigin){
        glPushAttrib(GL_ENABLE_BIT);glLineStipple(1, 0xAAAA);glEnable(GL_LINE_STIPPLE);
        glLineWidth(2);
        glColor3d(0,0,0);
        glBegin(GL_LINES);
        for (int i = 0; i <= segments; ++i) {
            glColor3dv(Colormap::get(BLACK).data());
            glVertex3d(0,0,0);
            Visualize::glVertex4dvQuaternion(getPoseAt(timeOfFirstSupportPose+i*inc).orientation.coeffs());
        }
        //even linear spacing
        if(lineFromOrigin==2){
            Visualize::glVertex4dvQuaternion(getPoseAt(getStartTime()).orientation.coeffs());
            Visualize::glVertex4dvQuaternion(getPoseAt(getEndTime()).orientation.coeffs());
        }
        glEnd();
        glPopAttrib();
        }

    }

    bool isAlternativeTraj = false;//modifiers > 0;

    glColor3dv(Colormap::get(colorChoice).data());

    if(isAlternativeTraj) glColor4dv(Colormap::get(colorChoice).data());

    glLineWidth(3);
    if(this->name=="DLUp") glLineWidth(1);


    if(Visualize::Toggled(GLFW_KEY_F7)) {
        //regularSampled Poses must be called before. Draws actuall continious trajectory, discretized with small time steps
        //higher frequency sampled poses, approximate curve
        glBegin(GL_LINE_STRIP);
        for (auto &p : regularSampledPoses) {
            if(p.time>Visualize::getInstance()->end) break;
            glVertex3dv(p.position.data());
        }
        glEnd();
    }


    if(Visualize::Toggled(GLFW_KEY_F8)){
        glBegin(GL_LINE_STRIP);
        for(auto& p : regularSampledPoses){
            if(p.time>Visualize::getInstance()->end) break;
            Visualize::glVertex4dvQuaternion(p.orientation.coeffs());
        }
        glEnd();
    }

    //surrounding polyhedron of base poses in both R3 and S3 space
    if(Visualize::Toggled(GLFW_KEY_C)){
        glLineWidth(1);
        Poses3d controlPolygon = getControlPolygon();
        for (int i = 0; i < controlPolygon.size()-1; ++i) {
            const Pose3d& p1 = controlPolygon[i];
            const Pose3d& p2 = controlPolygon[i+1];
            //if(p2.time>Visualize::getInstance()->end) break;



            glPushAttrib(GL_ENABLE_BIT);glLineStipple(2, 0xAAAA);glEnable(GL_LINE_STIPPLE);

            if(Visualize::Toggled(GLFW_KEY_F7)) {
                glBegin(GL_LINE_STRIP);
                glVertex3dv(p1.position.data());
                glVertex3dv(p2.position.data());
                glEnd();
            }

            if(Visualize::Toggled(GLFW_KEY_F8)) {
                glBegin(GL_LINE_STRIP);
                for (int j = 0; j <= 100; ++j) {
                    auto q = SLERP(p1.orientation, 0.01 * j, p2.orientation);
                    //                    auto q = p1.orientation.slerp(j*0.1,p2.orientation);
                    Visualize::glVertex4dvQuaternion(q.coeffs());
                }
                /*                Visualize::glVertex4dvQuaternion(p1.orientation.coeffs());
                                Visualize::glVertex4dvQuaternion(p2.orientation.coeffs());*/
                glEnd();
            }

            glPopAttrib();
        }
    }

    if(Visualize::Toggled(GLFW_KEY_C,GLFW_MOD_SHIFT)) {
        Poses3d controlPolygon = getControlPolygon();
        for (Pose3d &p : controlPolygon) {
            //if(p.time>Visualize::getInstance()->end) break;
            p.draw(colorChoice,true);
        }
    }


    //base Poses, filled in by Trajectory Constructor method, may contain phantom poses in case of b spline
    //base poses of Trajectory. These are usually only approximated and may be more then poses
    //stützstellen. Do not need to lie on trajectory (e.g. in case of spline)
    if(drawBasePoses && Visualize::Toggled(GLFW_KEY_B)) {
        for (Pose3d &p : basePoses) {
            //if(p.time>Visualize::getInstance()->end) break;
            p.draw(/*p.fixed ? BLACK : colorChoice*/ GRAY,smallPose,scale);
        }
    }

    if(drawBasePoses && Visualize::Toggled(GLFW_KEY_B,GLFW_MOD_SHIFT)) {
        for (Pose3d &p : basePoses) {
            if(p.time>Visualize::getInstance()->end) break;
            if(timeOfFirstSupportPose <= p.time && p.time <= timeOfLastSupportPose) getPoseAt(p.time).draw(p.fixed ? BLACK : colorChoice,true);
        }
    }
}



//////////////////////
//STATIC FUNCTIONS

void Trajectory::initDisplay(){
    Visualize* V = Visualize::getInstance();
    V->toggle(GLFW_KEY_B,true,"draw base poses");
    V->toggle(GLFW_KEY_C,false,"draw control polygon");

    V->toggle(GLFW_KEY_F5,false,"now");
    V->toggle(GLFW_KEY_F6,false,"2d analytic deriv");
    V->toggle(GLFW_KEY_F7,false,"R3");
    V->toggle(GLFW_KEY_F8,false,"S3");
    V->toggle(GLFW_KEY_U,false,"unit sphere",GLFW_MOD_SHIFT);


    V->toggle(GLFW_KEY_R,false,"angular velocity");
    V->toggle(GLFW_KEY_V,false,"linear velocity");
    V->toggle(GLFW_KEY_A,false,"linear acceleration");


    V->toggle(GLFW_KEY_A,false,"ate",GLFW_MOD_SHIFT);
    V->toggle(GLFW_KEY_R,false,"rpe",GLFW_MOD_SHIFT);


}

void Trajectory::toggle(int key_, int modifiers_, bool show_, std::string name_, int color_) {
    key = key_;
    name = name_;
    show = show_;
    color = color_;
    modifiers = modifiers_;
    Visualize::getInstance()->toggle(key,show,name,[this](bool showNew){
        show=showNew;
    },modifiers);
}

} // ns interpol
