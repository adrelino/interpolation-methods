/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include "trajectory/Pose3.h"
#include "Visualize.h"

namespace interpol {

template<>
void Pose3d::draw(int color, bool small,double scale) {
    if(Visualize::Toggled(GLFW_KEY_F7)) {
        if(small){
            glColor3dv(Colormap::get(color).data());
            glPointSize(15);
            glBegin(GL_POINTS);
            glVertex3dv(position.data());
            glEnd();
            glPointSize(1);
        }else{
            visualize::drawIso3d(getIsoInv(), color,scale);
        }
    }

    if(Visualize::Toggled(GLFW_KEY_F8)) {
        if(small){
            glColor4dv(Colormap::get(color).data());
            glPointSize(15);
            glBegin(GL_POINTS);
            Visualize::glVertex4dvQuaternion(orientation.coeffs());
            glEnd();
            glPointSize(1);
        }else {
            glColor4dv(Colormap::get(color).data());
            glPushMatrix();
            Visualize::glTranslate4dvQuaternion(orientation.coeffs());
            visualize::nanosphere.draw();
            glPopMatrix();
        }
    }
}

template<>
Poses3d Pose3d::getRegularSampledPosesWithPhantoms(const Poses3d &posesOriginal, double taus) {
    uint64_t start = posesOriginal[0].time;
    uint64_t end = posesOriginal.back().time;

    Poses3d basePoses;

    uint64_t Taus = taus*1e9;

    uint64_t i = start - Taus;
    while(i-2*Taus<end) {
        Pose3d p;
        p.time=i;
        basePoses.push_back(p);
        i +=Taus;
    }
    return basePoses;

}

template<>
Pose3d Pose3d::getPoseBefore(Pose3d& p0, Pose3d& p1) {
    Pose3d phantom0 = p0;
    phantom0.time -= (p1.time - p0.time);
    phantom0.position -= (p1.position - p0.position);
    phantom0.orientation *= p1.orientation.conjugate() * p0.orientation;
    return phantom0;
}

template<>
Poses3d Pose3d::addPhantomPoses(Poses3d poses) {
    Poses3d basePoses;
    int l = poses.size()-1;
    //cout<<"poses size "<<l+1<<endl;

    Pose3d phantom0 = getPoseBefore(poses[0],poses[1]);

    Pose3d phantomn = getPoseBefore(poses[l],poses[l-1]);

/*    Pose3d phantomn =  poses[l];
    phantomn.time += (poses[l].time - poses[l-1].time);
    phantomn.position += (poses[l].position - poses[l - 1].position);
    phantomn.orientation *= poses[l - 1].orientation.conjugate() * poses[l].orientation;*/

    //add at beginning and end;
    basePoses.push_back(phantom0);
    for(Pose3d pose : poses){
        basePoses.push_back(pose);
    }
    basePoses.push_back(phantomn);

    return basePoses;
}


#define XXXX(x) (x >= 0 ? "  " : " ")
std::ostream& operator<<(std::ostream &os, const Pose3d &pose) {

    os.setf(std::ios::fixed, std::ios::floatfield); // set fixed floating format
    os.precision(2);
    // os.fill(' ');
    os.width(5);
    //    os<<std::fixed<<std::internal;
    //    os.setf(std::ios_base::internal, std::ios_base::adjustfield);
    const Eigen::Matrix<double, 3, 1> &p(pose.position);
    const Eigen::Quaterniond &q(pose.orientation);
    //os <<setfill('0')<<setw(5);
    //std::fixed<<std::setprecision(2)<<std::setw(5)<<std::setfill('0')<<std::internal
    os << "time: " << pose.time << "\t" \
       << "    pos: " << XXXX(p.x()) << p.x() << XXXX(p.y()) << p.y() << XXXX(p.z()) << p.z() \
       << "    ori: " << XXXX(q.x()) << q.x() << XXXX(q.y()) << q.y() << XXXX(q.z()) << q.z() << XXXX(q.w()) << q.w();
    //const Eigen::IOFormat fmt(2/*Eigen::StreamPrecision*/, 0/*Eigen::DontAlignCols*/, " ", " ", "", "\t", "", "");
    return os;
}

} // ns interpol
