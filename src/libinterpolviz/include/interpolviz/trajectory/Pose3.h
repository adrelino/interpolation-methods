/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_POSE_H
#define INTERPOL_POSE_H

#include <interpol/utils/vector_a.hpp>
#include <iosfwd>
#include <string>
#include <iomanip>
#include <interpol/rigid.hpp>

namespace interpol {

#define PosX 0
#define PosY 1
#define PosZ 2
#define AngVel 3
#define LinAcc 4
#define LinVel 5
#define AngAcc 6

template <typename T>
Eigen::Matrix<T,3,1> toEigVec(const T* tra){
    return Eigen::Map<const Eigen::Matrix<T,3,1> >(tra);
}

template <typename T>
Eigen::Quaternion<T> toEigQuat(const T* rot){
    return Eigen::Map<const Eigen::Quaternion<T> >(rot);
}

// Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
template<typename T>
Sophus::SE3<T> toSophusSE3(const T* const cam){
    return Eigen::Map< const Sophus::SE3<T> >(cam);
}


template<typename Scalar>
class Pose3{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaternion<Scalar> orientation;
    Eigen::Matrix<Scalar,3,1> position;

    uint64_t time;
    bool fixed;


    const Sophus::SE3<Scalar> toSophusSE3() const{
        Sophus::SE3<Scalar> s(orientation, position);
        return s;
    }

    void fromSophusSE3(const Sophus::SE3<Scalar>& s){
        position = s.translation();
        orientation = s.unit_quaternion();
    }

    const DH1<Scalar> toDualQuat() const{
        return DH1<Scalar>(orientation,position);
    }

    void fromDualQuat(const DH1<Scalar>& dq){
        position = dq.translation();
        orientation = dq.rotation();
    }

    Pose3<Scalar>(uint64_t t=0, const Eigen::Matrix<Scalar,3,1>& p=Eigen::Matrix<Scalar,3,1>::Zero(), const Eigen::Quaternion<Scalar>& o=Eigen::Quaternion<Scalar>::Identity(), bool f=false) :
            time(t), fixed(f),
            position(p), orientation(o)
    {
    }

    Eigen::Transform<Scalar,3,Eigen::Isometry> getIso() const
    {
        Eigen::Transform<Scalar,3,Eigen::Isometry> T;
        T.translation() = position;
        T.linear()      = orientation.toRotationMatrix();
        return T;
    }

    Eigen::Transform<Scalar,3,Eigen::Isometry> getIsoInv(){
        return inverse().getIso().inverse();
    }

    Pose3<Scalar> operator*(const Pose3<Scalar>& rhs) const
    {
        //assertCustom(time == rhs.time);
        assertCustom(fixed == rhs.fixed);
        return Pose3<Scalar>(time,position+orientation*rhs.position, orientation*rhs.orientation, fixed);
    }

    /**
     * Rotate and translate a point.
     * to only rotate a directional vector (Vector3d), e.g. a normal, use this.orientation * normal
     */
    Eigen::Matrix<Scalar,3,1> operator*(const Eigen::Matrix<Scalar,3,1>& point) const
    {
        Eigen::Matrix<Scalar,3,1> result;
        result = position + orientation*point;
        return result;
    }


    //http://math.stackexchange.com/questions/1234948/inverse-of-a-rigid-transformation
    Pose3<Scalar> inverse() const
    {
        Eigen::Quaternion<Scalar> invOri = orientation.conjugate(); //conjugate is inverse orientation
        Eigen::Matrix<Scalar,3,1> invPos = -(invOri*position);
        return Pose3<Scalar>(time,invPos,invOri);
    }

    Pose3<Scalar> lerp(Scalar l, const Pose3<Scalar>& other, uint64_t t) const
    {
        Pose3<Scalar> p = Pose3<Scalar>(t);
        p.orientation = SLERP(orientation,l, other.orientation);
        p.position = position + (other.position - position)*l;
        return p;
    }

    static inline Pose3<Scalar> wrap(const Scalar* const c0_a){
        Pose3<Scalar> p(0,translationPart(c0_a),rotationPart(c0_a));
        p.linearAcc=linearAccPart(c0_a);
        p.angularVel=angularVelPart(c0_a);
        return p;
    }



    typedef vector_a <Pose3<Scalar>> Poses3;


    ///////////
    // STATIC functions on lists of poses


    static Poses3 getRegularSampledPosesWithPhantoms(const Poses3& posesOriginal, double taus);
    static Pose3 getPoseBefore(Pose3& a, Pose3& b);
    static Poses3 addPhantomPoses(Poses3 poses);

    static void centerAtOrigin(Poses3& poses){
        Eigen::Vector3d foo = poses[0].position;
        Eigen::Vector3d foo2 = poses.back().position;
        Eigen::Vector3d offset = (foo + foo2)/Scalar(2);

        for(Pose3<Scalar>& it : poses){
            it.position = it.position - offset;
        }
    }

    static uint64_t useRelativeTime(Poses3& poses){
        uint64_t relativeToAbsoluteStartTimeOffset = poses.front().time;

        relativeToAbsoluteStartTimeOffset -= 2000000000;//2 second buffer before for splines

        for(Pose3<Scalar>& it : poses){
            it.time -= relativeToAbsoluteStartTimeOffset;
        }

        return relativeToAbsoluteStartTimeOffset;
    }

    /**
     * @brief applyDelta
     * @param poses copy constructed and returned after update
     * @param delta passed by const ref
     * @return
     */
    static void applyDelta(Poses3& poses, const Poses3& delta){
        assertCustom(poses.size()==delta.size());

        for (int i = 0; i < poses.size(); ++i) {
            assertCustom(poses[i].time == delta[i].time);

            poses[i] = delta[i]*poses[i];
        }
    }

    static Poses3 initWithTime(const Poses3& original){
        Poses3 poses;
        for(const Pose3& p : original){
            poses.push_back(Pose3(p.time));
        }
        return poses;
    }
    void draw(int color, bool small=false, double scale=3.0);

}; //end class Pose3




typedef Pose3<double> Pose3d;
std::ostream& operator<<(std::ostream &os, const Pose3d& pose);

typedef vector_a<Pose3d> Poses3d;


static int binary_search_lowerIntervalIndex(const Poses3d& poses, uint64_t time){
    int left = 0;
    int right = poses.size()-1;

    while (left <= right){
        int middle = (left + (right-left) / 2);

        const uint64_t& timeMiddle = poses[middle].time;

        if(timeMiddle <= time && time <= poses[middle+1].time){
            return middle;
        }else{
            if(timeMiddle > time) right=middle-1;// go left
            else left=middle+1; //go right
        }
    }

    return -1; // not found
}

static int linearInterpolateTimestampToClosest2TrajectorySamples(const Poses3d& poses, const uint64_t& time, double& t){
    int idx = binary_search_lowerIntervalIndex(poses,time);
    const Pose3d& left = poses[idx];
    const Pose3d& right = poses[idx+1];
    t = (time - left.time)*1.0 / (right.time - left.time);
    return idx;
}


} // ns interpol


#endif
