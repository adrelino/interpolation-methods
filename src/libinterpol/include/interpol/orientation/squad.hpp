/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_SQUAD_HPP
#define INTERPOL_SQUAD_HPP

#include "SU2.hpp"
#include <iostream>
#include <iomanip>
#include <interpol/utils/vector_a.hpp>

namespace interpol {

namespace squad {

//https://groups.google.com/forum/#!topic/comp.graphics.algorithms/2Kvsa8r2w8o
//https://theory.org/software/qfa/writeup/node12.html
//http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
//https://gist.github.com/Veguard
//http://www.gamasutra.com/blogs/VegardMyklebust/20150911/253461/Spherical_Spline_Quaternions_For_Dummies.php


//s_i computation in SQUAD formula
static Eigen::Quaterniond intermediate(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2){
    Eigen::Quaterniond q1inv(q1.conjugate());

    Eigen::Quaterniond q1inv2 = q1inv*q2;
    Eigen::Quaterniond l(logq(q1inv2));

    Eigen::Quaterniond q1inv0 = q1inv*q0;
    Eigen::Quaterniond r(logq(q1inv0));

    Eigen::Quaterniond inter(q1 * expq(-0.25*(l + r)));

    double norm = inter.norm();
    assertCustom(norm < 1+1e-12 && norm > 1-1e-12);

    return inter;
//    return g2e(glm::intermediate(e2g(q0),e2g(q1),e2g(q2))); //for debugging
}


static Eigen::Vector3d intermediate(const Eigen::Vector3d& q0, const Eigen::Vector3d& q1, const Eigen::Vector3d& q2){
    Eigen::Vector3d l = q2-q1;
    Eigen::Vector3d r = q0-q1;
    Eigen::Vector3d inter = q1 +(-0.25*(l + r));
    //Vector3d inter(q1 - (q2 - 2*q1 + q0)/4);
    return inter;
}

//https://gist.github.com/Veguard/c59d5f7d35240733b80b
// Returns a smooth approximation between q1 and q2 using t1 and t2 as 'tangents'
// q0 and q3 are outsite of this spline segment, but needed to compute tangents t1 and t2
template <typename T>
class SQUAD{
  public:
    T q1,t1,t2,q2;

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SQUAD(const T& qq0,const T& qq1, const T& qq2, const T& qq3):
          q1(qq1),t1(intermediate(qq0,qq1,qq2)),t2(intermediate(qq1,qq2,qq3)),q2(qq2)
    {
       // cout<<"SQUAD\t"<<q1<<",\t"<<tt1<<",\t"<<t2<<",\t"<<q2<<endl;
    }

    /**
     * SQUAD evaluation for one segment (q1,q2,t1,t2)
     * @param u in [0,1]
     */
    T eval(double u) const{
        //not 1 recursion of de casteljau but bilinear spherical blending
        T slerpOuter = SLERP(q1,u,q2);
        T slerpInner = SLERP(t1,u,t2);
        double uSlerp = 2.0 * u * (1.0 - u);
        return SLERP(slerpOuter,uSlerp,slerpInner);
    //  return g2e(glm::squad(e2g(q1),e2g(q2),e2g(t1),e2g(t2),u)); //debug
    }
};

template <typename T>
class ShoemakeC1Spline{
public:
    ShoemakeC1Spline() = default;
    ShoemakeC1Spline(const vector_a<T>& qs, bool initialVelocityNotZero=false){
        n = qs.size();
        assertCustom(n >= 2);

        for (int i = 0; i < n-1; ++i) { //for n==2, we want only 1 spline segment == slerp == one loop with i=0

            int first = i-1;
            const T& q1 = qs[i];
            const T& q2 = qs[i+1];
            int last = i+2;

            T qfirst, qlast;
            if(first<0){
                first=0;//same as =i;
            }
            //0;   //duplicate first quaternion at beginning -> 0 1'st derivative boundary condition
            if(last>n-1){
                last=n-1;//same as =i+1;
            }
            assertCustom(last < qs.size());
            qfirst=qs[first];
            qlast=qs[last];
            //n-1; //duplicate last quaternion at end -> 0 1'st derivative boundary condition

            //if(initialVelocityNotZero){
              //  if(first==0) qfirst=((q1*q2)*q1.conjugate()).normalized();
                //if(last==n-1) qlast=(q2.conjugate()*(q1*q2)).normalized();
            //}

           // cout<<"Spline segment indices: "<<first<<","<<i<<","<<i+1<<","<<last<<endl;
            SQUAD<T> s(qfirst,q1,q2,qlast);

            segments.push_back(s);
        }
    }

    T eval(double tGlobal) const{
        int section = (n-1) * tGlobal;
        double t = (n-1) * tGlobal - section;
        if(section==segments.size()){
            section -= 1;
            t=1.0;
        }
        assertCustom(section<segments.size());
        return segments[section].eval(t);
    }

    vector_a<SQUAD<T>> segments;

    vector_a<T> getControlPolygon(){
        vector_a<T> control;
        for (int i = 0; i < segments.size(); ++i) {
            control.push_back(segments[i].q1);
            control.push_back(segments[i].t1);
            control.push_back(segments[i].t2);
            control.push_back(segments[i].q2);
        }
        return control;
    };

private:
    size_t n;
};

typedef ShoemakeC1Spline<Eigen::Quaterniond> QSpline;
typedef ShoemakeC1Spline<Eigen::Vector3d> VSpline;

} // ns squad

} // ns interpol

#endif // INTERPOL_SQUAD_HPP
