/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_DH_HPP
#define INTERPOL_DH_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include <hand_eye_calibration/DualQuaternion.h>

namespace interpol {

template<typename T>
class DH1;

//We have a new type for (screw) tangent since multiplication with scalar is defined different there
template<typename T>
class DH1Tangent
{
public:
    DH1Tangent(const Eigen::Quaternion<T>& r,const Eigen::Quaternion<T>& d) : m_real(r), m_dual(d) {}

    // dqTangent * scalar
    DH1Tangent<T> operator*(const T& scale) const{
        DH1Tangent<T> tt(m_real,m_dual);
        tt.m_real.w() *= scale; //theta
        tt.m_dual.w() *= scale; //d
        return tt;
    }

    //implemented below
    DH1<T> expScrewBen(void) const;
    DH1<T> expScrew(void) const;
    DH1<T> expT(bool useBenKenwrightImpl=false) const{
        return useBenKenwrightImpl ? expScrewBen() : expScrew();
    }

    Eigen::Quaternion<T> m_real; // real part
    Eigen::Quaternion<T> m_dual; // dual part

    DH1<T> toDH(void){//for derivative
        return DH1<T>(m_real,m_dual);
    }
};

// scalar * dqTangent
template<typename T>
inline DH1Tangent<T> operator*(T scale, DH1Tangent<T> t){
    t.m_real.w() *= scale;
    t.m_dual.w() *= scale;
    return t;
}
typedef DH1Tangent<float> DH1Tangentf;
typedef DH1Tangent<double> DH1Tangentd;



template<typename T>
DH1<T>
operator+(const DH1<T>& dq1, const DH1<T>& dq2);

template<typename T>
class DH1
{
public:
    DH1(){}
    DH1(const Eigen::Quaternion<T>& r,
        const Eigen::Quaternion<T>& d) : DH1(px::DualQuaternion<T>(r,d)) {}
    DH1(const Eigen::Quaternion<T>& r,
        const Eigen::Matrix<T, 3, 1>& t) : DH1(px::DualQuaternion<T>(r,t)) {}


    void fromScrew(T theta, T d,
                   const Eigen::Matrix<T, 3, 1>& l,
                   const Eigen::Matrix<T, 3, 1>& m);
    void toScrew  (T& theta, T& d, //added by us
                   Eigen::Matrix<T, 3, 1>& l,
                   Eigen::Matrix<T, 3, 1>& m) const;


    DH1Tangent<T> logScrewBen(void) const;
    DH1Tangent<T> logScrew(void) const;
    DH1Tangent<T> logT(bool useBenKenwrightImpl=false) const{
        return useBenKenwrightImpl ? logScrewBen() : logScrew();
    }

    DH1<T> conjugate(void) const { return DH1<T>(dq.conjugate());}
    DH1<T> normalized(void) const {return DH1<T>(dq.normalized());}
    Eigen::Quaternion<T> real() const {return dq.real();}
    Eigen::Quaternion<T> dual() const {return dq.dual();}
    Eigen::Quaternion<T> rotation(void) const {return dq.rotation();}
    Eigen::Matrix<T, 3, 1> translation(void) const {return dq.translation();}
    DH1<T> operator*(T scale) const {return DH1<T>(dq*scale);}
    DH1<T> operator*(const DH1<T>& other) const {return DH1<T>(dq*other.dq);}
    friend DH1<T> operator+<>(const DH1<T>& dq1, const DH1<T>& dq2);

    DH1<T>& operator*=(const DH1<T>& other);//added by us


private:
    DH1(const px::DualQuaternion<T>& dq) : dq(dq) {}
    px::DualQuaternion<T> dq;
    //there must be a mistake here for the dual part....
    //DH1<T> exp(void) const;
    //DH1<T> exp(const DH1<T> base) const;
    //px::DualQuaternion<T> log(void) const;
    //px::DualQuaternion<T> log(const px::DualQuaternion<T> &b) const;
};

template<typename T>
DH1<T>&
DH1<T>::operator*=(const DH1<T>& other)
{
    dq = dq * other.dq;
    return *this;
}

template<typename T>
DH1<T>
operator+(const DH1<T>& dh1, const DH1<T>& dh2)
{
    return DH1<T>(dh1.dq+dh2.dq);
}

template<typename T>
void
DH1<T>::fromScrew(T theta, T d,
        const Eigen::Matrix<T, 3, 1>& l,
        const Eigen::Matrix<T, 3, 1>& m)
{
    //was provided in original github implementation.
    dq.fromScrew(theta,d,l,m);
    //is equal to:
    //m_real = Eigen::AngleAxis<T>(theta, l);
    //m_dual.w() = -0.5*d * std::sin(0.5*theta);
    //m_dual.vec() = std::sin(0.5*theta) * m + d / 2.0 * std::cos(theta / 2.0) * l;
}

//modeled by hand as inverse of above
template<typename T>
void
DH1<T>::toScrew(T& theta, T& d,
        Eigen::Matrix<T, 3, 1>& l,
        Eigen::Matrix<T, 3, 1>& m) const
{

    Eigen::AngleAxis<T> ar(real()) ;
    theta=ar.angle();
    l=ar.axis();

    d = -2.0 * dual().w() / (std::sin(0.5*theta));
    m = (dual().vec() - 0.5*d*std::cos(0.5*theta)*l) / std::sin(0.5*theta);
}

//First half of ScLERPGeometricCosSin
template<typename T>
DH1Tangent<T>
DH1<T>::logScrewBen(void) const
{
    Eigen::Vector3d vr = real().vec();
    Eigen::Vector3d vd = dual().vec();
    double invr = 1.0 / vr.norm();

    // Screw parameters
    double  angle = 2.0 * std::acos( real().w() );
    double  pitch = -2.0 * dual().w() * invr;
    Eigen::Vector3d direction = vr * invr;
    Eigen::Vector3d moment = (vd - direction*pitch*real().w()*0.5)*invr;

    //TODO: absorb angle into norm of axis
    Eigen::Quaterniond realTangent;
    realTangent.w()=angle;//0
    realTangent.vec()=direction;//* angle

    //absorb translation into norm of moment
    Eigen::Quaterniond dualTangent;
    dualTangent.w()=pitch;//0;
    dualTangent.vec()=moment;// * pitch;


    return DH1Tangent<T>(realTangent, dualTangent);
}

//Second half of ScLERPGeometricCosSin
template<typename T>
DH1<T>
DH1Tangent<T>::expScrewBen(void) const
{
    //TODO: real and dual tangents should have w() = 0 and the norm absorbed into their vector part
    Eigen::Vector3d vr = m_real.vec();
    Eigen::Vector3d vd = m_dual.vec();

    double angle = m_real.w();//vr.norm();
    double pitch = m_dual.w();//vd.norm();

    Eigen::Vector3d direction = vr; // / angle;
    Eigen::Vector3d moment = vd;// / pitch;


    // Convert back to dual-quaternion
    double sinAngle = std::sin(0.5*angle);
    double cosAngle = std::cos(0.5*angle);

    Eigen::Vector3d axisReal = direction * sinAngle;
    Eigen::Quaterniond real(cosAngle, axisReal.x(),axisReal.y(),axisReal.z());

    Eigen::Vector3d axisDual = sinAngle*moment+pitch*0.5* cosAngle *direction;
    Eigen::Quaterniond dual(-pitch*0.5*sinAngle, axisDual.x(),axisDual.y(),axisDual.z());

    return DH1<T>(real, dual);
}


template<typename T>
DH1Tangent<T>
DH1<T>::logScrew(void) const
{
    T theta,d;
    Eigen::Matrix<T, 3, 1> axis, moment;
    toScrew(theta,d,axis,moment); //putting amount of angle or translation in w(), so it is not 0!

    //TODO: absorb angle and translation into norm of axis and moment
    Eigen::Quaternion<T> real,dual;
    real.w() = theta; //0;
    real.vec() = axis; //*theta;
//        real.w() = 0;
//        real.vec() = axis*theta;

    dual.w() = d; //0;
    dual.vec() = moment; //*d;

//        double m = moment.norm();
//        dual.w() /=m;
//        dual.vec() /= m;
//        cout<<"axis norm="<<axis.norm()<<"  moment norm="<<m<<" d="<<d<<endl;

    return DH1Tangent<T>(real, dual);
}

template<typename T>
DH1<T>
DH1Tangent<T>::expScrew(void) const
{
    DH1<T> dualQuat;
    //TODO: get angle and translation from norm of axis and moment
    dualQuat.fromScrew(
                m_real.w(),//vec().norm(),
                m_dual.w(),//vec().norm(),
                m_real.vec(),//.normalized(),
                m_dual.vec()//.normalized()
                       );
    return dualQuat;
}

typedef DH1<float> DH1f;
typedef DH1<double> DH1d;

} // ns interpol

#endif // INTERPOL_DH_HPP
