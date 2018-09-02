/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include <interpol/rigid.hpp>
#include <interpol/utils/timer.hpp>
#include <interpol/utils/random.hpp>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace interpol;

int main(int argc, char* argv[]){

    Timer t3;

//    Eigen::Quaterniond q0 = random::quaternionUniform(random::QuaternionSampling::NormalDistribution);cout<<"q0="<<q0<<endl;
//    Eigen::Quaterniond q1 = random::quaternionUniform(random::QuaternionSampling::NormalDistribution);cout<<"q1="<<q1<<endl;
//    Eigen::Vector3d v0 = random::vectorUniform(100);cout<<"v0="<<v0<<endl;
//    Eigen::Vector3d v1 = random::vectorUniform(100);cout<<"v1="<<v1<<endl;


    Eigen::Quaterniond q0(-0.43, 0.65, -0.58, -0.23);cout<<"q0="<<q0<<endl;
    Eigen::Quaterniond q1(-0.13, -0.60, -0.78, 0.10);cout<<"q1="<<q1<<endl;
    Eigen::Vector3d v0(45.72, 91.40, 53.44);cout<<"v0="<<v0<<endl;
    Eigen::Vector3d v1(77.84, 93.91, 43.07);cout<<"v1="<<v1<<endl;



    DH1d d0(q0,v0);
    Sophus::SE3d s0(q0,v0);
    R3xSO3 p0(q0,v0);

    DH1d d1(q1,v1);
    Sophus::SE3d s1(q1,v1);
    R3xSO3 p1(q1,v1);


    std::cout<<"=====  Accurracy ===== "<<std::endl;
    for(double u=0.1; u<=.9; u+=0.1){
        cout<<setprecision(1)<<std::fixed<<"u="<<u<<"\t"<<setprecision(2)<<std::fixed;
        auto slerp0=SPLIT(p0,u,p1);
        auto slerp1=SE3Up(s0,u,s1);
        cout<<"SPLIT-SE3UP-rot:\t"<<rot(slerp0).dot(rot(slerp1))<<"\t\t";
        cout<<"SPLIT-SE3UP-tra:\t"<<(tra(slerp0)-tra(slerp1)).norm()<<"\t\t";

        auto slerp2 = ScLERPGeometricCosSin(d0,u,d1);
        cout<<"SE3UP-ScLERP-rot:\t"<<rot(slerp2).dot(rot(slerp1))<<"\t\t";
        cout<<"SE3UP-ScLERP-tra:\t"<<(tra(slerp1)-tra(slerp2)).norm()<<"\t\t";

        auto slerp3 = DLUp(d0,u,d1);
        cout<<"ScLERP-DLUp-rot:\t"<<rot(slerp2).dot(rot(slerp3))<<"\t\t";
        cout<<"ScLERP-DLUp-tra:\t"<<(tra(slerp2)-tra(slerp3)).norm()<<"\t\t";

        auto slerp4 = SPAPP(p0,u,p1);
        cout<<"SPApp-DLUp-rot:\t"<<rot(slerp4).dot(rot(slerp3))<<"\t\t";
        cout<<"SPAPP-DLUp-tra:\t"<<(tra(slerp4)-tra(slerp3)).norm()<<endl;
    }
    

    auto test1=p0;
    auto test2=s0;
    auto test3=d0;
    auto test4=d0;
    auto test7=d0;
    auto test5=p0;
    auto test6=p0;


    double ui = 0.0001;

    int n=1000;
    int m=0;
    for(double u=0.0; u<1.0; u+=ui){
        m++;
    }
    std::cout<<"m="<<m<<std::endl;

    double num=0;

    for(int i=0; i<n; i++){
        //Eigen::Quaterniond q0 = random::quaternionUniform(random::QuaternionSampling::AngleAxis);cout<<"q0="<<q0<<endl;
        //Eigen::Quaterniond q1 = random::quaternionUniform(random::QuaternionSampling::AngleAxis); //poses[1];
        //Eigen::Vector3d v0 = random::vectorUniform();
        //Eigen::Vector3d v1 = random::vectorUniform(100);

        DH1d d0(q0,v0);
        Sophus::SE3d s0(q0,v0);
        R3xSO3 p0(q0,v0);

        DH1d d1(q1,v1);
        Sophus::SE3d s1(q1,v1);
        R3xSO3 p1(q1,v1);

        {
        auto test1=p0;
        auto test2=s0;
        auto test3=d0;
        auto test4=d0;
        auto test5=p0;
        auto test6=p0;
        auto test7=d0;



        //t3.tic();for(double u=0.0; u<1.0; u+=ui)test1*=SPLIT(p0,u,p1);t3.toc("SPLIT_G");
        t3.tic();for(double u=0.0; u<1.0; u+=ui)test6*=SPLIT(p0,u,p1);t3.toc("SPLIT");
        t3.tic();for(double u=0.0; u<1.0; u+=ui)test5*=SPAPP(p0,u,p1);t3.toc("QLB+LERP");
        t3.tic();for(double u=0.0; u<1.0; u+=ui)test2*=SE3Up(s0,u,s1);t3.toc("SE3Up");
        //t3.tic();for(double u=0.0; u<1.0; u+=ui)test3*=ScLERPGeometricCosSin(u,d0,d1);t3.toc("ScLERP_G");
        t3.tic();for(double u=0.0; u<1.0; u+=ui)test7*=ScLERPAnalyticExpLogScrewTangent(d0,u,d1);t3.toc("ScLERP");
        t3.tic();for(double u=0.0; u<1.0; u+=ui)test4*=DLUp(d0,u,d1);t3.toc("DLB");

        num+=rot(test1).dot(rot(test2))-rot(test3).dot(rot(test7))+rot(test5).dot(rot(test4))-rot(test6).dot(rot(test1));
        }

    }
    //t1.printAllTimings("passed",m,"SPLIT_A");
    //t2.printAllTimings("assign",m,"SPLIT_A");
    t3.printAllTimings("multip",m,"SPLIT");

    cout<<"test "<<num<<endl;

    std::cout<<test1.quat.dot(test5.quat)<<std::endl;
    std::cout<<test1.quat.dot(test2.unit_quaternion())<<endl;
    std::cout<<test2.unit_quaternion().dot(test3.rotation())<<std::endl;
    std::cout<<test4.rotation().dot(test3.rotation())<<std::endl;

}

