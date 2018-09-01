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
#include <interpol/utils/random.hpp>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace interpol;

int main(int argc, char* argv[]){

    Timer t1;

    Eigen::Quaterniond q0 = random::quaternionUniform();
    Eigen::Quaterniond q1 = random::quaternionUniform();

    cout<<"q0="<<q0<<endl;
    cout<<"q1="<<q1<<endl;


    for(double u=0.0; u<=1.0; u+=0.1){
        auto exact  = SLERP(q0,u,q1);
        auto approx = QLB(q0,u,q1);
        cout<<setprecision(1)<<"u="<<u<<"\t"<<setprecision(4)<<"diff="<<exact.dot(approx)<<endl;
    }


    int n=1000;
    int m=1000;

    Eigen::Quaterniond test1=q0;
    Eigen::Quaterniond test2=q0;

    for(int i=0; i<n; i++){
        double u = (rand() % 1000)/1000.0;
        q0 = random::quaternionUniform();
        q1 = random::quaternionUniform();
        //cout<<"q0="<<q0<<endl;
        //cout<<"q1="<<q1<<endl;

        t1.tic();for(int i=0; i<m; i++){test1*=QLB(q0,u,q1);}t1.toc("QLB");
        t1.tic();for(int i=0; i<m; i++){test2*=SLERP(q0,u,q1);}t1.toc("SLERP");
    }
    t1.printAllTimings("orientation",m,"SLERP");
    std::cout<<test1.dot(test2)<<std::endl;
}
