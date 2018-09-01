/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include <interpol/utils/timer.hpp>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <iostream>

namespace interpol{
  
void Timer::tic() {
    startTime = Clock::now();
}

void Timer::toc(std::string name){
    Clock::time_point endTime = Clock::now();
    auto s = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
    //std::cout<<s.count()<<std::endl;
    double ns = s.count();// * 1e-9; //1e6 microseconds is 1s, 1e9 nanoseconds
    //  if(print){
    //  cout<<endl;
    //  cout<<"=====  TIMING["<<name<<"] is ";
    //  cout<< setw(20) << std::setprecision(10)<< seconds << " s" << endl;
    //  cout << endl;
    //  }
    timingsMap[name].meas.push_back(ns);
}

void Timer::printAllTimings(std::string summary, int numOps, std::string relativeTo){
    std::cout<<"=====  TIMINGS ===== "<<summary<<" ====="<<" ops="<<numOps<<std::endl;
    //cout<<"clock is steady:"<<Clock::is_steady<<endl;

    double max = 0;
    double min = 999999999999.9;
    std::vector<double> means;

    std::vector<std::pair<std::string, double>> pairs;

    for (auto& it : timingsMap){
        std::vector<double>& vec = it.second.meas;
        //cout<<"mean of"<<vec.size()<<" measurements"<<endl;
        //cout<<"value "<<vec[0]<<" ";
        double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
        //cout<<"sum "<<vec[0]<<endl;

        double mean = sum / vec.size();
        if (mean > max) max = mean;
        if (mean < min) min = mean;
        it.second.mean = mean;

        pairs.push_back({it.first,mean});
    }
    //cout<<"max is"<<max<<endl;

    auto cmp = [](std::pair<std::string,double> const & a, std::pair<std::string,double> const & b)
    {
            return a.second < b.second;
    };
    std::sort(pairs.begin(), pairs.end(), cmp);

    std::cout<<std::left<<std::setw(10)<<"name"<<"\t";
    printf("nsec\t");
    printf("max \t");
    printf("min \t");
    if(relativeTo.length()){
        std::cout<<relativeTo<<"\t";
    }
    printf("#means");
    std::cout<<std::endl;


    for(auto it : pairs) {
    std::cout<<std::left<<std::setw(10)<<it.first<<"\t";
    printf("%0.2f \t",(it.second/numOps));
    printf("%0.2f \t",(it.second/max));
    printf("%0.2f \t",(it.second/min));
    if(relativeTo.length()){
        printf("%0.2f\t",(it.second/timingsMap[relativeTo].mean));
    }
    printf("%d\t",(int)(timingsMap[it.first].meas.size()));
    std::cout<<std::endl;

    }
}

} //ns interpol
