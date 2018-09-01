/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_TIMER_HPP
#define INTERPOL_TIMER_HPP

#include <string>
#include <unordered_map>
#include <chrono>
#include <vector>

namespace interpol {

typedef std::chrono::high_resolution_clock Clock;

class Timer {
    struct Meas{
        std::vector<double> meas;
        double mean;
    };

    private:
        Clock::time_point startTime;
        std::unordered_map<std::string,Meas> timingsMap;

    public:
        void tic();
        void toc(std::string name);
        void printAllTimings(std::string summary="", int numOps=1, std::string relativeTo="");
};

} // ns interpol

#endif // INTERPOL_TIMER_HPP
