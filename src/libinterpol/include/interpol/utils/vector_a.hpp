/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_VECTOR_A_HPP
#define INTERPOL_VECTOR_A_HPP

#include <Eigen/StdVector>
#include <iostream>

namespace interpol {

template <typename T>
using vector_a = std::vector<T,Eigen::aligned_allocator<T>>;

static void __assertCustom(bool cond, std::string filename,int line,std::string condText){
    if(!cond){
        std::cout <<std::endl<< filename << ":" << line << " assertion failed : " << condText <<std::endl;
        exit(EXIT_FAILURE);
    }
}

#define assertCustom(cond) __assertCustom(cond, __FILE__, __LINE__, #cond)

} // ns interpol

#endif // INTERPOL_VECTOR_A_HPP
