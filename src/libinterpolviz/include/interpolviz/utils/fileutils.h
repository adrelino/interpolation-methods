/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_FILEUTILS_H
#define INTERPOL_FILEUTILS_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

namespace interpol {

namespace fileutils {

template<typename Number>
void saveVector(const std::string& filename, const std::vector<Number>& vec){

    std::ofstream outputFile(filename, std::ofstream::out) ;

    for (const auto& it : vec){
        outputFile << it <<std::endl;
    }

    outputFile.close( );

    std::cout<<"Wrote "<<vec.size()<<" numbers to "<<filename<<std::endl;
}

template<typename Number>
void loadVector(const std::string& filename, std::vector<Number>& vec){
    std::ifstream file(filename,std::ifstream::in);
    vec.clear();
    if( file.fail() == true )
    {
        std::cerr << filename << " could not be opened" << std::endl;
    }else{
        Number elem;
        while(file >> elem){
            vec.push_back(elem);
        }
    }
}

template<typename Number>
std::vector<Number> loadVector(std::string filename){

    std::vector<Number> vec;
    loadVector(filename,vec);
    return vec;
}

} // ns fileutils

} // ns interpol

#endif // INTERPOL_FILEUTILS_H
