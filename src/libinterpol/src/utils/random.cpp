/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#include <interpol/utils/random.hpp>
#include <random>

namespace interpol {

namespace random {
    static std::random_device rd;
    static std::mt19937 e2(rd()); //different sequence on each start
    //static std::mt19937 e2(0);    //same random number sequence on each start
    static std::uniform_real_distribution<> distUni(0, 1);
    static std::normal_distribution<> distNor(0,1);

    double numberUniform(){
        return distUni(e2);
    }

    double numberNormal(){
        return distNor(e2);
    }

    Eigen::Vector3d vectorUniform(double scale) {
        return Eigen::Vector3d(numberUniform(),numberUniform(),numberUniform())*scale;
    }

    Eigen::Vector3d vectorNormal(double sigma) {
        return Eigen::Vector3d(numberNormal(),numberNormal(),numberNormal())*sigma;
    }

    /*
     * Another method uses unit quaternions. Multiplication of rotation matrices is homomorphic to multiplication
     * of quaternions, and multiplication by a unit quaternion rotates the unit sphere. Since the homomorphism
     * is a local isometry, we immediately conclude that to produce a uniform distribution on SO(3) we may use
     * a uniform distribution on S3. In practice: create a 4 element vector where each element is a sampling
     * of a normal distribution. Normalize its length and you have a uniformly sampled random unit quaternion
     * which represents a uniformly sampled random rotation.
     * 
     * https://en.wikipedia.org/wiki/Rotation_matrix#Uniform_random_rotation_matrices
     */
    Eigen::Quaterniond quaternionUniform(){
        Eigen::Quaterniond q(numberNormal(),numberNormal(),numberNormal(),numberNormal());
        return q.normalized();
    }
}

}
