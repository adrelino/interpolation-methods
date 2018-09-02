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
    //static std::mt19937 e2(rd()); //different sequence on each start
    static std::mt19937 e2(1);    //same random number sequence on each start
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

    /**
     * Convert Spherical=(r,θ,φ) to Cartesian=(x,y,z) Coordinates
     *
     * (radius r, inclination θ (theta), azimuth φ (phi), where r ∈ [0, ∞), θ ∈ [0, π], φ ∈ [0, 2π)
     *
     * https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
     */
    Eigen::Vector3d sphericalToCartesian(double r, double theta, double phi){
        double x = r * sin( theta) * cos( phi );
        double y = r * sin( theta) * sin( phi );
        double z = r * cos( theta );
        return Eigen::Vector3d(x,y,z);
    }

    /**
     * Uniform Random Distribution inside a Unit Sphere
     *
     * http://stackoverflow.com/questions/5408276/sampling-uniformly-distributed-random-points-inside-a-spherical-volume
     * http://mathproofs.blogspot.com/2005/05/uniformly-distributed-random-unit.html
     */
    Eigen::Vector3d vectorUniformInUnitSphere() {
        double r = std::cbrt(numberUniform()); //cubic root since radius grows to the³ (second link above)
        double theta = acos(1-2*numberUniform());
        double phi = numberUniform()*2*M_PI;
        return sphericalToCartesian(r,theta,phi);
    }


    /**
     * Uniform Random Distribution on a Sphere
     *
     * http://mathproofs.blogspot.com/2005/04/uniform-random-distribution-on-sphere.html
     * http://corysimon.github.io/articles/uniformdistn-on-sphere/
     */
    Eigen::Vector3d vectorUniformOnUnitSphere() {
        double r = 1;
        double theta = acos(1-2*numberUniform());
        double phi = numberUniform()*2*M_PI;
        return sphericalToCartesian(r,theta,phi);
    }


    /**
     * Uniformly Distributed Random Unit Quaternions
     * using Uniform Random Distribution on a Sphere
     *
     * http://mathproofs.blogspot.com/2005/05/uniformly-distributed-random-unit.html
     */
    Eigen::Quaterniond quaternionUniformFromHypersphere() {
        double t2 = 0.5*(M_PI*numberUniform()+asin(numberUniform())+M_PI_2);
        Eigen::Quaterniond q;
        q.vec() = vectorUniformOnUnitSphere()*sin(t2);
        q.w() = cos(t2);
        return q.normalized();//Norm is 1 even before normalization, but withouth, SLERP takes 300ns instead of 130
    }

    /**
     * Rotation matrices can be uniquely defined by a vector and a rotation angle.
     * To generate the vector, you can use random spherical coordinates ϕ and θ.
     *
     * https://math.stackexchange.com/questions/442418/random-generation-of-rotation-matrices
     */
    Eigen::Quaterniond quaternionUniformFromAngleAxis() {
        Eigen::Vector3d axis = vectorUniformOnUnitSphere();//vectorUniformInUnitSphere().normalized();
        double angle = numberUniform()*2*M_PI;
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).normalized();//Norm is 1 even before normalization, but withouth, SLERP takes 300ns instead of 130
    }

    /**
     * Generating a random element of SO(3) according to
     *
     * K. Shoemake.
     * Uniform random rotations.
     * In D. Kirk, editor, Graphics Gems III, pages 124-132. Academic, New York, 1992.
     *
     * Choose three points $ u_1,u_2,u_3 \in [0,1]$ uniformly at random. A uniform, random quaternion is given by the simple expression
     * $(\sqrt{1-u_1}\sin 2 \pi u_2,\; \sqrt{1-u_1}\cos 2 \pi u_2,\; \sqrt{u_1}\sin 2 \pi u_3,\; \sqrt{u_1}\cos 2 \pi u_3)$ (5.15)
     * The method above is used to provide a random point on $ {\mathbb{S}}^2$ using $ u_2$ and $ u_3$, and $ u_1$ produces
     * a random point on $ {\mathbb{S}}^1$; they are carefully combined in (5.15) to yield a random rotation.
     *
     * https://bitbucket.org/eigen/eigen/src/3.3.0/Eigen/src/Geometry/Quaternion.h#lines-619
     * http://planning.cs.uiuc.edu/node198.html
     */
    Eigen::Quaterniond quaternionUniformFromShoemake(){
        return Eigen::Quaterniond::UnitRandom().normalized();//Norm is 1 even before normalization, but withouth, SLERP takes 300ns instead of 130
    }

    /**
     * Another method uses unit quaternions. Multiplication of rotation matrices is homomorphic to multiplication
     * of quaternions, and multiplication by a unit quaternion rotates the unit sphere. Since the homomorphism
     * is a local isometry, we immediately conclude that to produce a uniform distribution on SO(3) we may use
     * a uniform distribution on S3. In practice: create a 4 element vector where each element is a sampling
     * of a normal distribution. Normalize its length and you have a uniformly sampled random unit quaternion
     * which represents a uniformly sampled random rotation.
     * 
     * https://en.wikipedia.org/wiki/Rotation_matrix#Uniform_random_rotation_matrices
     * http://corysimon.github.io/articles/uniformdistn-on-sphere/
     */
    Eigen::Quaterniond quaternionUniformFromNormalDistribution(){
        Eigen::Quaterniond q(numberNormal(),numberNormal(),numberNormal(),numberNormal());
        //while(q.norm() < 0.01){
        //    q=Eigen::Quaterniond(numberNormal(),numberNormal(),numberNormal(),numberNormal());
        //}
        return q.normalized();//Norm is 1 even before normalization, AND NO MATTER IF WITH OR withouth, SLERP takes 300ns instead of 130 as in the other 3 methods above.
    }

    Eigen::Quaterniond quaternionUniform(QuaternionSampling method){
        switch(method){
            case QuaternionSampling::Hypersphere : return quaternionUniformFromHypersphere(); break;
            case QuaternionSampling::AngleAxis : return quaternionUniformFromAngleAxis(); break;
            case QuaternionSampling::Shoemake : return quaternionUniformFromShoemake(); break;
            case QuaternionSampling::NormalDistribution : return quaternionUniformFromNormalDistribution(); break;
        }
    }
}

}
