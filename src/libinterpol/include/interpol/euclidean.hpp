/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_EUCLIDEAN_HPP
#define INTERPOL_EUCLIDEAN_HPP

namespace interpol {

template <typename T>
T LERP(const T& lhs, double u, const T& rhs) {
    return lhs*(1-u) + rhs*u;
}

} // ns interpol

#endif // INTERPOL_EUCLIDEAN_HPP
