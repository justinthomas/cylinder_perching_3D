/*
 * vectorTypes.hpp
 *
 *  Created on: Jan 3, 2014
 *      Author: tron
 */

#ifndef VECTORTYPES_HPP_
#define VECTORTYPES_HPP_
#include <TooN/TooN.h>

typedef TooN::Vector<2> Vec2;
typedef std::vector<Vec2> VVec2;
typedef TooN::Vector<3> Vec3;
typedef std::vector<Vec3> VVec3;
typedef std::vector<double> Vdouble;
typedef TooN::Matrix<2,2> Mat2;

//compute norm of a vector
template <typename Vec> double norm(Vec &v) {
	return TooN::norm_2(v);
}

//set all the entries of the vector to zero
template <typename Vec> void setZero(Vec &v) {
	v=TooN::Zeros;
}

//compute the inner product between two vectors
template <typename Vec> double innerProd(Vec &v1, Vec &v2) {
	return v1*v2;
}

#endif /* VECTORTYPES_HPP_ */
