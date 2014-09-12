/*
 * Functions to input/output vectors
 * Note: they use streams, iterators and the copy function.
 * Advantages of this approach:
 * - you can change implementation of points (e.g., using p.x, p.y instead of p.v[0], p.v[1])
 *   without changing the function (but you will have to change the implementation of the << and >> operators)
 * - automatic check of end of file (robust to incomplete files)
 * - compact code
 */

#ifndef VECTORUTILITIES_HPP_
#define VECTORUTILITIES_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iterator>

using namespace std;

template <typename type> bool loadScalarFromFile(type &v, const char *fileName) {
	ifstream inputFile(fileName);
	//IO of vectors
	if (!inputFile.is_open()) {
		cerr << "Error opening file " << fileName << endl;
		return false;
	} else {
		inputFile >> v;

		inputFile.close();
		cout << "Read file " << fileName << endl;
		return true;
	}

}

template <class type> bool loadVectorFromFile(vector<type> &v, const char *fileName) {
	ifstream inputFile(fileName);
	//IO of vectors
	if (!inputFile.is_open()) {
		cerr << "Error opening file " << fileName << endl;
		return false;
	} else {
		type					element;

		while (inputFile >> element) {
			v.push_back(element);
		}

		////For some reason the following does not compile
		//copy(istream_iterator<type>(inputFile),
		//istream_iterator<type>(),
		//back_inserter(v));

		inputFile.close();
		cout << "Read file " << fileName << endl;
		return true;
	}

}

template <class type> bool saveVectorToFile(vector<type> &v, const char *fileName) {
	ofstream outputFile(fileName);
	//IO of vectors
	if (!outputFile.is_open()) {
		cerr << "Error opening file " << fileName << endl;
		return false;
	} else {
		copy(v.begin(),
			 v.end(),
			 ostream_iterator<type>(outputFile, "\n"));

		outputFile.close();
		cout << "Wrote file " << fileName << endl;
		return true;
	}

}


/* Sends vector to output stream. Requires that the base class T has operator<< implemented */
template<typename type> ostream &operator<<(ostream &out, const vector<type> v) {
	copy(v.begin(),
		v.end(),
		ostream_iterator<type>(cout, "\n"));
	return out;
}


/* Apply transform from the beginning to the end of vectors. It also checks that the destination has enough space. */
template <typename typeSource, typename typeDest, typename UnaryOperator> vector<typeDest> &transformVector (vector<typeSource> &source, vector<typeDest> &dest, UnaryOperator op) {
	size_t vecLength=source.size();
	if (dest.size()<vecLength) {
		dest.resize(vecLength);
	}
	transform(source.begin(), source.end(), dest.begin(), op);
	return dest;
}

/* Apply transform from the beginning to the end of vectors. It also checks that the destination has enough space. */
template <class typeSource1, class typeSource2, class typeDest, class BinaryOperator> vector<typeDest> &transformVector (vector<typeSource1> &source1, vector<typeSource2> &source2, vector<typeDest> &dest, BinaryOperator op) {
	size_t vecLength=source1.size();
	if (dest.size()<vecLength) {
		dest.resize(vecLength);
	}
	transform(source1.begin(), source1.end(), source2.begin(), dest.begin(), op);
	return dest;
}

/* Apply accumulate from the beginning to the end of the vector. */
template <class typeSource, class typeDest, class BinaryOperator> typeDest &accumulateVector (vector<typeSource> &source1, typeDest &dest, BinaryOperator op) {
	accumulate(source1.begin(), source1.end(), dest, op);
	return dest;
}

#endif /* VECTORUTILITIES_HPP_ */
