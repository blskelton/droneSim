#include <cmath>
#include "simulator.h"

#ifndef Geometry_h
#define Geometry_h

struct Plane {
	int coordinate;
	float offset;

	bool equals(Plane secondPlane) {
		if (coordinate == secondPlane.coordinate && offset == secondPlane.offset) {
			return true;
		}
		return false;
	}
};

struct Box {
	int positions[3];
	bool in_container;
};

/*struct box_hash {
	template<class T1, class T2, class T3, class T4> 
	std::size_t operator() (const Box(T1, T2, T3, T4) &box) const
	{
		std::size_t h1 = std::hash<T1>()(box.positions[0]);
		std::size_t h2 = std::hash<T2>()(box.positions[1]);
		std::size_t h3 = std::hash<T3>()(box.positions[2]);
		return h1 ^ h2 ^ h3; 
	}
};*/

struct myVector {
	float vals[3];
	
	myVector dif(myVector secondVec) {
		myVector result;
		result.vals[cX] = vals[cX] - secondVec.vals[cX];
		result.vals[cY] = vals[cY] - secondVec.vals[cY];
		result.vals[cZ] = vals[cZ] - secondVec.vals[cZ];
		return result;
	}

	float dot_product(myVector secondVec) {
		float dot = 0.0;
		dot += (vals[cX] * secondVec.vals[cX]);
		dot += (vals[cY] * secondVec.vals[cY]);
		dot += (vals[cZ] * secondVec.vals[cZ]);
		return dot;
	}

	myVector scalar_mult(float x) {
		myVector result = { vals[cX] * x , vals[cY] * x , vals[cZ] * x };
		return result;
	}

	float magnitude() {
		return sqrt(pow(vals[cX],2)+pow(vals[cY],2)+pow(vals[cZ],2));
	}

	myVector normalize() {
		float mag = magnitude();
		if (mag != 0) {
			return scalar_mult(1 / mag);
		}
		else return *this;
	}

	myVector add(myVector secondVec) {
		myVector result = {vals[cX]+secondVec.vals[cX],vals[cY] + secondVec.vals[cY],vals[cZ] + secondVec.vals[cZ] };
		return result;
	}

	float* to_array() {
		float result[3];
		result[cX] = vals[cX];
		result[cY] = vals[cY];
		result[cZ] = vals[cZ];
		return result;
	}

	myVector from_array(float* arr) {
		myVector result = { arr[cX], arr[cY], arr[cZ]};
		return result;
	}
};

#endif /*Geometry_h */