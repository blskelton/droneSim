/*<DroneSim - a simulator graphically modeling drone activity in real time.>
	Copyright(C) < 2019 > <Blake Skelton>

	This program is free software : you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see < https://www.gnu.org/licenses/>. */

#include <cmath>
#include "simulator.h"

#ifndef Geometry_h
#define Geometry_h

//coordinate plane defined by the coordinate it is in and its offset in that direction
struct Plane {
	int coordinate;
	float offset;

	//plane test for equality
	bool equals(Plane secondPlane) {
		if (coordinate == secondPlane.coordinate && offset == secondPlane.offset) {
			return true;
		}
		return false;
	}
};


//get other location
/*float x = (m_x - site_x);
float y = (m_y - site_y);
float z = (m_z - site_z);

//use distance formula
float distance = sqrt(abs(x*x + y * y + z * z));*/




//sub-box for collision detection efficiency
struct Box {
	//box indices
	int positions[3];
	bool in_container;
	//constructor if container status is unknown
	Box(int xPos, int yPos, int zPos, int boundaries[6]) {
		positions[cX] = xPos;
		positions[cY] = yPos;
		positions[cZ] = zPos;
		in_container = true;
		if (xPos<boundaries[0] || xPos>boundaries[1]) {
			in_container = false;
		}
		if (yPos<boundaries[2] || yPos>boundaries[3]) {
			in_container = false;
		}
		if (zPos<boundaries[4] || zPos>boundaries[5]) {
			in_container = false;
		}
	}
	//constructor if container status is known
	Box(int xPos, int yPos, int zPos, bool container_bool) {
		positions[cX] = xPos;
		positions[cY] = yPos;
		positions[cZ] = zPos;
		in_container = container_bool;
	}
	Box() {
	}
};

//box comparator for hashmap
inline bool operator==(const Box& first, const Box& second) {
	return (first.positions[cX] == second.positions[cX] && first.positions[cY] == second.positions[cY] && first.positions[cZ] == second.positions[cZ]);
}

namespace std {
	template<>
	struct hash<Box> {
		std::size_t operator() (const Box &box) const
		{
			std::size_t h1 = std::hash<int>{}(box.positions[0]);
			std::size_t h2 = std::hash<int>{}(box.positions[1]);
			std::size_t h3 = std::hash<int>{}(box.positions[2]);
			return h1 ^ h2 ^ h3;
		}
	};
}

//vector with basic vector operations
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