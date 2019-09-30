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
#include <cstdlib>
#include <array>

#ifndef Container_h
#define Container_h

class Container {
private:
	//boundaries and dimensions
	int m_Xdimension;
	int m_Ydimension;
	int m_Zdimension;
	int m_rightX;
	int m_leftX;
	int m_topY;
	int m_bottomY;
	int m_closeZ;
	int m_farZ;

public:
	//container dimensions as constant expressions
	static constexpr int CONTAINER_X = 20;
	static constexpr int CONTAINER_Y = 20;
	static constexpr int CONTAINER_Z = 20;

	Container();
	//methods to access container boundaries and dimensions
	inline int get_rightX() {
		return m_rightX;
	};

	inline int get_leftX() {
		return m_leftX;
	};

	inline int get_bottomY() {
		return m_bottomY;
	};

	inline int get_topY() {
		return m_topY;
	};

	inline int get_closeZ() {
		return m_closeZ;
	};

	inline int get_farZ() {
		return m_farZ;
	};

	inline int get_z_dimension() {
		return m_Zdimension;
	}

	inline int get_y_dimension() {
		return m_Ydimension;
	}

	inline int get_x_dimension() {
		return m_Xdimension;
	}

	//makes gl calls to draw container
	void draw_container();
};
#endif /* Container_h */