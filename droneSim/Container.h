#include <cmath>
#include <boost/geometry.hpp>
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
	//static constexpr int  GLOBAL_SIZE = 20;
	static constexpr int CONTAINER_X = 20;
	static constexpr int CONTAINER_Y = 20;
	static constexpr int CONTAINER_Z = 20;
	Container();

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

	void draw_container();
};
#endif /* Container_h */