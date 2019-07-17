#include <vector>
#include <unordered_map>

#include "Unit.h"
#include "Event.h"
#include "Boxes.h"
#include "simulator.h"
#include "Container.h"

class Environment;

Boxes::Boxes() {
	for (int i = 0; i < global_num_units; i++) {
		for (int j = 0; j < global_num_units; j++) {
			m_present_collisions[i][j] = false;
		}
	}

	//int containerSize = globalContainer.get_size();

	int leftX = globalContainer.get_leftX();
	int bottomY = globalContainer.get_bottomY();
	int farZ = globalContainer.get_farZ();
	int rightX = globalContainer.get_rightX();
	int topY = globalContainer.get_topY();
	int closeZ = globalContainer.get_closeZ();

	for (int i = 0; i < m_boxes_per_side[cX]; i++) {
		for (int j = 0; j < m_boxes_per_side[cY]; j++) {
			for (int k = 0; k < m_boxes_per_side[cZ]; k++) {
				m_planes[i][j][k][cX][0] = { cX, (float)leftX + (i + 1)*m_box_size[cX] };
				m_planes[i][j][k][cX][1] = { cX, (float)leftX + i * m_box_size[cX] };
				m_planes[i][j][k][cY][0] = { cY, (float)bottomY + (j + 1) * m_box_size[cY] };
				m_planes[i][j][k][cY][1] = { cY, (float)bottomY + j * m_box_size[cY] };
				m_planes[i][j][k][cZ][0] = { cZ, (float)farZ + (k + 1) * m_box_size[cZ] };
				m_planes[i][j][k][cZ][1] = { cZ, (float)farZ + k * m_box_size[cZ] };
				for (int a = 0; a < global_num_units; a++) {
					m_unit_membership[i][j][k][a] = false;
				}
			}
		}
	}
}

Boxes::~Boxes()
{
	//if using malloc to initialize arrays, free here!
}

//returns a vector of all 8 boxes (including current box) in the path of the given unit
void Boxes::get_future_boxes(Unit& unit, Box (&arr)[8]) {
	Box myBox = get_box(unit);
	int xIndex = myBox.positions[cX];
	int yIndex = myBox.positions[cY];
	int zIndex = myBox.positions[cZ];
	
	int xDir = -1;
	int yDir = -1;
	int zDir = -1;

	if (unit.get_direction()[cX] >= 0) {
		xDir = 1;
	}
	if (unit.get_direction()[cY] >= 0) {
		yDir = 1;
	}
	if (unit.get_direction()[cZ] >= 0) {
		zDir = 1;
	}
	if (xIndex == 0 || xIndex == m_boxes_per_side[cX] - 1) {
		xDir = 0;
	}
	if (yIndex == 0 || yIndex == m_boxes_per_side[cY] - 1) {
		yDir = 0;
	}
	if (zIndex == 0 || zIndex == m_boxes_per_side[cZ] - 1) {
		zDir = 0;
	}
	arr[0] = { xIndex, yIndex, zIndex }; 
	arr[1] = { xIndex + xDir, yIndex, zIndex };
	arr[2] = { xIndex + xDir, yIndex + yDir, zIndex };
	arr[3] = { xIndex + xDir, yIndex, zIndex + zDir };
	arr[4] = { xIndex + xDir, yIndex + yDir, zIndex +zDir};
	arr[5] = { xIndex, yIndex + yDir, zIndex};
	arr[6] = { xIndex, yIndex + yDir, zIndex + zDir};
	arr[7] = { xIndex, yIndex, zIndex + zDir};
}

void Boxes::add_collisions(Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator> &pq) {
	//get IDs of all eligible units
	vector<int> neighbors;
	Box possible_boxes[8];
	get_future_boxes(unit, possible_boxes);
	for (Box box : possible_boxes) {
		for (int a = 0; a < global_num_units; a++) {
			if (m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][a]) {
				if (a != unit.get_id()) {
					generate_collision_event(unit, a, pq);
				}
			}
		}
	}
}

//returns time in milliseconds until the given unit hits the given plane
float Boxes::time_to_plane(Unit& unit, Plane plane) {
	//find required vectors
	myVector planeNormal{};
	myVector planePoint{};
	get_plane_info(plane, planeNormal, planePoint);
	myVector directionVec = { unit.get_direction()[cX], unit.get_direction()[cY],  unit.get_direction()[cZ] };
	myVector location = { unit.get_location()[cX], unit.get_location()[cY], unit.get_location()[cZ] };
	myVector dirNormalized = directionVec.normalize();

	//if unit is traveling parallel to given plane
	if (planeNormal.dot_product(dirNormalized) == 0) {
		return std::numeric_limits<float>::infinity();
	}

	float t = (planeNormal.dot_product(planePoint)-planeNormal.dot_product(location)) / (planeNormal.dot_product(dirNormalized));
	return t;
}

void Boxes::get_plane_info(Plane plane, myVector &normal, myVector &point) {
	if (plane.coordinate == cX) {
		normal.vals[cX] = 1;
		normal.vals[cY] = 0;
		normal.vals[cZ] = 0;
		point = { plane.offset, 0, 0 };
	}
	if (plane.coordinate == cY) {
		normal.vals[cX] = 0;
		normal.vals[cY] = 1;
		normal.vals[cZ] = 0;
		point = { 0, plane.offset, 0 };
	}
	if (plane.coordinate == cZ) {
		normal.vals[cX] = 0;
		normal.vals[cY] = 0;
		normal.vals[cZ] = 1;
		point = { 0, 0, plane.offset};
	}
}

Event Boxes::get_next_box_event(Unit& unit) {
	int unitID = unit.get_id();

	//from unit, get candidate planes
	Box prevBox = get_box(unit);
	//if (prevBox.positions[cX] == 9) {
		//int val = 0;
	//}
	Plane right, left, top, bottom, nearPlane, farPlane;

	right = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cX][0];
	left = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cX][1];
	top = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cY][0];
	bottom = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cY][1];
	nearPlane = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cZ][0];
	farPlane = m_planes[prevBox.positions[cX]][prevBox.positions[cY]][prevBox.positions[cZ]][cZ][1];

	float min = INFINITY;
	Plane currPlane;
	Plane closestPlane;
	//eliminate planes in opposite direction
	if (unit.get_direction()[cX] >= 0) {
		currPlane = right;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = right;
			min = time;
		}
	}
	if (unit.get_direction()[cX] < 0) {
		currPlane = left;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = left;
			min = time;
		}
	}
	if (unit.get_direction()[cY] >= 0) {
		currPlane = top;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = top;
			min = time;
		}
	}
	if (unit.get_direction()[cY] < 0) {
		currPlane = bottom;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = bottom;
			min = time;
		}
	}
	if (unit.get_direction()[cZ] >= 0) {
		currPlane = nearPlane;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = nearPlane;
			min = time;
		}
	}
	if (unit.get_direction()[cZ] < 0) {
		currPlane = farPlane;
		float time = time_to_plane(unit, currPlane);
		if (time < min) {
			closestPlane = farPlane;
			min = time;
		}
	}
	bool containerCollision = false;
	
	//container collision in x or y directions
	if (closestPlane.coordinate == cX || closestPlane.coordinate == cY) {
		if (closestPlane.offset == globalContainer.get_leftX() || closestPlane.offset == globalContainer.get_rightX()) {
			containerCollision = true;
		}
	}

	//container collision in z direction
	if (closestPlane.coordinate == cZ) {
		if (closestPlane.offset == globalContainer.get_closeZ() || closestPlane.offset == globalContainer.get_farZ()) {
			containerCollision = true;
		}
	}

	float intersectionTime = unit.calc_intersection_time(min);
	//boxEvent data = { unitID, unit.get_age(), prevBox, containerCollision };
	Event event = { BOX_EVENT, unitID, unit.get_age(), 0, intersectionTime, prevBox, containerCollision };
	return event;
}

std::vector<int> Boxes::get_units_in_box(Unit& unit) {
	Box currBox = get_box(unit);
	vector<int> neighborIDs;
	for (int i = 0; i < global_num_units; i++) {
		if (m_unit_membership[currBox.positions[cX]][currBox.positions[cY]][currBox.positions[cZ]][i]) {
			neighborIDs.emplace_back(i);
		}
	}
	return neighborIDs;
}

/*std::vector<int> Boxes::get_units_in_box(Box box, vector<int>& vector_of_IDs) {
	vector<int> neighborIDs;
	for (int a = 0; a < global_num_units; a++) {
		if (m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][a]) {
			neighborIDs.emplace_back(a);
			//vector_of_IDs.emplace_back(a);
		}
	}
	return neighborIDs;
}*/
