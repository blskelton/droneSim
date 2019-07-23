#include <vector>
#include <unordered_map>
#include <map>

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

	m_bounding_planes[0] = Plane{ 0, (float)leftX };
	m_bounding_planes[1] = Plane{ 0, (float)rightX };
	m_bounding_planes[2] = Plane{ 1, (float)bottomY };
	m_bounding_planes[3] = Plane{ 1, (float)topY };
	m_bounding_planes[4] = Plane{ 2, (float)closeZ };
	m_bounding_planes[5] = Plane{ 2, (float)farZ };

	for (int i = 0; i < m_boxes_per_side[cX]; i++) {
		for (int j = 0; j < m_boxes_per_side[cY]; j++) {
			for (int k = 0; k < m_boxes_per_side[cZ]; k++) {
				std::array<Plane, 6> planes;
				planes[0] = { cX, (float)leftX + (i + 1)*m_box_size[cX] };
				planes[1] = { cX, (float)leftX + i * m_box_size[cX] };
				planes[2] = { cY, (float)bottomY + (j + 1) * m_box_size[cY] };
				planes[3] = { cY, (float)bottomY + j * m_box_size[cY] };
				planes[4] = { cZ, (float)farZ + (k + 1) * m_box_size[cZ] };
				planes[5] = { cZ, (float)farZ + k * m_box_size[cZ] };
				std::array<int, 3> position_array = { i, j, k};
				m_plane_map[position_array] = planes;

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
	return abs(t);
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

Event Boxes::get_next_non_container_box_event(Unit& unit, Box& box) {

	Plane right, left, top, bottom, nearPlane, farPlane;
	std::array<int, 3> position_array = { box.positions[0], box.positions[1],box.positions[2] };
	if (m_plane_map.find(position_array) == m_plane_map.end()) { //box not in map
		int leftX = globalContainer.get_leftX();
		int bottomY = globalContainer.get_bottomY();
		int farZ = globalContainer.get_farZ();
		int rightX = globalContainer.get_rightX();
		int topY = globalContainer.get_topY();
		int closeZ = globalContainer.get_closeZ();

		right = { cX, (float)leftX + (box.positions[cX] + 1)*m_box_size[cX] };
		left = { cX, (float)leftX + (box.positions[cX])*m_box_size[cX] };
		top = { cY, (float)bottomY + (box.positions[cY] + 1)*m_box_size[cY] };
		bottom = { cY, (float)bottomY + (box.positions[cY])*m_box_size[cY] };
		nearPlane = { cZ, (float)farZ + (box.positions[cZ] + 1)*m_box_size[cZ] };
		farPlane = { cZ, (float)farZ + (box.positions[cZ])*m_box_size[cZ] };

		std::array<Plane, 6> plane_array = { right, left, top, bottom, nearPlane, farPlane };
		m_plane_map[position_array] = plane_array;
	}
	else { //box is in map
		right = m_plane_map[position_array][0];
		left = m_plane_map[position_array][1];
		top = m_plane_map[position_array][2];
		bottom = m_plane_map[position_array][3];
		nearPlane = m_plane_map[position_array][4];
		farPlane = m_plane_map[position_array][5];
	}
	

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

	for (Plane plane : m_bounding_planes) {
		if (closestPlane.equals(plane)) {
			containerCollision = true;
		}
	}

	float intersectionTime = unit.calc_intersection_time(min);
	BoxEvent data = { unit.get_id(), unit.get_age(), box, containerCollision };
	Event event = { BOX_EVENT, intersectionTime, {data} };
	return event;

}

Event Boxes::get_next_box_event(Unit& unit) {
	int unitID = unit.get_id();

	//from unit, get candidate planes
	Box prevBox = get_box(unit);
	std::array<int, 3> position_array = { prevBox.positions[0], prevBox.positions[1],prevBox.positions[2] };

	Plane right, left, top, bottom, nearPlane, farPlane;

	right = m_plane_map[position_array][0];
	left = m_plane_map[position_array][1];
	top = m_plane_map[position_array][2];
	bottom = m_plane_map[position_array][3];
	nearPlane = m_plane_map[position_array][4];
	farPlane = m_plane_map[position_array][5];

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

	for (Plane plane : m_bounding_planes) {
		if (closestPlane.equals(plane)) {
			containerCollision = true;
		}
	}

	float intersectionTime = unit.calc_intersection_time(min);
	BoxEvent data = { unitID, unit.get_age(), prevBox, containerCollision };
	Event event = { BOX_EVENT, intersectionTime, {data} };
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
