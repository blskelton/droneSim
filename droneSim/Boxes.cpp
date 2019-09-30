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

#include <vector>
#include <unordered_map>
#include <map>

#include "Unit.h"
#include "Event.h"
#include "Boxes.h"
#include "simulator.h"
#include "Container.h"

class Environment;

Boxes::Boxes() 
{
	for (int i = 0; i < global_num_units; i++) {
		for (int j = 0; j < global_num_units; j++) {
			m_present_collisions[i][j] = -1;
		}
	}

	int leftX = globalContainer.get_leftX();
	int bottomY = globalContainer.get_bottomY();
	int farZ = globalContainer.get_farZ();
	int rightX = globalContainer.get_rightX();
	int topY = globalContainer.get_topY();
	int closeZ = globalContainer.get_closeZ();

	m_bounding_planes[0] = Plane{ 0, (float)globalContainer.get_leftX() };
	m_bounding_planes[1] = Plane{ 0, (float)rightX };
	m_bounding_planes[2] = Plane{ 1, (float)bottomY };
	m_bounding_planes[3] = Plane{ 1, (float)topY };
	m_bounding_planes[4] = Plane{ 2, (float)closeZ };
	m_bounding_planes[5] = Plane{ 2, (float)farZ };

	for (int i = 0; i < m_boxes_per_side[cX]; i++) {
		for (int j = 0; j < m_boxes_per_side[cY]; j++) {
			for (int k = 0; k < m_boxes_per_side[cZ]; k++) {
				m_planes[i][j][k][0] = { cX, (float)leftX + (i + 1)*m_box_size[cX] };
				m_planes[i][j][k][1] = { cX, (float)leftX + i * m_box_size[cX] };
				m_planes[i][j][k][2] = { cY, (float)bottomY + (j + 1) * m_box_size[cY] };
				m_planes[i][j][k][3] = { cY, (float)bottomY + j * m_box_size[cY] };
				m_planes[i][j][k][4] = { cZ, (float)farZ + (k + 1) * m_box_size[cZ] };
				m_planes[i][j][k][5] = { cZ, (float)farZ + k * m_box_size[cZ] };

				m_boxes[i][j][k] = Box{ i,j,k, true };

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

//returns a vector of all boxes (including current box) in the path of the given unit
void Boxes::get_future_boxes(Unit& unit, Box (&arr)[27]) {
	Box& myBox = get_box(unit);
	int xIndex = myBox.positions[cX];
	int yIndex = myBox.positions[cY];
	int zIndex = myBox.positions[cZ];

	int boundaries[6] = { 0, m_boxes_per_side[cX] - 1, 0, m_boxes_per_side[cY] - 1, 0, m_boxes_per_side[cZ] - 1 };

	vector<Box> boxes;
	int counter = 0;

	for (int x = -1; x < 2; x++) {
		for (int y = -1; y < 2; y++) {
			for (int z = -1; z < 2; z++) {
				Box box = { xIndex + x, yIndex + y, zIndex+z, boundaries };
				arr[counter] = box;
				counter++;
			}
		}
	}
}

bool Boxes::has_neighbors(Unit& unit) {
	Box box = get_box(unit);
	int my_id = unit.get_id();
	if (box.in_container) {
		for (int i = 0; i < global_num_units; i++) {
			if (m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][i]) {
				if (i != my_id) {
					return true;
				}
			}
		}
	}
	return false;
}

void Boxes::add_collisions(Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator> &pq) {
	Box possible_boxes[27];
	std::vector<int> collision_ids;
	get_future_boxes(unit, possible_boxes);
	for (Box box : possible_boxes) {
		if (box.in_container) {
			for (int i = 0; i < global_num_units; i++) {
				if (m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][i]) {
					if (i != unit.get_id()) {
						int id = generate_collision_event(unit, i, pq);
						if (id != -1) {
							collision_ids.emplace_back(i);
						}
					}
				}
			}
		}
		else {
			std::array<bool, global_num_units> membership_array = m_membership_map[{box.positions[cX], box.positions[cY], box.positions[cZ],false}];
			for (int i = 0; i < global_num_units; i++) {
				if (i != unit.get_id()) {
					int id = generate_collision_event(unit, i, pq);
					if (id != -1) {
						collision_ids.emplace_back(i);
					}
				}
			}
		}
	}
	collision_ids.clear();
}

float Boxes::get_earliest_collision(Unit& unit, float(&location_array)[3], float(&direction_array)[3], int collision_partner_id) {
	//iterate through neighboring boxes and return earliest collision time
	float earliest_collision = std::numeric_limits<float>::infinity();
	Box possible_boxes[27];
	get_future_boxes(unit, possible_boxes);
	for (Box box : possible_boxes) {
		if (box.in_container) {
			for (int i = 0; i < global_num_units; i++) {
				if (m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][i]) {
					if (i != unit.get_id() && i != collision_partner_id) {
						float collisionTime = unit.get_direction_intersection_time(i, direction_array, false);

						if (collisionTime < earliest_collision && collisionTime > 0) {
							earliest_collision = collisionTime;
							unit.get_uc_timestamp(i);
						}
					}
				}
			}
		}
		else {
			std::array<bool, global_num_units> membership_array = m_membership_map[{box.positions[cX], box.positions[cY], box.positions[cZ], false}];
			for (int i = 0; i < global_num_units; i++) {
				if (i != unit.get_id() && i!=collision_partner_id) {
					float collisionTime = unit.get_uc_timestamp(i);
					if (collisionTime < earliest_collision&& collisionTime > 0) {
						earliest_collision = collisionTime;
					}
				}
			}
		}
	}
	return earliest_collision;
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

void Boxes::get_next_box_event(Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator>& event_queue) {
	Box& box = get_box(unit);
	Plane right, left, top, bottom, nearPlane, farPlane;

	if (box.in_container) {
		right = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][0];
		left = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][1];
		top = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][2];
		bottom = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][3];
		nearPlane = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][4];
		farPlane = m_planes[box.positions[cX]][box.positions[cY]][box.positions[cZ]][5];
	}
	else { //box not in container
		if (m_plane_map.find(box) == m_plane_map.end()) { //box not in map
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
			m_plane_map[box] = plane_array;
		}
		else { //box is in map
			right = m_plane_map[box][0];
			left = m_plane_map[box][1];
			top = m_plane_map[box][2];
			bottom = m_plane_map[box][3];
			nearPlane = m_plane_map[box][4];
			farPlane = m_plane_map[box][5];
		}
	}
	float min = INFINITY;
	Plane currPlane;
	Plane closestPlane;
	
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

	float intersectionTime = unit.calculate_intersection_time(min);

	BoxEvent data = { unit.get_id(), unit.get_age(), box, containerCollision };
	Event event = { BOX_EVENT, intersectionTime, {data} };
	event_queue.emplace(event);
}
