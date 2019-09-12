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

#include <unordered_map>
#include <map>
#include <cmath>

#include "Unit.h"
#include "simulator.h"
#include "Geometry.h"
#include "Event.h"
#include "Container.h"

#ifndef Boxes_h
#define Boxes_h

using std::unordered_map;
using std::vector;

class Environment;

class Boxes {
private:
	//box dimensions (proportional to default radius)
	int m_box_size[3] = { (int)(Unit::DEFAULT_RADIUS * 4), (int)(Unit::DEFAULT_RADIUS * 4), (int)(Unit::DEFAULT_RADIUS * 4) };

	//number of boxes per side assuming cube container and boxes
	static constexpr int num_x_boxes = Container::CONTAINER_X / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_y_boxes = Container::CONTAINER_Y / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_z_boxes = Container::CONTAINER_Z / (int)(Unit::DEFAULT_RADIUS * 4);

	int m_boxes_per_side[3] = { num_x_boxes, num_y_boxes, num_z_boxes };
	Plane m_bounding_planes[6]; //bounding planes in each direction

	//unit membership, plane information, box information for boxes within container
	bool m_unit_membership[num_x_boxes][num_y_boxes][num_z_boxes][global_num_units];
	Plane m_planes[num_x_boxes][num_y_boxes][num_z_boxes][6];
	Box m_boxes[num_x_boxes][num_y_boxes][num_z_boxes];

	//unit membership and plane information for boxes outside of container
	std::vector<Box> m_outside_boxes;
	std::unordered_map<Box, std::array<Plane, 6>> m_plane_map; //hash func for box?
	std::unordered_map<Box, std::array<bool, global_num_units>> m_membership_map;

	int m_present_collisions[global_num_units][global_num_units]; //age at which unit A generated collision event with unit B
	int m_id_counter;

public:
	Boxes();

	virtual ~Boxes();

	//resets tracked collisions for a pair of units
	inline void remove_collision_tag(int idA, int idB) {
		m_present_collisions[idA][idB] = -1;
	};

	//calculates earliest collision with test direction
	float get_earliest_collision(Unit& unit, float(&location_array)[3], float(&direction_array)[3], int);

	//calculates box and updates membership
	inline Box& get_box(Unit& unit) {
		//get from unit location to box indices
		float unitX = unit.get_location()[cX];
		float unitY = unit.get_location()[cY];
		float unitZ = unit.get_location()[cZ];

		int boundaries[6] = { 0, m_boxes_per_side[cX] - 1, 0, m_boxes_per_side[cY] - 1, 0, m_boxes_per_side[cZ] - 1 };

		//get container boundaries
		int rightX = globalContainer.get_rightX();
		int topY = globalContainer.get_topY();
		int closeZ = globalContainer.get_closeZ();

		//transform location values to index from lower left far corner
		unitX += rightX;
		unitY += topY;
		unitZ += -(globalContainer.get_farZ());


		if (unitX < 0) {
			unitX = std::floor(unitX / m_box_size[cX]);
		}
		if (unitY < 0) {
			unitY = std::floor(unitY / m_box_size[cY]);
		}
		if (unitZ < 0) {
			unitZ = std::floor(unitZ / m_box_size[cZ]);
		}

		/*m_counter++;
		int id = unit.get_id();
		int new_counter = 0;
		if (m_counter > 500000) {
			for (int i = 0; i < num_x_boxes; i++) {
				for (int j = 0; j < num_y_boxes; j++) {
					for (int k = 0; k < num_z_boxes; k++) {
						if (m_unit_membership[i][j][k][id]) {
							new_counter++;
						}
					}
				}
			}
			int val = 0;
		}*/

		Box myBox = { (int)unitX, (int)unitY, (int)unitZ, boundaries };
		if (myBox.in_container) {
			int id = unit.get_id();
			m_unit_membership[(int)unitX][(int)unitY][(int)unitZ][unit.get_id()] = true;
			Box& myBox = m_boxes[(int)unitX][(int)unitY][(int)unitZ];
			return myBox;
		}
		else {
			static Box myBox = Box{ (int)unitX, (int)unitY, (int)unitZ, false };
			m_membership_map[myBox][unit.get_id()] = true;
			return myBox;
		}
	};

	//adds collisions with any units in neighboring boxes
	void add_collisions(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);

	//gets collision timestamp and generates event if necessary
	inline int generate_collision_event(Unit& unit, int id, std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
		float collisionTime = unit.get_uc_timestamp(id);
		if (collisionTime > 0) {
			int current_collision_age = m_present_collisions[unit.get_id()][id];
			if (current_collision_age != unit.get_age()) {
				UCEvent data = { unit.get_id(), unit.get_age(), id };
				Event collisionEvent = { UC_EVENT, collisionTime, {data} };
				pq.emplace(collisionEvent);
				m_present_collisions[unit.get_id()][id] = unit.get_age();
				return id;
			}
			return -1;
		}
	};

	//populates parameter array with 27 neighboring boxes
	void get_future_boxes(Unit&, Box(&arr)[27]);

	//returns true if a unit has neighbors
	bool has_neighbors(Unit&);
	
	//updates container information if necessary, removes old membership, adds new box and collision events
	inline void handle_box_event(Event currEvent, Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator>& event_queue) {
		Box& box = get_box(unit);
		if (currEvent.data.boxEvent.containerCollision && unit.get_container_bool()) { //about to cross boundary in -> out
			unit.check_container_collision();
		}
		if (currEvent.data.boxEvent.containerCollision && !unit.get_container_bool()) { //about to cross boundary out -> in
			if (box.in_container) { //next box is in container, must reenter
				unit.reenter_container(); 
			}
		}

		//remove old membership
		int positionsX = currEvent.data.boxEvent.box.positions[cX];
		int positionsY = currEvent.data.boxEvent.box.positions[cY];
		int positionsZ = currEvent.data.boxEvent.box.positions[cZ];
		if (currEvent.data.boxEvent.box.in_container) {
			m_unit_membership[positionsX][positionsY][positionsZ][currEvent.data.boxEvent.id] = false;
		}
		else {
			m_membership_map[{positionsX, positionsY, positionsZ, false}][currEvent.data.boxEvent.id] = false;
		}
		get_next_box_event(unit, event_queue);
		add_collisions(unit, event_queue);
	};

	//calls method in unit class to handle collisions
	inline void handle_uc_event(Event currEvent, Unit& unitA, Unit& unitB, std::priority_queue<Event, vector<Event>, myEventComparator>& event_queue) {
		unitA.perform_unit_collision(unitB, event_queue);
	};

	//accesses most recently logged collision given a pair of units
	inline int get_last_collision_age(int idA, int idB) {
		return m_present_collisions[idA][idB];
	};
	
	//adds next box transition event to the pq
	void get_next_box_event(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);
	
	//calculates the milliseconds it will take a unit before it intersects with a plane
	float time_to_plane(Unit&, Plane);
	
	//populates vectors with plane information
	void get_plane_info(Plane, myVector&, myVector&);
};


#endif /* Boxes_h */