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
	int m_box_size[3] = {(int) (Unit::DEFAULT_RADIUS * 4), (int)(Unit::DEFAULT_RADIUS * 4), (int)(Unit::DEFAULT_RADIUS * 4) };

	//number of boxes per side assuming cube container and boxes
	static constexpr int num_x_boxes = Container::CONTAINER_X / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_y_boxes = Container::CONTAINER_Y / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_z_boxes = Container::CONTAINER_Z / (int)(Unit::DEFAULT_RADIUS * 4);

	int m_boxes_per_side[3] = {num_x_boxes, num_y_boxes, num_z_boxes };
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

	vector<Event> m_pairwise_collisions[global_num_units][global_num_units];

public:
	Boxes();

	virtual ~Boxes();

	inline void remove_collision_tag(int idA, int idB) {
		m_present_collisions[idA][idB] = -1;
		//m_present_collisions[idB][idA] = false;
	};

	inline void get_box_dimensions(int(&arr)[3]) {
		arr[cX] = m_box_size[cX];
		arr[cY] = m_box_size[cY];
		arr[cZ] = m_box_size[cZ];
		return;
	};

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

		bool in_container = true;
		bool positive_dir = true;

		if (unitX > rightX || unitY > topY || unitZ > closeZ) { //out of boundaries in positive direction
			in_container = false;
		}

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
		Box myBox = { (int)unitX, (int)unitY, (int)unitZ, boundaries };
		if (myBox.in_container) {
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

	void add_collisions(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);

	inline void generate_collision_event(Unit& unit, int id, std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
		float collisionTime = unit.unit_collision(id, false);
		if (collisionTime > 0) {
			int current_collision_age = m_present_collisions[unit.get_id()][id];
			if (current_collision_age != unit.get_age()) {
				UCEvent data = { unit.get_id(), unit.get_age(), id };
				Event collisionEvent = { UC_EVENT, collisionTime, {data} };
				pq.emplace(collisionEvent);
				m_pairwise_collisions[unit.get_id()][id].emplace_back(collisionEvent);
				m_present_collisions[unit.get_id()][id] = unit.get_age();
				//m_present_collisions[id][unit.get_id()] = true;
			}
			/*if (m_present_collisions[unit.get_id()][id] == false && m_present_collisions[id][unit.get_id()] == false) {
				UCEvent data = { unit.get_id(), unit.get_age(), id };
				Event collisionEvent = { UC_EVENT, collisionTime, {data} };
				pq.emplace(collisionEvent);
				m_present_collisions[unit.get_id()][id] = true;
				m_present_collisions[id][unit.get_id()] = true;
			}*/
		}
	};

	void get_future_boxes(Unit&, Box(&arr)[8]);

	inline vector<Event> get_events(int idA, int idB) {
		return m_pairwise_collisions[idA][idB];
	};

	bool has_neighbors(Unit&);
	
	std::vector<int> get_units_in_box(Unit&); //returns vector of unit IDs of units in same box(es)

	inline void handle_box_event(Event currEvent, Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		Box& box = get_box(unit);
		if (currEvent.data.boxEvent.containerCollision && unit.get_container_bool()) {
			unit.check_container_collision();
		}
		if (currEvent.data.boxEvent.containerCollision && !unit.get_container_bool()) {
			//get box
			if (box.in_container) {
				unit.reenter_container();
			}
			else { //not in container
				eventQueue.emplace(get_next_box_event(unit));
			}
			//if not in container, make another call to get_outside_box_collision
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
		//Event nextEvent = get_next_box_event(unit);
		eventQueue.emplace(get_next_box_event(unit));
		add_collisions(unit, eventQueue);
	};

	inline void handle_uc_event(Event currEvent, Unit& unitA, Unit& unitB, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		Event avoidance = unitA.perform_unit_collision(unitB, eventQueue);
		m_pairwise_collisions[unitA.get_id()][unitB.get_id()].emplace_back(avoidance);
	};

	inline int get_last_collision_age(int idA, int idB) {
		return m_present_collisions[idA][idB];
	};

	inline void get_neighbors(Unit& unit, vector<int>& neighbors) {
		//vector<int> neighbors;
		Box possible_boxes[8];
		get_future_boxes(unit, possible_boxes);
		int id = unit.get_id();
		for (Box box : possible_boxes) {
			if (box.in_container) {
				for (int i = 0; i < global_num_units; i++) {
					if (i!=id && m_unit_membership[box.positions[cX]][box.positions[cY]][box.positions[cZ]][i]) {
						neighbors.emplace_back(i);
					}
				}
			}

			else {
				std::array<bool, global_num_units> membership_array = m_membership_map[{box.positions[cX], box.positions[cY], box.positions[cZ],false}];
				for (int i = 0; i < global_num_units; i++) {
					if (i != id && membership_array[i]) {
						neighbors.emplace_back(i);
					}
				}
			}
		}
		return;// neighbors;
	};

	inline void clear_present_collisions(int idA, int idB) {
		m_present_collisions[idA][idB] = 0;
	}
	
	Event get_next_box_event(Unit&);
	
	float time_to_plane(Unit&, Plane);
	
	void get_plane_info(Plane, myVector&, myVector&);
	
};


#endif /* Boxes_h */