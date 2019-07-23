#include <boost/algorithm/clamp.hpp>
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

	//plane map
	std::map<std::array<int, 3>, std::array<Plane, 6>> m_plane_map; //hash func for box?

	Plane m_bounding_planes[6]; //bounding planes in each direction
	//4d nested array representing whether not a given unit is in a given box
	bool m_unit_membership[num_x_boxes][num_y_boxes][num_z_boxes][global_num_units];
	bool m_present_collisions[global_num_units][global_num_units];

public:
	Boxes();

	virtual ~Boxes();

	inline void remove_collision_tag(int idA, int idB) {
		m_present_collisions[idA][idB] = false;
		m_present_collisions[idB][idA] = false;
	};

	inline void get_box_dimensions(int(&arr)[3]) {
		arr[cX] = m_box_size[cX];
		arr[cY] = m_box_size[cY];
		arr[cZ] = m_box_size[cZ];
		return;
	};

	inline Box get_box(Unit& unit) {
		//get from unit location to box indices
		float unitX = unit.get_location()[cX];
		float unitY = unit.get_location()[cY];
		float unitZ = unit.get_location()[cZ];

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
			in_container = false;
			positive_dir = false;
		}
		if (unitY < 0) {
			unitY = std::floor(unitY / m_box_size[cY]);
			in_container = false;
			positive_dir = false;
		}
		if (unitZ < 0) {
			unitZ = std::floor(unitZ / m_box_size[cZ]);
			in_container = false;
			positive_dir = false;
		}
		Box myBox = {(int) unitX,(int) unitY, (int)unitZ, in_container };
		m_unit_membership[(int)unitX][(int)unitY][(int)unitZ][unit.get_id()] = true; //have to change for non-container boxes (hashmap?)
		return myBox;
	};

	void add_collisions(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);

	inline void generate_collision_event(Unit& unit, int id, std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
		float collisionTime = unit.unit_collision(id);
		if (collisionTime > 0) {
			if (m_present_collisions[unit.get_id()][id] == false && m_present_collisions[id][unit.get_id()] == false) {
				UCEvent data = { unit.get_id(), unit.get_age(), id };
				Event collisionEvent = { UC_EVENT, collisionTime, {data} };
				pq.emplace(collisionEvent);
				m_present_collisions[unit.get_id()][id] = true;
				m_present_collisions[id][unit.get_id()] = true;
			}
		}
	};

	void get_future_boxes(Unit&, Box(&arr)[8]);
	
	std::vector<int> get_units_in_box(Unit&); //returns vector of unit IDs of units in same box(es)

	inline void handle_box_event(Event currEvent, Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		Box box = get_box(unit);
		if (currEvent.data.boxEvent.containerCollision && unit.get_container_bool()) {
			unit.check_container_collision();
		}
		if (currEvent.data.boxEvent.containerCollision && !unit.get_container_bool()) {
			//get box
			if (box.in_container) {
				unit.reenter_container();
			}
			else { //not in container
				eventQueue.emplace(get_next_non_container_box_event(unit, box));
			}
			//if not in container, make another call to get_outside_box_collision
		}

		//remove old membership
		int positionsX = currEvent.data.boxEvent.box.positions[cX];
		int positionsY = currEvent.data.boxEvent.box.positions[cY];
		int positionsZ = currEvent.data.boxEvent.box.positions[cZ];
		m_unit_membership[positionsX][positionsY][positionsZ][currEvent.data.boxEvent.id] = false;
		//add new membership
		get_box(unit);
		//Event nextEvent = get_next_box_event(unit);
		eventQueue.emplace(get_next_box_event(unit));
		add_collisions(unit, eventQueue);
	};

	inline void handle_uc_event(Event currEvent, Unit& unitA, Unit& unitB, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		unitA.perform_unit_collision(unitB, eventQueue);
		remove_collision_tag(currEvent.data.ucEvent.idA, currEvent.data.ucEvent.idB);
	};
	
	Event get_next_box_event(Unit&);

	Event get_next_non_container_box_event(Unit&, Box&);
	
	float time_to_plane(Unit&, Plane);
	
	void get_plane_info(Plane, myVector&, myVector&);
	
};


#endif /* Boxes_h */