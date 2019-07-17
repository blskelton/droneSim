#include <boost/algorithm/clamp.hpp>
#include <unordered_map>

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
	int m_box_size[3] = {(int) Unit::DEFAULT_RADIUS * 4, (int)Unit::DEFAULT_RADIUS * 4, (int)Unit::DEFAULT_RADIUS * 4 };

	//number of boxes per side assuming cube container and boxes
	static constexpr int num_x_boxes = Container::CONTAINER_X / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_y_boxes = Container::CONTAINER_Y / (int)(Unit::DEFAULT_RADIUS * 4);
	static constexpr int num_z_boxes = Container::CONTAINER_Z / (int)(Unit::DEFAULT_RADIUS * 4);

	int m_boxes_per_side[3] = {num_x_boxes, num_y_boxes, num_z_boxes };
	Plane m_planes[num_x_boxes][num_y_boxes][num_z_boxes][3][2]; //in three directions, two planes per direction
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

	inline Box get_box(Unit& unit) {
		//get from unit location to box indices
		float unitX = unit.get_location()[cX];
		float unitY = unit.get_location()[cY];
		float unitZ = unit.get_location()[cZ];

		//transform location values to index from lower left far corner
		unitX += globalContainer.get_rightX();
		unitY += globalContainer.get_topY();
		unitZ += -(globalContainer.get_farZ());

		int xBoxIndex = unitX / m_box_size[cX];
		int yBoxIndex = unitY / m_box_size[cY];
		int zBoxIndex = unitZ / m_box_size[cZ];

		xBoxIndex = boost::algorithm::clamp(xBoxIndex, 0, 9);
		yBoxIndex = boost::algorithm::clamp(yBoxIndex, 0, 9);
		zBoxIndex = boost::algorithm::clamp(zBoxIndex, 0, 9);

		Box myBox = { xBoxIndex, yBoxIndex, zBoxIndex };
		m_unit_membership[xBoxIndex][yBoxIndex][zBoxIndex][unit.get_id()] = true;
		return myBox;
	};

	void add_collisions(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);

	inline void generate_collision_event(Unit& unit, int id, std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
		float collisionTime = unit.unit_collision(id);
		if (collisionTime > 0) {
			if (m_present_collisions[unit.get_id()][id] == false && m_present_collisions[id][unit.get_id()] == false) {
				Event collisionEvent = { UC_EVENT, unit.get_id(), unit.get_age(), id, collisionTime };
				pq.emplace(collisionEvent);
				m_present_collisions[unit.get_id()][id] = true;
				m_present_collisions[id][unit.get_id()] = true;
			}
		}
	};

	void get_future_boxes(Unit&, Box(&arr)[8]);
	
	std::vector<int> get_units_in_box(Unit&); //returns vector of unit IDs of units in same box(es)
	
	//std::vector<int> get_units_in_box(Box, vector<int>&);

	inline void handle_box_event(Event currEvent, Unit& unit, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		if (currEvent.containerCollision) {
			unit.check_container_collision();
		}
		//remove old membership
		m_unit_membership[currEvent.box.positions[cX]][currEvent.box.positions[cY]][currEvent.box.positions[cZ]][currEvent.idA] = false;
		//add new membership
		get_box(unit);
		Event nextEvent = get_next_box_event(unit);
		eventQueue.emplace(get_next_box_event(unit));
		add_collisions(unit, eventQueue);
	};

	inline void handle_uc_event(Event currEvent, Unit& unitA, Unit& unitB, std::priority_queue<Event, vector<Event>, myEventComparator>& eventQueue) {
		Event actionEvent = unitA.perform_unit_collision(unitB);
		if (actionEvent.timestamp > 0) {
			eventQueue.emplace(actionEvent);
		}
		remove_collision_tag(currEvent.idA, currEvent.idB);
	
		/*if (actionEvent.timestamp > m_curr_loop_start_time) { //collision occured, actionEvent is valid
			m_event_queue.emplace(actionEvent);
		}
		m_grid.add_collisions(m_unitArray[currEvent.idA], m_event_queue);
		m_grid.remove_collision_tag(currEvent.idA, currEvent.idB);*/
	};
	
	Event get_next_box_event(Unit&);
	
	float time_to_plane(Unit&, Plane);
	
	void get_plane_info(Plane, myVector&, myVector&);
	
};


#endif /* Boxes_h */