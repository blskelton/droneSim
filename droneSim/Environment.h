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
#include <cassert>
#include <random>
#include <cmath>

#include "Unit.h"
#include "Boxes.h"
#include "Container.h"
#include "Event.h"
#include "simulator.h"

#ifndef Environment_h
#define Environment_h

using std::priority_queue;


class Environment {
private:
	//global event manager
	priority_queue<Event, vector<Event>, myEventComparator> m_event_queue;
	//array of units
	std::array<Unit, global_num_units> m_unitArray;
	//instance of grid for managing efficient collision detection
	Boxes* m_grid;

	//time
	LARGE_INTEGER m_freq;
	LARGE_INTEGER m_start_time;
	float m_prev_loop_end_time;
	float m_curr_loop_start_time;
	float m_current_time;

	//for random message delay
	std::random_device m_rd;
	std::mt19937 e2;
	std::normal_distribution<> distribution;
	float m_speed_change_frequency;
	
	//for package use case
	Package m_packages[NUMBER_PACKAGES];
	int m_package_statuses[NUMBER_PACKAGES];
	float m_package_fall_times[NUMBER_PACKAGES];
	std::vector<int> m_package_assignments[NUMBER_PACKAGES];
	
	//longest time between consecutive events (in ms)
	float m_longest_process;

public:
	Environment();

	//call at end of initialization to set up timestamps for current and future events
	inline void init_prev_end_timestamp() { 
		QueryPerformanceCounter(&m_start_time);
		QueryPerformanceFrequency(&m_freq);

		m_prev_loop_end_time = get_num_milliseconds();
		m_current_time = m_prev_loop_end_time;

		//write all events into vector of events
		int size = m_event_queue.size();
		vector<Event> event_vector;
		for (int i = 0; i < size; i++) {
			Event currEvent = m_event_queue.top(); //get current event
			m_event_queue.pop(); //remove from queue
			event_vector.emplace_back(currEvent);
		}
		for (int i = 0; i < size; i++) {
			Event currEvent = event_vector[i];
			currEvent.update_timestamp(m_current_time);
			m_event_queue.emplace(currEvent);
		}

		//add first speed change event to pq
		SpeedChangeEvent data = { };
		float timestamp = m_prev_loop_end_time+m_speed_change_frequency;

		Event speed_change_event = { SPEED_CHANGE_EVENT, timestamp, {data} };
		m_event_queue.emplace(speed_change_event);
	};

	//returns current time
	inline float get_time() {
		return m_current_time;
	}

	//returns random message delay
	inline int get_message_delay() {
		int delay = (int)abs(distribution(e2)) * 10;	
		return delay;
	};

	//updates package assignment array
	inline void update_assignment(int assignment_id, int content) {
		m_package_assignments[assignment_id].push_back(content);
	};

	//returns most recent assignment entry
	inline int check_assignment(int assignment_id) {
		if (m_package_assignments[assignment_id].size() == 0) {
			return -1;
		}
		return m_package_assignments[assignment_id].back();
	};

	//sets up packages
	void initialize_packages();

	//makes gl calls to draw packages
	void draw_packages();

	//returns a package given its id
	inline Package& get_package_info(int id) {
		Package& package = m_packages[id];
		return package;
	};

	/*inline void update_package_location(int id, float new_location[3]) {
		m_packages[id].update_location(new_location);
	};*/

	inline void update_package_carrier(int package_id, int carrier_id) {
		m_packages[package_id].update_carrier(carrier_id);
	};

	//returns package status given its id
	inline int get_package_status(int id) {
		Package& package = m_packages[id];
		return package.status;
	}

	//updates package status to given status
	inline void update_package_status(int id, int status) {
		m_packages[id].update_status(status);
	}

	//writes current time into package_fall_times array for falling animation
	inline void drop_package(int id) {
		m_package_fall_times[id] = m_current_time;
	}

	//returns timestamp of earliest collision with a test direction
	inline float get_earliest_collision(Unit& unit, float(&location)[3], float(&direction)[3], int collision_partner_id) {
		return m_grid->get_earliest_collision(unit, location, direction, collision_partner_id);
	};

	//populates array with random position within container
	inline void get_random_position(float (&numbers)[3]) {
		numbers[cX] = (float)(rand() % 20) - 10;
		numbers[cY] = (float)(rand() % 20) - 10;
		numbers[cZ] = (float)(rand() % 20) - 30;
	};

	//updates age and collision information for a unit upon a speed/direction change
	inline void recalculate_collisions(Unit& unit) {
		//increment unit age by 1 to invalidate old events
		unit.increment_age();
		//add new unit collision events
		m_grid->add_collisions(unit, m_event_queue);
		//add new box transfer event
		m_grid->get_next_box_event(unit, m_event_queue);
	};

	//regenerates collision event with two units
	inline void recalculate_uc_collision(int idA, int idB) {
		Unit& unit = m_unitArray[idA];
		m_grid->remove_collision_tag(idA, idB);
		m_grid->generate_collision_event(unit, idB, m_event_queue);
	};

	inline bool check_direction(int idA, int idB, float(&direction)[3]) {
		Unit& unit = m_unitArray[idA];
		//m_grid->remove_collision_tag(idA, idB);
		float collisionTime = unit.get_direction_intersection_time(idB, direction, true);
		if (collisionTime > -1 && collisionTime < INFINITY) {
			return true;
		}
		return false;
	};

	//considers and handles all events up to current time
	inline void process() {
		//update loop time
		m_curr_loop_start_time = get_num_milliseconds();
		m_current_time = m_prev_loop_end_time;
		float last_milestone = m_prev_loop_end_time;
		float loop_time = (m_curr_loop_start_time - m_prev_loop_end_time);

		if (m_event_queue.size() > 0) {
			//if there is an event with a timestamp in the past, handle it
			while (m_event_queue.top().timestamp <= m_curr_loop_start_time) {
				Event currEvent = m_event_queue.top();
				
				//get time offset and update current time
				float t = currEvent.timestamp - last_milestone;
				m_current_time += t;

				if (currEvent.tag == BOX_EVENT) {
					int actual_age = m_unitArray[currEvent.data.boxEvent.id].get_age();
					if (currEvent.data.boxEvent.age == actual_age) {
						if (t > 0) {
							for (int i = 0; i < global_num_units; i++) {
								m_unitArray[i].process(t);
							}
						}
						m_grid->handle_box_event(currEvent, m_unitArray[currEvent.data.boxEvent.id], m_event_queue);
					}
				}

				if (currEvent.tag == UC_EVENT) {
					int actual_age = m_grid->get_last_collision_age(currEvent.data.ucEvent.idA, currEvent.data.ucEvent.idB);
					if (currEvent.data.ucEvent.ageA == actual_age) {
						if (t > 0) {
							for (int i = 0; i < global_num_units; i++) {
								m_unitArray[i].process(t);
							}
						}
						m_grid->handle_uc_event(currEvent, m_unitArray[currEvent.data.ucEvent.idA], m_unitArray[currEvent.data.ucEvent.idB], m_event_queue);
					}
					else {
						recalculate_uc_collision(currEvent.data.ucEvent.idA, currEvent.data.ucEvent.idB);
					}
				}

				/*if (currEvent.tag == ACTION_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					m_unitArray[currEvent.data.actionEvent.id].handle_action_event();
				}*/

				if (currEvent.tag == PING_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					m_unitArray[currEvent.data.pingEvent.id].handle_ping_event(currEvent.data.pingEvent.tag);
				}

				if (currEvent.tag == ETA_EVENT) {
					int id = currEvent.data.etaEvent.id;
						if (t > 0) {
							for (int i = 0; i < global_num_units; i++) {
								m_unitArray[i].process(t);
							}
						}
					m_unitArray[id].handle_eta_event(m_event_queue, currEvent.data.etaEvent.epsilon);
				}

				if (currEvent.tag == WAIT_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					int id = currEvent.data.waitEvent.id;
					int tag = currEvent.data.waitEvent.tag;
					if (tag == 1) {
						int val = 1;
					}
					m_unitArray[id].handle_wait_event(m_event_queue, tag);
				}

				if (currEvent.tag == SPEED_CHANGE_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					for (Unit& unit : m_unitArray) {
						unit.update_speed();
					}
					for (Unit& unit : m_unitArray) {
						recalculate_collisions(unit);
					}
					SpeedChangeEvent data = { };
					float timestamp = currEvent.timestamp + m_speed_change_frequency;

					Event speed_change_event = { SPEED_CHANGE_EVENT, timestamp, {data} };
					m_event_queue.emplace(speed_change_event);
				}

				if (t > m_longest_process) {
					m_longest_process = t;
				}

				last_milestone += t;
				m_current_time = last_milestone;

				if (m_event_queue.size() == 0) {
					return;
				}
				m_event_queue.pop();
			}
		}
		float time_remaining = m_curr_loop_start_time - last_milestone;
		m_current_time += time_remaining;

		//process all units for time remaining after processing all past events
		for (int i = 0; i < global_num_units; i++) {
			m_unitArray[i].process(time_remaining);
			draw_unit(m_unitArray[i].get_id());
			m_unitArray[i].user_process();
			check_core_collisions(m_unitArray[i]);
		}

		if (time_remaining > m_longest_process) {
			m_longest_process = time_remaining;
		}

		if (packages) {
			draw_packages();
		}
		m_prev_loop_end_time = m_curr_loop_start_time;
	};

	//checks for core collisions
	void check_core_collisions(Unit&);

	//checks for collisions upon system initialization
	bool check_initial_collisions(int);

	//add unit to array of units
	void add_unit(int unitID);

	//get unit object from id
	inline Unit& get_unit(int id) {
		return m_unitArray[id];
	};

	//access longest processing period
	inline float get_longest_process() {
		return m_longest_process;
	};

	//update time
	inline float get_num_milliseconds() {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);

		float dif = (float)(now.QuadPart - m_start_time.QuadPart);
		dif = dif / m_freq.QuadPart;
		return dif*1000;
	};

	//return the box containing a given unit
	inline Box& get_box(Unit& unit) {
		return m_grid->get_box(unit);
	};
	
	//calls to gl to draw unit
	void draw_unit(int);
	
	//adds message event to pq
	void send_message(int, int);
};

#endif /*Environment_h*/