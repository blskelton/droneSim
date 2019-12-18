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
#include "Packages.h"
#include "globalMessaging.h"

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

	//unit colors
	float unit_colors[4][3];
	
	//for package use case
	Packages* m_package_manager;

	int m_package_carriers[NUMBER_PACKAGES];

	//global messaging vector
	//std::vector <std::array<Message, global_num_units>> m_messages;
	int i = 0;
	globalMessaging* m_global_messages;
	int m_round;

	//Package m_packages[NUMBER_PACKAGES];
	//int m_package_statuses[NUMBER_PACKAGES];
	//float m_package_fall_information[NUMBER_PACKAGES][2]; //2d array with fall time and height
	//std::vector<int> m_package_assignments[NUMBER_PACKAGES];
	
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

	inline void write_carrier(int unit_id, int assignment_id) {
		m_package_carriers[assignment_id] = unit_id;
	}

	inline int query_carrier(int assignment_id) {
		return m_package_carriers[assignment_id];
	}

	//returns current time
	inline float get_time() {
		return m_current_time;
	};

	inline int get_round() {
		return m_round;
	};

	inline void recv_global_message(Message message) {
		GlobalMessageEvent data = {message};
		Event new_event = { GLOBAL_MESSAGE_EVENT, get_time() + get_message_delay(), {data} };
		m_event_queue.emplace(new_event);
	};

	inline void write_message(Message message) {
		m_global_messages->write_message(message);
		//int round = message.get_round();
		//int sender_id = message.get_sender();
		//m_messages[round][sender_id] = message;
	};

	//returns random message delay
	inline int get_message_delay() {
		int delay = (int)abs(distribution(e2)) * 10;	
		return delay;
	};

	//sets up packages
	void initialize_packages();

	//makes gl calls to draw packages
	void draw_packages();

	inline void update_package_location(int assignment_id, int unit_status, float unit_x, float unit_y, float unit_z) {
		m_package_manager->set_location(assignment_id, unit_x, unit_y-0.5f, unit_z);
	};

	/*inline void update_package(int assignment_id, int unit_status, float unit_x, float unit_y, float unit_z) {
		int status = -1;
		if (unit_status == HEADED_TOWARDS_PICKUP) {
			status = WAITING_FOR_PICKUP;
		}
		if (unit_status == CARRYING_PACKAGE) {
			status = IN_TRANSIT;
		}
		float* new_dest = m_package_manager->update(assignment_id, status, unit_x, unit_y - 0.5f, unit_z);
		//m_packages[id].update_status(status, x, y, z);
		//m_packages[id].update_location(x, y, z);
	};*/

	inline int propose_status_change(int package_id, int current_unit_status, float x, float y, float z) {
		//accepts current unit status and assignment id
		//uses status, unit location, and package location to update package status
		//return new unit status
		return m_package_manager->propose_status_change(package_id, current_unit_status, x, y, z);
	};

	//returns a package given its id
	inline Package& get_package_info(int id) {
		Package& package = m_package_manager->get_package(id);
		return package;
	};

	//inline void update_package_carrier(int package_id, int carrier_id) {
		//m_packages[package_id].update_carrier(carrier_id);
	//};

	//inline int get_package_carrier(int package_id) {
		//return m_packages[package_id].carrier;
	//};

	//returns package status given its id
	inline int get_package_status(int id) {
		Package& package = m_package_manager->get_package(id);
		return package.status;
	}

	//updates package status to given status
	/*inline void update_package_status(int id, int status, float x, float y, float z) {
		m_packages[id].update_status(status, x, y, z);
	}*/

	inline int get_available_assignment_id() {
		for (int i = 0; i < NUMBER_PACKAGES; i++) {
			int status = m_package_manager->get_package(i).status;
			if (status== DROPPED||status==UNINITIALIZED) {
				if (query_carrier(i) == -1) {
					return i;
				}
			}
		}
		return -1;
	};

	//writes current time into package_fall_times array for falling animation
	inline void drop_package(int package_id, int unit_id) {
		Unit carrier = m_unitArray[unit_id];
		m_package_manager->set_status(package_id, DROPPING);
		m_package_manager->set_location(package_id, carrier.get_location()[cX], carrier.get_location()[cY] - 0.5f, carrier.get_location()[cZ]);
		write_carrier(-1, package_id);
		m_package_manager->drop_package(package_id, m_current_time);
		/*m_packages[id].update_location(carrier.get_location()[cX], carrier.get_location()[cY]-0.5f, carrier.get_location()[cZ]);
		m_packages[id].update_carrier(-1);
		//m_packages[id].update_status(DROPPING, carrier.get_location()[cX], carrier.get_location()[cY] - 0.5f, carrier.get_location()[cZ]);
		m_packages[id].set_status(DROPPING);
		m_package_fall_information[id][0] = m_current_time;
		m_package_fall_information[id][1] = m_packages[id].position[cY];*/
	}

	//returns timestamp of earliest collision with a test direction
	inline float get_earliest_collision(Unit& unit, float(&location)[3], float(&direction)[3], int collision_partner_id) {
		return m_grid->get_earliest_collision(unit, location, direction, collision_partner_id);
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
					//m_unitArray[currEvent.data.pingEvent.id].set_color(WHITE);
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

				if (currEvent.tag == GLOBAL_MESSAGE_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					write_message(currEvent.data.globalMessageEvent.message);
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
			//draw_unit(m_unitArray[i].get_id());
			m_unitArray[i].user_process();
			check_core_collisions(m_unitArray[i]);
		}
		draw_units();
		if (time_remaining > m_longest_process) {
			m_longest_process = time_remaining;
		}

		if (packages) {
			draw_packages();
		}
		m_prev_loop_end_time = m_curr_loop_start_time;
		m_round++;
		//std::array<Message, global_num_units> empty_array;
		//m_messages.emplace_back(empty_array);
		if (m_round > 200) {
			int val = 0;
		}
		m_global_messages->add_array();
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

	inline std::array<Message, global_num_units>& recv(int round, bool &populated) {
		//std::array<Message, global_num_units>& my_messages = m_global_messages->recv(round, populated);
		return m_global_messages->recv(round, populated);
		//std::array<Message, global_num_units> my_messages;
	//	m_global_messages->test_recv(0, std::array<Message, global_num_units> &my_messages);
		//m_global_messages->place_test_message();
	//	m_global_messages->test_recv(0, my_messages);
		//my_messages = m_global_messages->test_recv(round);
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
	void draw_units();
	
	//adds message event to pq
	void send_message(int, int);
};

#endif /*Environment_h*/