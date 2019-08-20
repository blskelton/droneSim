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
	priority_queue<Event, vector<Event>, myEventComparator> m_event_queue;
	std::array<Unit, global_num_units> m_unitArray;
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
	
	int m_num_packages;
	Package m_packages[global_num_units];
	int m_package_statuses[global_num_units];
	
	float m_max_loop_distance = 0.25;

public:
	Environment();

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

		SpeedChangeEvent data = { };
		float timestamp = m_prev_loop_end_time+m_speed_change_frequency;

		Event speed_change_event = { SPEED_CHANGE_EVENT, timestamp, {data} };
		m_event_queue.emplace(speed_change_event);
	};

	inline float get_time() {
		return m_current_time;
	}

	inline int get_message_delay() {
		int delay = abs(distribution(e2)) * 10;	
		return delay;
	};

	void initialize_packages();

	void draw_packages();

	inline Package& get_package_info(int id) {
		Package& package = m_packages[id];
		return package;
	};

	inline int get_package_status(int id) {
		Package& package = m_packages[id];
		return package.status;
	}

	inline void update_package_status(int id) {
		m_packages[id].update_status();
	}

	inline void get_random_position(int (&numbers)[3]) {
		numbers[cX] = (rand() % 20) - 10;
		numbers[cY] = (rand() % 20) - 10;
		numbers[cZ] = (rand() % 20) - 30;
	};

	inline void recalculate_collisions(Unit& unit) {//call when unit changes speed or direction
		//increment unit age by 1 to invalidate old events
		unit.increment_age();
		//add new unit collision events
		m_grid->add_collisions(unit, m_event_queue);
		//add new box transfer event
		m_grid->get_next_box_event(unit, m_event_queue);
	};

	inline void recalculate_uc_collision(int idA, int idB) {
		Unit& unit = m_unitArray[idA];
		m_grid->remove_collision_tag(idA, idB);
		m_grid->generate_collision_event(unit, idB, m_event_queue);
	};

	inline void process() {
		//update loop time
		m_curr_loop_start_time = get_num_milliseconds();
		m_current_time = m_prev_loop_end_time;
		float last_milestone = m_prev_loop_end_time;
		float loop_time = (m_curr_loop_start_time - m_prev_loop_end_time);
		
		float loop_distance = loop_time * 0.01; //0.01 = max speed
		//if (loop_distance > m_max_loop_distance) {
			//int val = 0;
		//}

		if (m_event_queue.size() > 0) {
			while (m_event_queue.top().timestamp <= m_curr_loop_start_time) {
				Event currEvent = m_event_queue.top();

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

				if (currEvent.tag == ACTION_EVENT) {
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
						}
					}
					m_unitArray[currEvent.data.actionEvent.id].handle_action_event();
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
					m_unitArray[id].handle_wait_event(m_event_queue);
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

		for (int i = 0; i < global_num_units; i++) {
			m_unitArray[i].process(time_remaining);
			draw_unit(m_unitArray[i].get_id());
			m_unitArray[i].user_process();
			check_core_collisions(m_unitArray[i]);
		}

		if (packages) {
			draw_packages();
		}
		m_prev_loop_end_time = m_curr_loop_start_time;
	};

	void check_core_collisions(Unit&);

	bool check_initial_collisions(int);

	//add unit to array of units
	void add_unit(int unitID);

	//get unit object from id
	inline Unit& get_unit(int id) {
		return m_unitArray[id];
	};

	inline float get_num_milliseconds() {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);

		float dif = (now.QuadPart - m_start_time.QuadPart);
		dif = dif / m_freq.QuadPart;
		return dif*1000;
	};

	inline Box& get_box(Unit& unit) {
		return m_grid->get_box(unit);
	}

	inline void add_out_of_container_box_event(Unit& unit, Box& box) {
		m_grid->get_next_box_event(unit, m_event_queue);
	};
	
	void draw_unit(int);
	
	void send_message(int, int);
};

#endif /*Environment_h*/