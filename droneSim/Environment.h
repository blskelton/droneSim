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
	LARGE_INTEGER m_freq;
	//start time
	LARGE_INTEGER m_start_time;
	float m_prev_loop_end_time;
	float m_curr_loop_start_time;
	std::array<Unit, global_num_units> m_unitArray;
	int m_camera[3];
	int m_look[3];
	int m_view;
	Boxes* m_grid;
	int max_queue_size;
	//for random message delay
	std::random_device m_rd;
	std::mt19937 e2;
	std::normal_distribution<> distribution;
	bool m_units_colliding[global_num_units];
	int m_speed_change_counter;
	vector<int> m_neighbors;
	float m_speed_change_frequency;
	float m_distance;
	float m_camera_angle;
	float lx = 0.0f, lz = -1.0f;


	//FOR TESTING
	vector<float> m_loop_times;
	vector<float> m_hundred_loop_times;
	float hundred_loop_start = 0;
	float testTime;

public:
	Environment();

	inline void init_prev_end_timestamp() {
		m_prev_loop_end_time = get_num_milliseconds();
		testTime = m_prev_loop_end_time;

		SpeedChangeEvent data = { };
		float timestamp = m_prev_loop_end_time+m_speed_change_frequency;

		Event speed_change_event = { SPEED_CHANGE_EVENT, timestamp, {data} };
		m_event_queue.emplace(speed_change_event);
	};

	inline float get_time() {
		return testTime;
		//return m_curr_loop_start_time;
	}

	inline int get_message_delay() {
		int delay = abs(distribution(e2)) * 10;
		//int vals[100];
		//for (int i = 0; i < 100; i++) {
			//vals[i] = ((int)(abs(distribution(e2)) * 10));
		//}		
		return delay;
	};

	inline void recalculate_collisions(Unit& unit) {//call when unit changes speed or direction
		//increment unit age by 1 to invalidate old events
		unit.increment_age();
		//add new unit collision events
		m_grid->add_collisions(unit, m_event_queue);
		//add new box transfer event
		m_event_queue.emplace(m_grid->get_next_box_event(unit));
		//can't recalculate destination event; causes infinite loop
	};

	inline void recalculate_uc_collision(int idA, int idB) {
		Unit& unit = m_unitArray[idA];
		m_grid->clear_present_collisions(idA, idB);
		m_grid->generate_collision_event(unit, idB, m_event_queue);
	};

	inline void process() {
		//update loop time
		m_curr_loop_start_time = get_num_milliseconds();
		testTime = m_curr_loop_start_time;
		float last_milestone = m_prev_loop_end_time;
		float loop_time = (m_curr_loop_start_time - m_prev_loop_end_time);

		m_loop_times.emplace_back(loop_time);
		if (m_loop_times.size() == 100) {

		}


		if (m_event_queue.size() > 0) {
			while (m_event_queue.top().timestamp <= m_curr_loop_start_time) {
				Event currEvent = m_event_queue.top(); //make event const?
				float t = currEvent.timestamp - last_milestone;
				testTime += t;
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
					int actual_age = m_grid->get_last_collision_age(currEvent.data.ucEvent.idA, currEvent.data.ucEvent.idB);//m_unitArray[currEvent.data.ucEvent.idA].get_age();
					if (currEvent.data.ucEvent.ageA == actual_age) {
						if (t > 0) {
							for (int i = 0; i < global_num_units; i++) {
								m_unitArray[i].process(t);
							}
						}
						Unit& unitA = m_unitArray[currEvent.data.ucEvent.idA];
						Unit& unitB = m_unitArray[currEvent.data.ucEvent.idB];
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
					int id = currEvent.data.waitEvent.id;
					m_unitArray[id].handle_wait_event(m_event_queue);
				}

				if (currEvent.tag == SPEED_CHANGE_EVENT) {
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

				if (m_event_queue.size() == 0) {
					return;
				}
				m_event_queue.pop();
			}
		}
		float time_remaining = m_curr_loop_start_time - last_milestone;
		for (int i = 0; i < global_num_units; i++) {
			m_unitArray[i].process(time_remaining);
			draw_unit(m_unitArray[i].get_id());
			m_unitArray[i].user_process();
		}

		/*if (m_speed_change_counter == 100) {
			float dif = m_curr_loop_start_time - hundred_loop_start;
			hundred_loop_start = m_curr_loop_start_time;
			m_hundred_loop_times.emplace_back(dif);
			for (int i = 0; i < global_num_units; i++) {
				//m_unitArray[i].update_speed();
			}
			if (m_hundred_loop_times.size() > 100) {
				m_hundred_loop_times.clear();
			}
			m_speed_change_counter = 0;
		}
		//if (m_speed_change_counter == 100) {
			//m_speed_change_counter = 0;
		//}
		m_speed_change_counter++;*/
		m_prev_loop_end_time = m_curr_loop_start_time;

	};

	inline void update_collision_status(int unitID, bool colliding) {
		m_units_colliding[unitID] = colliding;
	};

	inline bool get_collision_status(int unitID) {
		return m_units_colliding[unitID];
	};

	void check_core_collisions(Unit&);

	inline void get_collisions(int unitID) {
		m_grid->add_collisions(m_unitArray[unitID], m_event_queue);
	};

	inline LARGE_INTEGER get_freq() {
		return m_freq;
	};

	//add unit to array of units
	void add_unit(int unitID);

	//get unit object from id
	inline Unit& get_unit(int id) {
		return m_unitArray[id];
	};

	inline int* get_camera() {
		return m_camera;
	};

	inline int* get_look() {
		m_look[cX] = m_camera[cX] + lx;
		m_look[cZ] = m_camera[cZ] + lz;
		return m_look;
	};

	inline float get_num_milliseconds() {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		m_start_time;

		float dif = (now.QuadPart - m_start_time.QuadPart);
		dif = dif / m_freq.QuadPart;
		return dif*1000;
	};

	inline Box& get_box(Unit& unit) {
		return m_grid->get_box(unit);
	}

	inline void add_out_of_container_box_event(Unit& unit, Box& box) {
		Event new_event = m_grid->get_next_box_event(unit);
		m_event_queue.emplace(new_event);
	};

	inline void add_to_queue(Event new_event) {
		m_event_queue.emplace(new_event);
	};

	void change_view(int);
	
	void draw_unit(int);
	
	void send_message(int, int);
	
	std::vector<int> get_box_neighbors(int);
};

#endif /*Environment_h*/