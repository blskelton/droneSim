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
	bool x = QueryPerformanceCounter(&m_start_time);
	bool useQPC = QueryPerformanceFrequency(&m_freq);
	float m_prev_loop_end_time;
	float m_curr_loop_start_time;
	std::array<Unit, global_num_units> m_unitArray;
	int m_camera[3];
	int m_look[3];
	int m_view;
	Boxes m_grid;
	int max_queue_size = 0;
	//for random message delay
	std::random_device m_rd;
	//for message counting
	int m_message_counter[global_num_units][global_num_units];

public:
	Environment();

	inline void init_prev_end_timestamp() {
		m_prev_loop_end_time = get_num_milliseconds();
	};

	inline float get_time() {
		return m_curr_loop_start_time;
	}

	inline int get_message_delay() {
		//move elsewhere?
		std::mt19937 e2(m_rd());
		std::normal_distribution<> distribution(100, 100);

		int delay = abs(distribution(e2)) * 10;
		//int vals[100];
		//for (int i = 0; i < 100; i++) {
			//vals[i] = ((int)(abs(distribution(e2)) * 10));
		//}		

		return delay;
	};

	inline void recalculate_collisions(Unit& unit) {
		//call when unit changes speed or direction
		
		//increment unit age by 1 to invalidate old events
		unit.increment_age();
		//add new unit collision events
		m_grid.add_collisions(unit, m_event_queue);
		//add new box transfer event
		m_event_queue.emplace(m_grid.get_next_box_event(unit));
	};

	inline void recalculate_uc_collision(Unit& unit, int id) {
		m_grid.generate_collision_event(unit, id, m_event_queue);
	};

	inline void process() {
		//update loop time
		m_curr_loop_start_time = get_num_milliseconds();
		float last_milestone = m_prev_loop_end_time;
		float loop_time = (m_curr_loop_start_time - m_prev_loop_end_time);

		//for testing
		Event lastEvent = {};
		int counter = 0;

		//assert((m_curr_loop_start_time - m_prev_loop_end_time) > 0);

		if (m_event_queue.size() > 0) {
			while (m_event_queue.top().timestamp <= m_curr_loop_start_time) {
				Event currEvent = m_event_queue.top(); //make event const?
				if (currEvent.age == m_unitArray[currEvent.idA].get_age() || currEvent.tag == ACTION_EVENT || currEvent.tag == MESSAGE_EVENT) {
					//assert((currEvent.timestamp>=last_milestone));
					float t = currEvent.timestamp - last_milestone;
					if (t > 0) {
						for (int i = 0; i < global_num_units; i++) {
							m_unitArray[i].process(t);
							//just update location, don't undergo entire processing
						}
					}
					if (currEvent.tag == UC_EVENT) { //unit collision event
						m_grid.handle_uc_event(currEvent, m_unitArray[currEvent.idA], m_unitArray[currEvent.idB], m_event_queue);
						m_unitArray[currEvent.idA].set_color(1, 0, 0);
					}
					if (currEvent.tag == BOX_EVENT) {
						m_grid.handle_box_event(currEvent, m_unitArray[currEvent.idA], m_event_queue);
					}
					if (currEvent.tag == ACTION_EVENT) {
						m_unitArray[currEvent.idA].handle_action_event();
					}
					//if (currEvent.tag == MESSAGE_EVENT) {
						//m_unitArray[currEvent.idB].accept_message(currEvent.message);
					//}

					last_milestone += t;

					if (lastEvent.tag == currEvent.tag) {
						if (lastEvent.idA == currEvent.idA) {
							if (lastEvent.idB == currEvent.idB) {
								if (lastEvent.timestamp == currEvent.timestamp) {
									counter++;
								}
							}
						}
					}
					lastEvent = currEvent;
					if (m_event_queue.size() == 0) {
						return;
					}
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
		m_prev_loop_end_time = m_curr_loop_start_time;

		/*if (m_event_queue.size() > max_queue_size) {
			max_queue_size = m_event_queue.size();
			printf("%d\n", max_queue_size);
		}*/

	};

	inline void get_collisions(int unitID) {
		m_grid.add_collisions(m_unitArray[unitID], m_event_queue);
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

	void change_view(int);
	
	void draw_unit(int);
	
	void test_send(int, int);
	
	std::vector<int> get_box_neighbors(int);
};

#endif /*Environment_h*/