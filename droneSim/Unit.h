#include <vector>
#include <unordered_map>
#include <cmath>
#include <queue>

#include "Container.h"
#include "Message.h"
#include "Event.h"

#ifndef Unit_h
#define Unit_h

using std::vector;
using std::queue;
using std::unordered_map;
using std::priority_queue;

class Environment;

class Unit {
private:
	queue<Event> m_message_queues[global_num_units];
	vector<unordered_map<int, vector<Message>>> m_sorted_messages;
	int m_msgsReceived;
	//bool m_colliding_units[global_num_units];
	float m_color[3];

	bool m_in_container;
	bool m_stopped;
	bool m_core_collided;

	int m_saw_collision;

	//current location of unit
	float m_location[3];

	//inner radius of unit
	float m_radius = DEFAULT_RADIUS;

	//normalized direction vector
	float m_direction[3];

	//speed of unit
	float m_speed;

	//acceleration
	float m_acceleration;

	//goal speed
	float m_goal_speed;

	//destination point of unit
	float m_destination[3];

	//unit id
	int m_id;

	//outer radius
	float m_bufferRadius = m_radius*3;

	//container dimensions
	int m_containerX;
	int m_containerY;
	int m_containerZ;

	//number of changes in speed or direction a unit has seen
	int m_age = 0;

public:
	static constexpr float DEFAULT_RADIUS = 0.25;
	//create a unit with an int id and the total number of units
	Unit(int);

	Unit();

	virtual ~Unit();

	bool heading_back();

	inline bool get_container_bool() {
		return m_in_container;
	};

	inline void perform_core_collision() {
		m_saw_collision;
		m_core_collided = true;
		stop_unit();
		set_color(RED);
	}

	void reenter_container();

	inline void increment_age() {
		m_age++;
	};

	inline void accept_message(Event messageEvent) {
		int senderID = messageEvent.data.messageEvent.message.get_sender();
		m_message_queues[senderID].emplace(messageEvent);
	};

	inline int get_age() {
		return m_age;
	};

	inline void normalize_direction() {
		float sum = abs(m_direction[cX]) + abs(m_direction[cY]) + abs(m_direction[cZ]);
		m_direction[cX] = m_direction[cX] / sum;
		m_direction[cY] = m_direction[cY] / sum;
		m_direction[cZ] = m_direction[cZ] / sum;
	}

	void handle_action_event();

	inline void unprocess(float t) {
		m_location[cX] = m_location[cX] - m_direction[cX] * m_speed * t;
		m_location[cY] = m_location[cY] - m_direction[cY] * m_speed * t;
		m_location[cZ] = m_location[cZ] - m_direction[cZ] * m_speed * t;
	};

	inline bool is_colliding(Unit& secondUnit, bool core_collision) {
		float x1, y1, z1, x2, y2, z2, r1, r2;
		x1 = m_location[cX];
		y1 = m_location[cY];
		z1 = m_location[cZ];
		x2 = secondUnit.get_location()[cX];
		y2 = secondUnit.get_location()[cY];
		z2 = secondUnit.get_location()[cZ];

		if (core_collision) {
			r1 = m_radius;
			r2 = secondUnit.get_radius();
		}
		else {
			r1 = m_bufferRadius;
			r2 = secondUnit.get_buffer_radius();
		}
		float left = std::abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2)*(z1 - z2));
		float right = (r1 + r2) * (r1 + r2);

		return std::abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)+(z1-z2)*(z1-z2)) < (r1 + r2) * (r1 + r2);
	};

	inline float get_goal_speed() {
		return m_goal_speed;
	};

	float unit_collision(int, bool);

	void process_msg(Message);

	float calc_intersection_time(float);

	float calc_arrival_time(float);

	inline int get_num_messages() {
		return m_msgsReceived;
	};	

	//updates location
	void calc_next_location(float);

	//returns color
	inline float* get_color() {
		return m_color;
	};

	//sets color
	inline void set_color(const float color[3]) {
		m_color[0] = color[0];
		m_color[1] = color[1];
		m_color[2] = color[2];
	};

	//returns radius
	inline float get_radius() {
		return m_radius;
	};

	//returns direction
	inline float* get_direction() {
		return m_direction;
	};

	//returns (does not update) location
	inline float* get_location() {
		return m_location;
	};

	//updates dest variable, hasDest boolean to true, calls set_direction
	void set_dest(float, float, float);

	//returns destination
	inline float* get_destination() {
		return m_destination;
	};

	inline float get_speed() {
		return m_speed;
	};

	//returns id
	inline int get_id() {
		return m_id;
	};

	//returns bufferRadius
	inline float get_buffer_radius() {
		return m_bufferRadius;
	};

	//returns distance to a given point
	float calc_distance_to_site(float, float, float);

	//rewrites location variable
	inline void set_location(float x, float y, float z) {
		m_location[0] = x;
		m_location[1] = y;
		m_location[2] = z;
	};

	void update_speed();

	inline bool is_core_collided() {
		return m_core_collided;
	};

	//calculates unit-container collision
	void check_container_collision();

	//given two colliding units, enacts collision protocol (currently calculates new velocity for both)
	Event perform_unit_collision(Unit&, priority_queue<Event, vector<Event>, myEventComparator>&);

	//generates a new random destination
	void init_destination();

	void generate_eta_event(priority_queue<Event, vector<Event>, myEventComparator>&);

	void handle_eta_event(priority_queue<Event, vector<Event>, myEventComparator>&, float);

	void handle_wait_event(priority_queue<Event, vector<Event>, myEventComparator>&);

	//sets direction towards destination or in a random direction, then normalizes velocity
	void set_direction();

	void randomize_direction();

	//sets unit speed
	inline void stop_unit() {
		m_speed = 0;
		m_stopped = true;
	};

	void randomize_location();

	void set_speed(float);

	void send(Message, int);

	void recv(int);

	void process(float);

	virtual void user_process();

};
#endif /* Unit_h */