#include <vector>
#include <unordered_map>
#include <cmath>

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

	float temp_direction[3];
	int m_msgsReceived;
	float m_color[3];

	bool m_in_container = true;
	bool m_heading_towards_container = false;

	//current location of unit
	float m_location[3];

	//inner radius of unit
	float m_radius = DEFAULT_RADIUS;

	//normalized direction vector
	float m_direction[3];

	//speed of unit
	float m_speed;

	//destination point of unit
	float m_dest[3];

	//unit id
	int m_id;

	//outer radius
	float m_bufferRadius = m_radius*3;

	//length of one side of square container
	int m_containerSize;

	//boolean representing whether or not a unit has a destination point
	bool m_hasDest;

	//number of changes in speed or direction a unit has seen
	int m_age = 0;

public:
	static constexpr float DEFAULT_RADIUS = 0.25;
	//create a unit with an int id and the total number of units
	Unit(int);

	Unit();

	virtual ~Unit();

	inline bool get_container_bool() {
		return m_in_container;
	};

	inline void update_heading_back_var() {
		m_heading_towards_container = true;
	};

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

	inline void handle_action_event() {
		m_direction[cX] = temp_direction[cX];
		m_direction[cY] = temp_direction[cY];
		m_direction[cZ] = temp_direction[cZ];
		if (m_dest) {
			set_direction();
		}
	};

	inline bool is_colliding(Unit secondUnit) {
		float x1, y1, z1, x2, y2, z2, r1, r2;
		x1 = m_location[cX];
		y1 = m_location[cY];
		z1 = m_location[cZ];
		x2 = secondUnit.get_location()[cX];
		y2 = secondUnit.get_location()[cY];
		z2 = secondUnit.get_location()[cZ];

		r1 = m_bufferRadius;
		r2 = secondUnit.get_buffer_radius();

		float left = std::abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2)*(z1 - z2));
		float right = (r1 + r2) * (r1 + r2);
		
		float distance = this->calc_distance_to_site(x2, y2, z2);

		return std::abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)+(z1-z2)*(z1-z2)) < (r1 + r2) * (r1 + r2);
	};

	inline void change_direction(float x, float y, float z) {
		m_direction[cX] = x;
		m_direction[cY] = y;
		m_direction[cZ] = z;
		normalize_direction();
	};

	float unit_collision(int);

	void process_msg(Message);

	float calc_intersection_time(float);

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
	inline void set_color(float red, float green, float blue) {
		m_color[0] = red;
		m_color[1] = green;
		m_color[2] = blue;
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
	inline float* get_dest() {
		return m_dest;
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

	void change_speed(bool);

	//calculates unit-container collision
	void check_container_collision();

	//given two colliding units, enacts collision protocol (currently calculates new velocity for both)
	void perform_unit_collision(Unit&, std::priority_queue<Event, vector<Event>, myEventComparator>&);

	//generates a new random destination
	void init_dest();

	void generate_destination_event(std::priority_queue<Event, vector<Event>, myEventComparator>&);

	//sets direction towards destination or in a random direction, then normalizes velocity
	void set_direction();

	void randomize_direction();

	//sets unit speed
	void set_speed(float);

	void send(Message, int);

	void recv(int);

	void process(float);

	virtual void user_process();

	inline bool has_dest() {
		return m_hasDest;
	};
};
#endif /* Unit_h */