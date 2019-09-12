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

//forward declaration
class Environment;

class Unit {
private:
	//unit id
	int m_id;
	queue<Event> m_message_queues[global_num_units]; //unprocessed messages
	vector<unordered_map<int, vector<Message>>> m_sorted_messages; //processed messages
	int m_msgsReceived;
	float m_color[3];

	bool m_in_container;
	bool m_stopped;
	bool m_core_collided;
	int m_status; //0 - normal; 1 - paused, waiting for collision avoidance recalculation; 2 - paused at goal; 3 - core collision

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

	int m_assignment_id;

	//outer radius
	float m_buffer_radius = DEFAULT_RADIUS * 1.5;

	//container dimensions
	int m_containerX;
	int m_containerY;
	int m_containerZ;
	int m_farZ;

	//number of changes in speed or direction a unit has seen
	int m_age = 0;

public:
	static constexpr float DEFAULT_RADIUS = 0.25;

	static constexpr int RESUME_TOWARDS_DESTINATION = 0;
	static constexpr int REATTEMPT_COLLISION_AVOIDANCE = 1;

	Unit(int);

	Unit();

	virtual ~Unit();

	//returns true if an out-of-container unit is moving towards container
	bool heading_back();

	//set destination to given parameters
	void set_destination(float, float, float);

	//returns container status
	inline bool get_container_bool() {
		return m_in_container;
	};

	//attempt to find an assignment
	void seek_assignment();

	//stops unit and updates color upon core collision
	inline void perform_core_collision() {
		m_status = 3;
		m_core_collided = true;
		stop_unit();
		set_color(RED);
	}

	//update direction using collision avoidance protocol
	bool collision_avoidance(int);

	//update variables, direction upon reentering container
	void reenter_container();

	//increment age to invalidate old events
	inline void increment_age() {
		m_age++;
	};

	//add message to message queue
	inline void accept_message(Event messageEvent) {
		int senderID = messageEvent.data.messageEvent.message.get_sender();
		m_message_queues[senderID].emplace(messageEvent);
	};

	//returns unit age
	inline int get_age() {
		return m_age;
	};

	//normalizes direction
	inline void normalize_direction() {
		float sum = abs(m_direction[cX]) + abs(m_direction[cY]) + abs(m_direction[cZ]);
		m_direction[cX] = m_direction[cX] / sum;
		m_direction[cY] = m_direction[cY] / sum;
		m_direction[cZ] = m_direction[cZ] / sum;
	}

	//end collision avoidance and resume path towards destination
	void handle_action_event();

	//tests to see if two units are colliding
	inline bool is_colliding(Unit& secondUnit, bool core_collision) { 
		float x1, y1, z1, x2, y2, z2, r1, r2;
		x1 = m_location[cX];
		y1 = m_location[cY];
		z1 = m_location[cZ];
		float* location = secondUnit.get_location();
		x2 = location[cX];
		y2 = location[cY];
		z2 = location[cZ];

		if (core_collision) { //testing for actual unit collision
			r1 = m_radius;
			r2 = secondUnit.get_radius();
		}
		else { //testing for overlap on buffer radii
			r1 = m_buffer_radius;
			r2 = secondUnit.get_buffer_radius();
		}
		float left = std::abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2)*(z1 - z2));
		float right = (r1 + r2) * (r1 + r2);

		return (left<right);
	};

	//returns goal speed
	inline float get_goal_speed() {
		return m_goal_speed;
	};

	//accesses global environment to get current time, returns
	float get_time();

	//determines time at which two units will collide
	float get_uc_timestamp(int);

	//determines time at which two units will collide with specified direction and location
	float get_direction_intersection_time(int, float(&direction_array)[3], bool);

	//consider contents of message and add to sorted messages
	void process_msg(Message);

	//determines intersection timestamp
	float calculate_intersection_time(float);

	//determines eta timestamp
	float calculate_eta(float);

	//returns the number of messages the unit has received
	inline int get_num_messages() {
		return m_msgsReceived;
	};	

	//updates location
	void update_location(float);

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

	//returns destination
	inline float* get_destination() {
		return m_destination;
	};

	//returns speed
	inline float get_speed() {
		return m_speed;
	};

	//returns id
	inline int get_id() {
		return m_id;
	};

	//returns bufferRadius
	inline float get_buffer_radius() {
		return m_buffer_radius;
	};

	//returns distance to a given point
	float calc_distance_to_site(float, float, float);

	//rewrites location variable
	inline void set_location(float x, float y, float z) {
		m_location[0] = x;
		m_location[1] = y;
		m_location[2] = z;
	};

	//updates speed using acceleration/deceleration factor and goal speed
	void update_speed();

	//returns core_collided var
	inline bool is_core_collided() {
		return m_core_collided;
	};

	//calculates unit-container collision
	void check_container_collision();

	//given two colliding units, enacts collision protocol (currently calculates new velocity for both)
	void perform_unit_collision(Unit&, priority_queue<Event, vector<Event>, myEventComparator>&);

	//generates a new random destination
	void init_destination();

	//creates and adds an eta event to the event queue
	void generate_eta_event(priority_queue<Event, vector<Event>, myEventComparator>&);

	//determines if a unit has reached its destination and either handles the destination or regenerates eta event
	void handle_eta_event(priority_queue<Event, vector<Event>, myEventComparator>&, float);

	//resumes motion towards next destination and adds eta event to the queue
	void handle_wait_event(priority_queue<Event, vector<Event>, myEventComparator>&, int);

	//sets direction towards destination or in a random direction, then normalizes velocity
	void set_direction();

	//randomizes direction
	void randomize_direction();

	//sets unit speed
	inline void stop_unit() {
		m_speed = 0;
		m_stopped = true;
	};

	//returns acceleration
	inline float get_acceleration() {
		return m_acceleration;
	};

	//populates array with normalized random direction
	inline void get_random_direction(float(&direction_array)[3]) {
		direction_array[cX] = ((float)(rand() % 201)) - 100;
		direction_array[cY] = ((float)(rand() % 201)) - 100;
		direction_array[cZ] = ((float)(rand() % 201)) - 100;

		float sum = abs(direction_array[cX]) + abs(direction_array[cY]) + abs(direction_array[cZ]);

		direction_array[cX] = direction_array[cX] / sum;
		direction_array[cY] = direction_array[cY] / sum;
		direction_array[cZ] = direction_array[cZ] / sum;
	};

	//updates location variable with a random position within the container
	void randomize_location();

	//passes message to environment to add to the event queue
	void send(Message, int);

	//receive a message from a given or random unit
	void recv(int);

	//process for a given amount of time
	void process(float);

	//perform any user-specified processes
	virtual void user_process();

};
#endif /* Unit_h */