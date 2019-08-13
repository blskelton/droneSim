#include <vector>
#include <random>

#include "Unit.h"
#include "Container.h"
#include "Message.h"
#include "Environment.h"
#include "simulator.h"

Unit::Unit() {
}

Unit::Unit(int i) : m_id{ i }, m_acceleration{ 0.001 }, m_saw_collision(-1), m_goal_speed(0.0075)
{
	for (int j = 0; j < global_num_units; j++) {
		m_sorted_messages.emplace_back();
	}	
	m_msgsReceived = 0;
	m_color[cX] = GREEN[cX];
	m_color[cY] = GREEN[cY];
	m_color[cZ] = GREEN[cZ];

	m_containerX = globalContainer.get_x_dimension();
	m_containerY = globalContainer.get_y_dimension();
	m_containerZ = globalContainer.get_z_dimension();

	m_in_container = true;
	m_stopped = false;
	m_core_collided = false;

	int maxX = 1000 * (m_containerX - 2 * m_bufferRadius);
	int maxY = 1000 * (m_containerY - 2 * m_bufferRadius);
	int maxZ = 1000 * (m_containerZ - 2 * m_bufferRadius);

	m_location[cX] = (float)(rand() % maxX) / 1000 - m_containerX / 2 + m_bufferRadius;
	m_location[cY] = (float)(rand() % maxY) / 1000 - m_containerY / 2 + m_bufferRadius;
	m_location[cZ] = (float)(rand() % maxZ) / 1000 - 30 + m_bufferRadius;
	
	m_speed = (float)0.00; //distance traveled per ms
	m_bufferRadius = m_radius * 2;
	init_destination();
}

Unit::~Unit()
{
	m_sorted_messages.clear();
}

void Unit::calc_next_location(float loop_time) {
	float t = loop_time;
	
	m_location[cX] = m_location[cX] + m_direction[cX] * m_speed * t;
	m_location[cY] = m_location[cY] + m_direction[cY] * m_speed * t;
	m_location[cZ] = m_location[cZ] + m_direction[cZ] * m_speed * t;
	if (m_location[cX] < -15 || m_location[cX] > 15) {
		m_in_container = false;
	}
	if (m_location[cY] < -15 || m_location[cY] > 15) {
		m_in_container = false;
	}
	if (m_location[cZ] < -35 || m_location[cZ] > 5) {
		m_in_container = false;
	}
	
	Box current_box = globalEnvironment.get_box(*this);
	if (!current_box.in_container) { //has escaped the container
		//bool var = heading_back(current_box);
		if (!heading_back()) { //not currently heading back towards container
			//flip boolean
			m_in_container = false;
			//reverse direction
			m_direction[cX] = -m_direction[cX];
			m_direction[cY] = -m_direction[cY];
			m_direction[cZ] = -m_direction[cZ];
			//age unit
			globalEnvironment.recalculate_collisions(*this); //reversed direction, must recalculate
			//add new box collision
			globalEnvironment.add_out_of_container_box_event(*this, current_box);
			//m_heading_towards_container = true;
		}
	}
};

bool Unit::heading_back() {
	//escaped in what direction(s)
	int direction_status[3] = { 0,0,0 };
	int leftX = globalContainer.get_leftX();
	int bottomY = globalContainer.get_bottomY();
	int farZ = globalContainer.get_farZ();
	int rightX = globalContainer.get_rightX();
	int topY = globalContainer.get_topY();
	int closeZ = globalContainer.get_closeZ();
	if (m_location[cX] < leftX) {
		direction_status[cX] = -1;
	}
	if (m_location[cY] < bottomY) {
		direction_status[cY] = -1;
	}
	if (m_location[cZ] < farZ) {
		direction_status[cZ] = -1;
	}
	if (m_location[cX] > rightX) {
		direction_status[cX] = 1;
	}
	if (m_location[cY] > topY) {
		direction_status[cY] = 1;
	}
	if (m_location[cZ] > closeZ) {
		direction_status[cZ] = 1;
	}
	for (int i = 0; i < 3; i++) {
		if (direction_status[i] < 0) {
			if (m_direction[i] < 0) {
				return false;
			}
		}
		if (direction_status[i] > 0) {
			if (m_direction[i] > 0) {
				return false;
			}
		}
	}
	return true;
}

void Unit::reenter_container() {
	//if in container, update bool, randomize direction, update age and collisions
	m_in_container = true;
	randomize_direction();
}

float Unit::calc_intersection_time(float distance) {
	float time = distance / m_speed;
	time = max(time, 0.001); //ensure that timestamp does not equal current time
	return globalEnvironment.get_time() + time;
}

float Unit::calc_arrival_time(float distance) {
	float time = distance / m_goal_speed;
	time = max(time, 0.001); //ensure that timestamp does not equal current time
	return globalEnvironment.get_time() + time;
}

void Unit::update_speed() {
	if (!m_stopped) {
		if (m_speed < m_goal_speed) {
			m_speed += m_acceleration;
		}
		if (m_speed > m_goal_speed) {
			m_speed -= m_acceleration;
		}
		//globalEnvironment.recalculate_collisions(*this); //change of speed, must recalculate
	}
}


void Unit::set_direction() {
	float temp_directionX = m_destination[cX] - m_location[cX];
	float temp_directionY = m_destination[cY] - m_location[cY];
	float temp_directionZ = m_destination[cZ] - m_location[cZ];

	if (temp_directionX != m_direction[cX] || temp_directionY != m_direction[cY] || temp_directionZ != m_direction[cZ]) { //change in dir
		m_direction[cX] = m_destination[cX] - m_location[cX];
		m_direction[cY] = m_destination[cY] - m_location[cY];
		m_direction[cZ] = m_destination[cZ] - m_location[cZ];

		normalize_direction();
		globalEnvironment.recalculate_collisions(*this);
	}	
}

void Unit::init_destination() {
	//generate new random destination
	int maxX = 1000 * (m_containerX - 2 * m_bufferRadius);
	int maxY = 1000 * (m_containerY - 2 * m_bufferRadius);
	int maxZ = 1000 * (m_containerZ - 2 * m_bufferRadius);

	m_destination[cX] = (float)(rand() % maxX)/1000  - m_containerX / 2 + m_bufferRadius;
	m_destination[cY] = (float)(rand() % maxY)/1000 - m_containerY / 2 + m_bufferRadius;
	m_destination[cZ] = (float)(rand() % maxZ)/1000  - 30 + m_bufferRadius;

	set_direction();
}

void Unit::generate_eta_event(std::priority_queue<Event, vector<Event>, myEventComparator>& event_queue) {
	//set_direction();
	ETAEvent data = { m_id, m_radius };
	float distance = calc_distance_to_site(m_destination[cX], m_destination[cY], m_destination[cZ]);
	float timestamp = calc_arrival_time(distance);

	Event destination_event = { ETA_EVENT, timestamp, {data} };
	event_queue.emplace(destination_event);
}

void Unit::handle_eta_event(std::priority_queue<Event, vector<Event>, myEventComparator>& pq, float epsilon) {
	float distance_to_destination = calc_distance_to_site(m_destination[cX], m_destination[cY], m_destination[cZ]);
	if (distance_to_destination < epsilon) {
		stop_unit();
		set_color(WHITE);
		float wait_time = globalEnvironment.get_message_delay(); //rename to reflect multi-purpose?
		WaitEvent data = { m_id };
		Event waitEvent = { WAIT_EVENT, globalEnvironment.get_time() + wait_time, {data} };
		pq.emplace(waitEvent);
		m_stopped = true;
	}
	else { //regenerate destination event - event is in date but not accurate within epsilon
		generate_eta_event(pq);
	}
}

void Unit::handle_wait_event(std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
	if (!m_core_collided) {
		set_color(GREEN);
		m_stopped = false;
		init_destination();
		//set_speed((float)0.0075);
		generate_eta_event(pq);
	}
}

void Unit::set_dest(float x_dest, float y_dest, float z_dest) {
	m_destination[cX] = x_dest;
	m_destination[cY] = y_dest;
	m_destination[cZ] = z_dest;
	set_direction();
}

float Unit::calc_distance_to_site(float site_x, float site_y, float site_z) {
	//get particle location
	float m_x = m_location[cX];
	float m_y = m_location[cY];
	float m_z = m_location[cZ];

	//get other location
	float x = (m_x - site_x);
	float y = (m_y - site_y);
	float z = (m_z - site_z);

	//use distance formula
	float distance = sqrt(abs(x*x + y * y + z * z));
	return distance;
}

void Unit::check_container_collision() {
	//get particle location
	float particleX = m_location[cX];
	float particleY = m_location[cY];
	float particleZ = m_location[cZ];
	
	int leftX = globalContainer.get_leftX();
	int rightX = globalContainer.get_rightX();
	int bottomY = globalContainer.get_bottomY();
	int topY = globalContainer.get_topY();
	int closeZ = globalContainer.get_closeZ();
	int farZ = globalContainer.get_farZ();

	bool change_in_direction = false;

	if (particleX - m_bufferRadius <= leftX && m_direction[0] < 0) {
		m_direction[0] = -m_direction[0];
		change_in_direction = true;
	}
	if (particleX + m_bufferRadius >= rightX && m_direction[0] > 0) {
		m_direction[0] = -m_direction[0];
		change_in_direction = true;
	}
	if (particleY - m_bufferRadius <= bottomY && m_direction[1] < 0) {
		m_direction[1] = -m_direction[1];
		change_in_direction = true;
	}
	if (particleY + m_bufferRadius >= topY && m_direction[1] > 0) {
		m_direction[1] = -m_direction[1];
		change_in_direction = true;
	}
	if (particleZ - m_bufferRadius <= farZ && m_direction[2] < 0) {
		m_direction[2] = -m_direction[2];
		change_in_direction = true;
	}
	if (particleZ + m_bufferRadius >= closeZ && m_direction[2] > 0) {
		m_direction[2] = -m_direction[2];
		change_in_direction = true;
	}
	if (change_in_direction) {
		//m_speed = 0.0;
		globalEnvironment.recalculate_collisions(*this); //reversed direction to stay in container
	}
}

void Unit::recv(int senderID = -1) {
	int num_units = global_num_units;
	if (senderID == -1) {
		senderID = rand() % num_units;
	}
	if (m_message_queues[senderID].size() > 0) {
		Event event = m_message_queues[senderID].front();
		float timestamp = event.timestamp;
		while (timestamp <= globalEnvironment.get_time()) {
			process_msg(event.data.messageEvent.message);
			m_message_queues[senderID].pop();
			if (m_message_queues[senderID].size() > 0) {
				event = m_message_queues[senderID].front();
				timestamp = event.timestamp;
			}
			else {
				timestamp = INFINITY;
			}
		}
	}
}

void Unit::set_speed(float speed) { //try to replace with acceleration and deceleration
	m_speed = speed;
	globalEnvironment.recalculate_collisions(*this); //change of speed possible
};

void Unit::process_msg(Message message) {
	//get senderID and tag from message
	int msg_tag = message.get_tag();
	int sender_id = message.get_sender();
	m_sorted_messages[sender_id][msg_tag].emplace_back(message);
	m_msgsReceived++;
}

void Unit::send(Message message, int recvID) {
	float timestamp = globalEnvironment.get_message_delay();
	timestamp += globalEnvironment.get_time();

	MessageEvent data = { m_id, recvID, message };
	Event message_event = { MESSAGE_EVENT, timestamp, {data} };

	Unit& recv = globalEnvironment.get_unit(recvID);
	recv.accept_message(message_event);
}

void Unit::process(float loop_time) {
	//normalize_direction(); //only call when direction is changed?
	calc_next_location(loop_time);
	//check for core collision
	globalEnvironment.check_core_collisions(*this);
	
}

void Unit::user_process() {
	globalEnvironment.send_message(m_id, rand() % global_num_units);
	recv(-1);
	m_goal_speed = (float)0.0075;
}

void Unit::handle_action_event() {
	if (!m_core_collided) {
		globalEnvironment.update_collision_status(m_id, false);
		set_color(GREEN);
		set_direction();
	}
};

void Unit::randomize_location() {
	int maxX = 1000 * (m_containerX - 2 * m_bufferRadius);
	int maxY = 1000 * (m_containerY - 2 * m_bufferRadius);
	int maxZ = 1000 * (m_containerZ - 2 * m_bufferRadius);

	m_location[cX] = (float)(rand() % maxX) / 1000 - m_containerX / 2 + m_bufferRadius;
	m_location[cY] = (float)(rand() % maxY) / 1000 - m_containerY / 2 + m_bufferRadius;
	m_location[cZ] = (float)(rand() % maxZ) / 1000 - 30 + m_bufferRadius;
}

float Unit::unit_collision(int unitID, bool test) {
	if (unitID == m_id) {
		return -1;
	}
	Unit& unitB = globalEnvironment.get_unit(unitID);
	//vector between unit centers
	myVector locationVector = { m_location[cX] - unitB.get_location()[cX],
		m_location[cY] - unitB.get_location()[cY],
		m_location[cZ] - unitB.get_location()[cZ] };
	//difference in velocity vector (try incorporating speed?)
	myVector sumDirectionVector = { m_direction[cX] - unitB.get_direction()[cX],
		m_direction[cY] - unitB.get_direction()[cY],
		m_direction[cZ] - unitB.get_direction()[cZ] };

	myVector velocityVectorA = {m_direction[cX], m_direction[cY], m_direction[cZ]};
	velocityVectorA = velocityVectorA.scalar_mult(m_speed);
	myVector velocityVectorB = {unitB.get_direction()[cX], unitB.get_direction()[cY], unitB.get_direction()[cZ]};
	velocityVectorB = velocityVectorB.scalar_mult(unitB.get_speed());

	myVector velocityDifVector = velocityVectorA.dif(velocityVectorB);

	//sum of radii
	float radiusSum = m_bufferRadius + unitB.get_buffer_radius();

	//distance minus buffer radius sum squared
	double c = locationVector.dot_product(locationVector) - pow(radiusSum, 2);
	if (c < 0) { //if negative, already overlapping
		stop_unit();
		unitB.stop_unit();
		return -1;
	}

	float a = velocityDifVector.dot_product(velocityDifVector);
	float b = velocityDifVector.dot_product(locationVector);
	if (b >= 0) { //not moving towards eachother 
		return -1;
	}
	//maybe replace c with dot of location vector with itself?
	float d = b * b - a * c;
	if (d < 0) { //if negative, "no real roots"
		return -1;
		//d = 0;
	}

	float t = (-b - sqrt(d)) / a;
	if (test) {
		process(t);
		unitB.process(t);
		bool bufferCollision = is_colliding(unitB, false);
		bool coreCollision = is_colliding(unitB, true);
		float distance = this->calc_distance_to_site(unitB.get_location()[cX], unitB.get_location()[cY], unitB.get_location()[cZ]);
	}
	

	return t+globalEnvironment.get_time(); //add /speed back in
	
}

void Unit::randomize_direction() {
	m_direction[cX] = ((float)(rand() % 201)) - 100;
	m_direction[cY] = ((float)(rand() % 201)) - 100;
	m_direction[cZ] = ((float)(rand() % 201)) - 100;
	normalize_direction();
	globalEnvironment.recalculate_collisions(*this); //change in direction
}

Event Unit::perform_unit_collision(Unit &unit, std::priority_queue<Event, vector<Event>, myEventComparator> &pq) {
	bool colliding = false;
	if (unit.get_id() != m_id) {
		if (is_colliding(unit, false)) { //check for buffer radius collision
			m_saw_collision = unit.get_id();
			set_color(YELLOW);
			colliding = true;
			float time = m_bufferRadius*2 / m_speed; //number of ms to travel buffer diameter
			time += globalEnvironment.get_time();
			
			ActionEvent data = { m_id };
			Event new_event = { ACTION_EVENT, time, { data } };
			pq.emplace(new_event);

			stop_unit();
			unit.stop_unit();

			//randomize_direction();

			m_direction[cX] = -m_direction[cX];
			m_direction[cY] = -m_direction[cY];
			m_direction[cZ] = -m_direction[cZ];
			/*if (m_id > unit.get_id()) {
				m_direction[cX] = 1;
				m_direction[cY] = 1;
				m_direction[cZ] = 1;
				normalize_direction();
			}
			else {
				m_direction[cX] = -1;
				m_direction[cY] = -1;
				m_direction[cZ] = -1;
				normalize_direction();
			}*/

			globalEnvironment.update_collision_status(m_id, true);
			globalEnvironment.update_collision_status(unit.get_id(), true);
			globalEnvironment.recalculate_collisions(*this); //change in direction from collision avoidance
			ActionEvent newdata = { 1 };
			Event avoidance = { -1, globalEnvironment.get_time(), newdata };
			return avoidance;
		}
	}
	if (!colliding) {
		//attempt to generate a new collision event for the two units
		//globalEnvironment.recalculate_uc_collision(*this, unit.get_id());
		//age unit and recalculate
		globalEnvironment.recalculate_uc_collision(m_id, unit.get_id()); //substitute to age and recalculate. instead, clear curr collisions and recalculate?
		ActionEvent newdata = { -1 };
		Event avoidance = { -1, globalEnvironment.get_time(), newdata };
		return avoidance;
	}
}