#include <vector>
#include <random>

#include "Unit.h"
#include "Container.h"
#include "Message.h"
#include "Environment.h"
#include "simulator.h"

Unit::Unit() {
}

Unit::Unit(int i) : m_id{i}
{
	for (int j = 0; j < global_num_units; j++) {
		m_message_boxes.emplace_back();
		m_sorted_messages.emplace_back();
	}	
	m_msgsReceived = 0;
	//initialize particle properties
	m_color[0] = 0;
	m_color[1] = 1;
	m_color[2] = 1;

	m_containerSize = globalContainer.get_x_dimension();

	int max = 1000 * (m_containerSize - 2 * m_bufferRadius); //sub out m_radius for m_bufferRadius?

	m_location[0] = (float)(rand() % max) / 1000 - m_containerSize / 2 + m_bufferRadius;
	m_location[1] = (float)(rand() % max) / 1000 - m_containerSize / 2 + m_bufferRadius;
	m_location[2] = (float)(rand() % max) / 1000 - 30 + m_bufferRadius;
	
	m_speed = (float)  0.0075; //distance traveled per ms
	m_hasDest = false;
	//m_radius = 0.5;// (float);// -m_containerSize / 2; 0.25; //make constexpr default radius
	//m_radius = default_radius;
	m_bufferRadius = m_radius * 2;
	set_direction();
}

Unit::~Unit()
{
	m_message_boxes.clear();
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
		if (!m_heading_towards_container) { //not currently heading back towards container
			//flip boolean
			m_in_container = false;
			//reverse direction
			m_direction[cX] = -m_direction[cX];
			m_direction[cY] = -m_direction[cY];
			m_direction[cZ] = -m_direction[cZ];
			//age unit
			increment_age(); //later on, call environment.recalculate collisions
			//add new box collision
			globalEnvironment.add_out_of_container_box_event(*this, current_box);
			//m_heading_towards_container = true;
		}
	}
};

void Unit::reenter_container() {
	//if in container, update bool, randomize direction, update age and collisions
	m_in_container = true;
	randomize_direction();
}

float Unit::calc_intersection_time(float distance) {
	//globalEnvironment.init_prev_end_timestamp();
	float time = distance / m_speed;
	time = max(time, 0.001);
	//process(time);
	m_location;
	return globalEnvironment.get_time() + time; //increment by 1 to account for floating point errors
}

void Unit::change_speed(bool is_accelerating) {
	if (is_accelerating && m_speed < .01) {
		m_speed += (float)0.00001;
		globalEnvironment.recalculate_collisions(*this);
	}
	if (!is_accelerating && m_speed > 0.0) {
		m_speed -= (float)0.00001;
		globalEnvironment.recalculate_collisions(*this);
	}
}

void Unit::set_direction() {
	if (m_hasDest) {
		//incorporate location
		m_direction[0] = m_dest[0] - m_location[0];
		m_direction[1] = m_dest[1] - m_location[1];
		m_direction[2] = m_dest[2] - m_location[2];
	}
	else {
		m_direction[0] = ((float)(rand() % 201))-100;
		m_direction[1] = ((float)(rand() % 201))-100;
		m_direction[2] = ((float)(rand() % 201))-100;
	}
	normalize_direction();
	globalEnvironment.recalculate_collisions(*this);
}

void Unit::init_dest() {
	//generate new random destination
	m_radius;
	m_bufferRadius;
	
	int max = 1000 * (m_containerSize - 2 * (m_radius * 3)); //sub out m_radius for m_bufferRadius?
	m_dest[0] = (float)(rand() % max) / 1000 - m_containerSize / 2 + (m_radius * 3);
	m_dest[1] = (float)(rand() % max) / 1000 - m_containerSize / 2 + (m_radius * 3);
	m_dest[2] = (float)(rand() % max) / 1000 - 30 + m_radius * 3;

	m_hasDest = true;
	set_direction();
}

void Unit::set_dest(float x_dest, float y_dest, float z_dest) {
	m_dest[0] = x_dest;
	m_dest[1] = y_dest;
	m_dest[2] = z_dest;
	m_hasDest = true;
	set_direction();
}

float Unit::calc_distance_to_site(float site_x, float site_y, float site_z) {
	//get particle location
	float m_x = m_location[0];
	float m_y = m_location[1];
	float m_z = m_location[2];

	//get other location
	float x = (m_x - site_x);
	float y = (m_y - site_y);
	float z = (m_z - site_z);

	//use distance formula
	float distance = sqrt(abs(x*x + y * y + z * z));
	return distance;
}

void Unit::set_speed(float speed) {
	m_speed = speed;
	globalEnvironment.recalculate_collisions(*this);
}

void Unit::check_container_collision() {
	//get particle location
	float particleX = m_location[0];
	float particleY = m_location[1];
	float particleZ = m_location[2];
	
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
		globalEnvironment.recalculate_collisions(*this);
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
	normalize_direction(); //only call when direction is changed?
	calc_next_location(loop_time);
}

void Unit::user_process() {
	if (m_dest && calc_distance_to_site(m_dest[cX], m_dest[cY], m_dest[cZ]) < 0.1) {
		init_dest();
	}
	for (int i = 0; i < global_num_units; i++) {
		globalEnvironment.test_send(m_id, i);
	}
	
	recv(-1);
	//change_speed(true);
}

float Unit::unit_collision(int unitID) {
	if (unitID == m_id) {
		return -1;
	}
	Unit& unitB = globalEnvironment.get_unit(unitID);
	myVector locationVector = {m_location[cX]-unitB.get_location()[cX],
		m_location[cY] - unitB.get_location()[cY],
		m_location[cZ] - unitB.get_location()[cZ] };
	myVector sumDirectionVector = {m_direction[cX] - unitB.get_direction()[cX],
		m_direction[cY] - unitB.get_direction()[cY],
		m_direction[cZ] - unitB.get_direction()[cZ] };
	double radiusSum = m_bufferRadius + unitB.get_buffer_radius();

	double c = locationVector.dot_product(locationVector) - pow(radiusSum, 2);
	if (c < 0) {
		return -1;
	}

	float a = sumDirectionVector.dot_product(sumDirectionVector);
	float b = sumDirectionVector.dot_product(locationVector);
	if (b >= 0) {
		return -1;
	}
	float d = b * b - a * c;
	if (d < 0) {
		return -1;
	}

	float t = (-b - sqrt(d)) / a;
	return t/m_speed+globalEnvironment.get_time();
}

void Unit::randomize_direction() {
	m_direction[0] = ((float)(rand() % 201)) - 100;
	m_direction[1] = ((float)(rand() % 201)) - 100;
	m_direction[2] = ((float)(rand() % 201)) - 100;
	normalize_direction();
	globalEnvironment.recalculate_collisions(*this);
}

void Unit::perform_unit_collision(Unit &unit, std::priority_queue<Event, vector<Event>, myEventComparator> &pq) {
	bool colliding = false;
	if (unit.get_id() != m_id) {
		if (is_colliding(unit)) {
			colliding = true;
			float time = m_bufferRadius*2 / m_speed; //number of ms to travel buffer diameter
			time += globalEnvironment.get_time();
			
			ActionEvent data = { m_id };
			Event new_event = { ACTION_EVENT, time, { data } };
			pq.emplace(new_event);

			temp_direction[cX] = m_direction[cX];
			temp_direction[cY] = m_direction[cY];
			temp_direction[cZ] = m_direction[cZ];

			randomize_direction();
			globalEnvironment.recalculate_collisions(*this);
		}
	}
	if (!colliding) {
		//attempt to generate a new collision event for the two units
		globalEnvironment.recalculate_uc_collision(*this, unit.get_id());
	}
}