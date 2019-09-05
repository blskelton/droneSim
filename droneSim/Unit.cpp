#include <vector>
#include <random>

#include "Unit.h"
#include "Container.h"
#include "Message.h"
#include "Environment.h"
#include "simulator.h"

Unit::Unit() {
}

Unit::Unit(int i) : m_id{ i }, m_speed{ 0.00 }, m_goal_speed(0.0075), m_msgsReceived{ 0 }, m_in_container{ true }, m_stopped{ false }, m_core_collided{ false }
{
	for (int j = 0; j < global_num_units; j++) {
		m_sorted_messages.emplace_back();
	}	
	m_color[cX] = GREEN[cX];
	m_color[cY] = GREEN[cY];
	m_color[cZ] = GREEN[cZ];

	m_containerX = globalContainer.get_x_dimension();
	m_containerY = globalContainer.get_y_dimension();
	m_containerZ = globalContainer.get_z_dimension();
	m_farZ = -(globalContainer.get_farZ());

	int maxX = 1000 * (m_containerX - 2 * m_buffer_radius);
	int maxY = 1000 * (m_containerY - 2 * m_buffer_radius);
	int maxZ = 1000 * (m_containerZ - 2 * m_buffer_radius);

	m_location[cX] = (float)(rand() % maxX) / 1000 - m_containerX / 2 + m_buffer_radius;
	m_location[cY] = (float)(rand() % maxY) / 1000 - m_containerY / 2 + m_buffer_radius;
	m_location[cZ] = (float)(rand() % maxZ) / 1000 - m_farZ + m_buffer_radius;

	
	
	m_acceleration = ((float)(rand() % 9+1)) * 0.0001;

	m_buffer_radius = DEFAULT_RADIUS * 1.5;

	if (packages) {
		Package package = globalEnvironment.get_package_info(m_id);
		set_destination(package.position[cX], package.position[cY]+0.5, package.position[cZ]);
	}
	else {
		init_destination();
	}
}

Unit::~Unit()
{
	m_sorted_messages.clear();
}

void Unit::set_destination(float x, float y, float z) {
	m_destination[cX] = x;
	m_destination[cY] = y;
	m_destination[cZ] = z;
	set_direction();
}

void Unit::update_location(float loop_time) {
	float t = loop_time;

	m_location[cX] = m_location[cX] + m_direction[cX] * m_speed * t;
	m_location[cY] = m_location[cY] + m_direction[cY] * m_speed * t;
	m_location[cZ] = m_location[cZ] + m_direction[cZ] * m_speed * t;

	
	Box current_box = globalEnvironment.get_box(*this);
	if (!current_box.in_container) { //has escaped the container
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
	increment_age();
	randomize_direction();
	globalEnvironment.recalculate_collisions(*this); //commit change in direction
}

float Unit::calc_intersection_time(float distance) {
	float time = distance / m_speed;
	time = max(time, 0.1); //ensure that timestamp does not equal current time
	return globalEnvironment.get_time() + time;
	
}

float Unit::calc_arrival_time(float distance) {
	float time = distance / max(m_goal_speed, m_speed);
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
		if (m_speed == 0) {
			int val = 0;
		}
		m_buffer_radius = (m_radius*1.5) + globalEnvironment.get_longest_process() * m_speed;
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
	int maxX = 1000 * (m_containerX - 2 * m_buffer_radius);
	int maxY = 1000 * (m_containerY - 2 * m_buffer_radius);
	int maxZ = 1000 * (m_containerZ - 2 * m_buffer_radius);

	m_destination[cX] = (float)(rand() % maxX)/1000  - m_containerX / 2 + m_buffer_radius;
	m_destination[cY] = (float)(rand() % maxY)/1000 - m_containerY / 2 + m_buffer_radius;
	m_destination[cZ] = (float)(rand() % maxZ)/1000  - m_farZ + m_buffer_radius;

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
		if (packages) {
			int status = globalEnvironment.get_package_status(m_id);
			if (status == WAITING_FOR_PICKUP){
				globalEnvironment.update_package_status(m_id, IN_TRANSIT);
			}
			if (status == IN_TRANSIT) {
				globalEnvironment.update_package_status(m_id, AT_DESTINATION);
			}
		}
	}
	else { //regenerate destination event - event is in date but not accurate within epsilon
		generate_eta_event(pq);
	}
}

void Unit::handle_wait_event(std::priority_queue<Event, vector<Event>, myEventComparator>& pq) {
	if (!m_core_collided) {
		set_color(GREEN);
		m_stopped = false;
		if (packages) {
			int status = globalEnvironment.get_package_status(m_id);
			if (status == 1) {
				Package package = globalEnvironment.get_package_info(m_id);
				set_destination(package.destination[cX], package.destination[cY]+0.5, package.destination[cZ]);
			}
			if (status == 2) {
				init_destination();
			}
		}
		else {
			init_destination();
		}
		generate_eta_event(pq);
	}
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

	if (particleX - m_buffer_radius <= leftX && m_direction[0] < 0) {
		m_direction[0] = -m_direction[0];
		change_in_direction = true;
	}
	if (particleX + m_buffer_radius >= rightX && m_direction[0] > 0) {
		m_direction[0] = -m_direction[0];
		change_in_direction = true;
	}
	if (particleY - m_buffer_radius <= bottomY && m_direction[1] < 0) {
		m_direction[1] = -m_direction[1];
		change_in_direction = true;
	}
	if (particleY + m_buffer_radius >= topY && m_direction[1] > 0) {
		m_direction[1] = -m_direction[1];
		change_in_direction = true;
	}
	if (particleZ - m_buffer_radius <= farZ && m_direction[2] < 0) {
		m_direction[2] = -m_direction[2];
		change_in_direction = true;
	}
	if (particleZ + m_buffer_radius >= closeZ && m_direction[2] > 0) {
		m_direction[2] = -m_direction[2];
		change_in_direction = true;
	}
	if (change_in_direction) {
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
	float traveled_distance = loop_time * m_speed;
	float max_distance = m_buffer_radius - m_radius;
	//assert(max_distance > traveled_distance);
	update_location(loop_time);
	//check for core collision
	//globalEnvironment.check_core_collisions(*this);
	
}

void Unit::user_process() {
	globalEnvironment.send_message(m_id, rand() % global_num_units);
	recv(-1);
	//m_goal_speed = (float)0.0075;
}

float Unit::get_time() {
	float time = globalEnvironment.get_time();
	return time;
}

void Unit::handle_action_event() {
	if (!m_core_collided) {
		set_color(GREEN);
		set_direction();
	}
};

void Unit::randomize_location() {
	int maxX = 1000 * (m_containerX - 2 * m_buffer_radius);
	int maxY = 1000 * (m_containerY - 2 * m_buffer_radius);
	int maxZ = 1000 * (m_containerZ - 2 * m_buffer_radius);

	m_location[cX] = (float)(rand() % maxX) / 1000 - m_containerX / 2 + m_buffer_radius;
	m_location[cY] = (float)(rand() % maxY) / 1000 - m_containerY / 2 + m_buffer_radius;
	m_location[cZ] = (float)(rand() % maxZ) / 1000 - m_farZ + m_buffer_radius;
}

float Unit::get_direction_intersection_time(int unitID, float(&direction_array)[3], float(&location_array)[3]) {
	if (unitID == m_id) {
		return -1;
	}
	Unit& unitB = globalEnvironment.get_unit(unitID);
	//vector between unit centers
	myVector locationVector = { location_array[cX] - unitB.get_location()[cX],
		location_array[cY] - unitB.get_location()[cY],
		location_array[cZ] - unitB.get_location()[cZ] };
	//difference in velocity vector (try incorporating speed?)
	myVector sumDirectionVector = { direction_array[cX] - unitB.get_direction()[cX],
		direction_array[cY] - unitB.get_direction()[cY],
		direction_array[cZ] - unitB.get_direction()[cZ] };

	myVector velocityVectorA = { direction_array[cX], direction_array[cY], direction_array[cZ] };
	velocityVectorA = velocityVectorA.scalar_mult(m_speed);
	myVector velocityVectorB = { unitB.get_direction()[cX], unitB.get_direction()[cY], unitB.get_direction()[cZ] };
	velocityVectorB = velocityVectorB.scalar_mult(unitB.get_speed());

	myVector velocityDifVector = velocityVectorA.dif(velocityVectorB);

	//sum of radii
	float radiusSum = m_buffer_radius + unitB.get_buffer_radius();

	//distance minus buffer radius sum squared
	double c = locationVector.dot_product(locationVector) - pow(radiusSum, 2);
	if (c < 0) { //if negative, already overlapping
		return 0;
	}

	float a = velocityDifVector.dot_product(velocityDifVector);
	float b = velocityDifVector.dot_product(locationVector);
	if (b >= 0) { //not moving towards eachother 
		return std::numeric_limits<float>::infinity();
	}
	//maybe replace c with dot of location vector with itself?
	float d = b * b - a * c;
	if (d < 0) { //if negative, "no real roots"
		return -1;
		//d = 0;
	}

	float t = (-b - sqrt(d)) / a;

	t = max(0.01, t);

	return t + globalEnvironment.get_time();
}

float Unit::get_uc_timestamp(int unitID) {
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
	float radiusSum = m_buffer_radius + unitB.get_buffer_radius();

	//distance minus buffer radius sum squared
	double c = locationVector.dot_product(locationVector) - pow(radiusSum, 2);
	if (c < 0) { //if negative, already overlapping
		return globalEnvironment.get_time();
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
	
	t = max(0.01, t);

	return t+globalEnvironment.get_time();
}

void Unit::randomize_direction() {
	m_direction[cX] = ((float)(rand() % 201)) - 100;
	m_direction[cY] = ((float)(rand() % 201)) - 100;
	m_direction[cZ] = ((float)(rand() % 201)) - 100;
	normalize_direction();
	//globalEnvironment.recalculate_collisions(*this); //change in direction
}

void Unit::collision_avoidance() {
	float new_direction[3];
	get_random_direction(new_direction);
	//randomize_direction(); //try a different random direction
	float earliest_collision_time = globalEnvironment.get_earliest_collision(*this, m_location, new_direction);
	//randomize direction
	/*randomize_direction();
	//see if any collision in next epsilon ms
	float earliest_collision_time = globalEnvironment.get_earliest_collision(*this);*/
	float current_time = get_time();
	float offset = earliest_collision_time - current_time;
	int counter = 0;
	while (offset < 100) {
		float new_direction[3];
		get_random_direction(new_direction);
		//randomize_direction(); //try a different random direction
		earliest_collision_time = globalEnvironment.get_earliest_collision(*this, m_location, new_direction); 	//see if any collision in next epsilon ms
		offset = earliest_collision_time - current_time;
		counter++;
	}
	if (counter > 0) {
		int val = 0;
	}

	m_direction[cX] = new_direction[cX];
	m_direction[cY] = new_direction[cY];
	m_direction[cZ] = new_direction[cZ];
	return;
}

void Unit::perform_unit_collision(Unit &unit, std::priority_queue<Event, vector<Event>, myEventComparator> &pq) {
	bool colliding = false;
	if (unit.get_id() != m_id) {
		if (is_colliding(unit, false)) { //check for buffer radius collision
			set_color(YELLOW);
			colliding = true;
			float time = m_buffer_radius*2 / m_speed; //number of ms to travel buffer diameter
			time += globalEnvironment.get_time();
			
			ActionEvent data = { m_id };
			Event new_event = { ACTION_EVENT, time, { data } };
			pq.emplace(new_event);

			//stop_unit();
			collision_avoidance();

			//drop package
			if (globalEnvironment.get_package_status(m_id) == IN_TRANSIT) {
				globalEnvironment.update_package_status(m_id, DROPPED);
				globalEnvironment.drop_package(m_id);
			}
		
			//globalEnvironment.recalculate_collisions(*this); //change in direction from collision avoidance
		}
	}
	if (!colliding) {
		//attempt to generate a new collision event for the two units
		//age unit and recalculate
		globalEnvironment.recalculate_uc_collision(m_id, unit.get_id()); //substitute to age and recalculate. instead, clear curr collisions and recalculate?
	}
}