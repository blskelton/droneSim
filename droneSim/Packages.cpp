#include "Packages.h"



Packages::Packages(float left_bound, float right_bound)
{
	float random_location[3];
	float random_destination[3];
	for (int i = 0; i < NUMBER_PACKAGES; i++) {
		get_random_position(random_location);
		get_random_position(random_destination);
		m_packages[i] = Package(random_location, random_destination, left_bound, right_bound);
	}
}

Packages::Packages() {

}

Package& Packages::get_package(int id) {
	return m_packages[id];
}

void Packages::drop_package(int package_id, float time) {
	m_drops[package_id][0] = time;
	m_drops[package_id][1] = get_package(package_id).position[cY];
}

void Packages::fall(int package_id, float time) {
	float old_y = m_drops[package_id][1];
	float time_delta = time - m_drops[package_id][0];
	float new_y = old_y - 0.005*(time_delta);
	if (new_y < -10) {
		get_package(package_id).update_y(-10.0);
		get_package(package_id).status = DROPPED;
	}
	else {
		get_package(package_id).update_y(new_y);
	}
	
}

int Packages::propose_status_change(int package_id, int current_unit_status, float x, float y, float z) {
	Package package = m_packages[package_id];
	if (current_unit_status == HEADED_TOWARDS_PICKUP) {
		float dx = package.position[cX] - x;
		float dy = package.position[cY] - y;
		float dz = package.position[cZ] - z;
		float distance = sqrt(abs(dx*dx + dy * dy + dz * dz));
		if (distance < 1.0) {
			m_packages[package_id].status = IN_TRANSIT;
			return CARRYING_PACKAGE;
		}
	}
	if (current_unit_status == CARRYING_PACKAGE) {
		float dx = m_packages[package_id].position[cX] - x;
		float dy = m_packages[package_id].position[cY] - y;
		float dz = m_packages[package_id].position[cZ] - z;
		float distance = sqrt(abs(dx*dx + dy * dy + dz * dz));
		if (distance < 1.0) {
			m_packages[package_id].status = AT_DESTINATION;
			return AWAITING_TASK;
		}
	}
	return current_unit_status;
}


Packages::~Packages()
{
}
