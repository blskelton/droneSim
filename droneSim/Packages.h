#pragma once

#include <cmath>
#include "simulator.h"

//package statuses
extern constexpr int UNINITIALIZED = -1;
extern constexpr int WAITING_FOR_PICKUP = 0;
extern constexpr int IN_TRANSIT = 1;
extern constexpr int AT_DESTINATION = 2;
extern constexpr int DROPPING = 3;
extern constexpr int DROPPED = 4;

//package for units to relocate
struct Package {
	float position[3]; //current position
	float destination[3]; //goal position
	int status;

	float* get_carrier_destination() {
		if (status == IN_TRANSIT) {
			return destination;
		}
		else {
			return position;
		}
	}

	void update_location(float new_x, float new_y, float new_z) {
		position[cX] = new_x;
		position[cY] = new_y;
		position[cZ] = new_z;
	}

	void update_y(float new_y) {
		position[cY] = new_y;
	}

	void update_status(int new_status) {
		status = new_status;
	}



	/*void update(int new_status, float unitX, float unitY, float unitZ) {
		if (status == IN_TRANSIT && new_status == IN_TRANSIT) {//update location
			update_location(unitX, unitY - 0.5f, unitZ);
			return;
		}
		/*if (status != new_status) {
			if (new_status == IN_TRANSIT || new_status == AT_DESTINATION) {
				float x = position[cX] - unitX;
				float y = position[cY] - unitY;
				float z = position[cZ] - unitZ;
				float distance = sqrt(abs(x*x + y * y + z * z));

				if (distance < 1) {
					status = new_status;
					if (status == IN_TRANSIT) {
						update_location(unitX, unitY - 0.5f, unitZ);
					}
					if (status == AT_DESTINATION) {
						arrive();
					}
				}
			}
			else {
				status = new_status;
			}
		}

	}*/

	void arrive() {
		position[cX] = destination[cX];
		position[cY] = destination[cY];
		position[cZ] = destination[cZ];
	}



	//constructor
	Package(float my_position[3], float my_destination[3], float start, float end) {
		position[cX] = start;// my_position[cX];
		position[cY] = my_position[cY];
		position[cZ] = my_position[cZ];
		destination[cX] = end;// my_destination[cX];
		destination[cY] = my_destination[cY];
		destination[cZ] = my_destination[cZ];
		status = UNINITIALIZED;
	}
	Package() {
	}
};

class Packages
{
	Package m_packages[NUMBER_PACKAGES];
	float m_drops[NUMBER_PACKAGES][2];
public:
	Packages(float, float);
	Packages();
	void drop_package(int, float);
	void fall(int, float);
	float* update(int, int, float, float, float);
	Package& get_package(int);
	int propose_status_change(int, int, float, float, float);
	inline void set_status(int package_id, int new_status) {
		m_packages[package_id].status = new_status;
	};
	inline void set_location(int package_id, float x, float y, float z) {
		m_packages[package_id].position[cX] = x;
		m_packages[package_id].position[cY] = y;
		m_packages[package_id].position[cZ] = z;
	};
	//populates array with random position within container
	inline void get_random_position(float(&numbers)[3]) {
		numbers[cX] = (float)(rand() % 20) - 10;
		numbers[cY] = (float)(rand() % 20) - 10;
		numbers[cZ] = (float)(rand() % 20) - 30;
	};

	

	~Packages();
};

