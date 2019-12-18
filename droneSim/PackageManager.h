/*<DroneSim - a simulator graphically modeling drone activity in real time.>
	Copyright(C) < 2019 > <Blake Skelton>

	This program is free software : you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see < https://www.gnu.org/licenses/>. */

#include "Geometry.h"
#include "Unit.h"

#ifndef PackageManager_h
#define PackageManager_h

class PackageManager {
private:

	Package m_packages[NUMBER_PACKAGES];
	float m_package_fall_information[NUMBER_PACKAGES][2]; //2d array with fall time and height

public:
	PackageManager();

	void initialize_packages();

	void update_in_transit_package_location(int, Unit);

	void update_dropping_package_location(int, float);

	inline Package& get_package(int id) {
		Package& package = m_packages[id];
		return package;
	};

	inline void update_package_carrier(int package_id, int carrier_id) {
		m_packages[package_id].update_carrier(carrier_id);
	};

	inline int get_package_carrier(int package_id) {
		return m_packages[package_id].carrier;
	};

	//returns package status given its id
	inline int get_package_status(int id) {
		Package& package = m_packages[id];
		return package.status;
	}

	//updates package status to given status
	inline bool update_package_status(int id, int status, float x, float y, float z) {
		return m_packages[id].update_status(status, x, y, z);
	}

	inline int get_available_assignment_id() {
		for (int i = 0; i < NUMBER_PACKAGES; i++) {
			if (m_packages[i].status == DROPPED) {
				return i;
			}
		}
		return -1;
	};

	//writes current time into package_fall_times array for falling animation
	inline void drop_package(int id, Unit unit, float time) {
		Unit carrier = unit;
		m_packages[id].update_location(carrier.get_location()[cX], carrier.get_location()[cY] - 0.5f, carrier.get_location()[cZ]);
		m_packages[id].update_carrier(-1);

		m_package_fall_information[id][0] = time;
		m_package_fall_information[id][1] = m_packages[id].position[cY];
	}
}

#endif /*Package_Manager_h */