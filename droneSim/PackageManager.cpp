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

#include "PackageManager.h"

PackageManager::PackageManager()
{

}

void PackageManager::update_in_transit_package_location(int id, Unit unit) {
	Package& package = m_packages[id];
	package.update_location(unit.get_location()[cX], unit.get_location()[cY] - 0.5f, unit.get_location()[cZ]);
}

void PackageManager::update_dropping_package_location(int id, float time) {
	Package& package = m_packages[id];
	float time_offset = time - m_package_fall_information[id][0];
	float new_y = m_package_fall_information[id][1] - (time_offset * 0.0075f);
	if (new_y <= globalContainer.get_bottomY()) {
		new_y = (float)globalContainer.get_bottomY();
		package.update_status(DROPPED);
	}
	package.update_location(package.position[cX], new_y, package.position[cZ]);
}



void PackageManager::initialize_packages() {
	for (int i = 0; i < NUMBER_PACKAGES; i++) {
		m_packages[i] = Package(globalContainer.get_leftX(), globalContainer.get_rightX());
	}
}
