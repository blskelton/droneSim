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

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>

#include "Environment.h"
#include "Unit.h"
#include "simulator.h"
#include "Boxes.h"


Environment::Environment() : e2{ m_rd() }, distribution{ 100, 100 }, m_speed_change_frequency{ 1000 }, m_longest_process{0}, m_round{0}
{
	m_grid = new Boxes();
	m_package_manager = new Packages(globalContainer.get_leftX(), globalContainer.get_rightX());
	m_global_messages = new globalMessaging();
	for (int i = 0; i < NUMBER_PACKAGES; i++) {
		m_package_carriers[i] = -1;
	}

	//if (packages) {
		//initialize_packages();
	//}
}

void Environment::add_unit(int unitID) {
	m_unitArray[unitID] = Unit(unitID);
	Unit& newUnit = m_unitArray[unitID];

	m_grid->get_box(m_unitArray[unitID]);

	bool has_neighbors = check_initial_collisions(unitID);
	while (has_neighbors) {
		m_unitArray[unitID].randomize_location();
		has_neighbors = check_initial_collisions(unitID);
	}
	
	//get and add first box transition event
	m_grid->get_next_box_event(m_unitArray[unitID], m_event_queue);

	//get and add any unit collisions
	m_grid->add_collisions(m_unitArray[unitID], m_event_queue);

	//add destination event
	m_unitArray[unitID].generate_eta_event(m_event_queue);
};

void Environment::check_core_collisions(Unit& unit) {
	for (int i = 0; i < global_num_units; i++) {
		if (i != unit.get_id()) {
			Unit& unitB = m_unitArray[i];
			if (unit.is_colliding(unitB, true)) {
				unit.perform_core_collision();
				unitB.perform_core_collision();
			}
		}
	}
}

bool Environment::check_initial_collisions(int id) {
	for (int i = 0; i < id; i++) {
		Unit& unitB = m_unitArray[i];
		if (m_unitArray[id].is_colliding(unitB, true)) {
			return true;
		}
	}
	return false;
}

void Environment::initialize_packages() {
	/*float random_location[3];
	float random_destination[3];
	for (int i = 0; i < NUMBER_PACKAGES; i++) {
		get_random_position(random_location);
		get_random_position(random_destination);
		m_packages[i] = Package(random_location, random_destination, globalContainer.get_leftX(), globalContainer.get_rightX());
	}*/
}

void Environment::draw_packages() {
	for (int i = 0; i < NUMBER_PACKAGES; i++) {
		Package package = m_package_manager->get_package(i);
		if (package.status == DROPPING) {
			m_package_manager->fall(i, m_current_time);
			package = m_package_manager->get_package(i);
		}
		if (package.status == DROPPED) {

		}
		/*if (package.status == WAITING_FOR_PICKUP) { //draw at init location, don't update location
			//glPushMatrix();
			//glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);
			//glColor3f(1, 0, 0);
			//glutWireCube(.25);
			//glPopMatrix();
		}
		if (package.status == IN_TRANSIT) { //draw underneath carrying unit - update location, then draw
			int carrier_id = package.carrier;
		//	float* location = m_unitArray[carrier_id].get_location();
			package.update_location(m_unitArray[carrier_id].get_location()[cX], m_unitArray[carrier_id].get_location()[cY]-0.5f, m_unitArray[carrier_id].get_location()[cZ]);
			//glPushMatrix();
			//glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);
			//glColor3f(0, 0, 1);
			//glutWireCube(0.25);
			//glPopMatrix();
		}
		if (package.status == AT_DESTINATION) { //draw at destination
			//glPushMatrix();
			//glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);
			//glColor3f(0, 0, 1);
			//glutWireCube(.25);
			//glPopMatrix();
		}
		if (package.status == DROPPED) {
			float time_offset = m_current_time - m_package_fall_information[i][0];
			float new_y = m_package_fall_information[i][1] - (time_offset * 0.0075f);
			if (new_y <= globalContainer.get_bottomY()) {
				new_y = (float)globalContainer.get_bottomY();
			}
			package.position;
			//package.update_location(package.position[cX], new_y, package.position[cZ]);
			package.update_location(-1, new_y, -1);
			glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);

			//glColor3f(1, 0, 0);
			//glutWireCube(.25);
			//glPopMatrix();

		}*/
		glPushMatrix();
		glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);
		glColor3f(1, 1, 0);
		glutWireCube(0.25);
		glPopMatrix();
	}
}

void Environment::draw_units() {
	for (int id = 0; id < global_num_units; id++) {
		Unit& currUnit = m_unitArray[id];
		int status = currUnit.get_status();
		/*if (status == NOT_INITIALIZED) {
			glColor3f(RED[cX], RED[cY], RED[cZ]);
		}
		if (status == AWAITING_TASK) {
			glColor3f(BLUE[cX], BLUE[cY], BLUE[cZ]);
		}
		if (status == HEADED_TOWARDS_PICKUP || status == CARRYING_PACKAGE) {
			glColor3f(GREEN[cX], GREEN[cY], GREEN[cZ]);
		}
		if (status == COLLISION_AVOIDANCE) {
			glColor3f(YELLOW[cX], YELLOW[cY], YELLOW[cZ]);
		}
		if (status == CORE_COLLIDED) {
			glColor3f(RED[cX], RED[cY], RED[cZ]);
		}*/
		glColor3f(GREEN[cX], GREEN[cY], GREEN[cZ]);
		float radius = currUnit.get_radius();
		float bufferRadius = currUnit.get_buffer_radius();
		float* location = currUnit.get_location();

		//draw unit
		glPushMatrix();
		glTranslatef(*(location), *(location + 1), *(location + 2));
		//inside
		glutSolidSphere(radius, 10, 10);
		//outside
		glColor4f((float) .5, (float) 0.5, (float) 0.5, .5);
		glutSolidSphere(bufferRadius, 20, 20);
		glPopMatrix();
	}
}

void Environment::send_message(int id1, int id2) {
	int number = 5;
	void* content = &number;
	Message newMessage = Message(0, id1, 0, content);
	m_unitArray[id1].send(newMessage, id2);
}