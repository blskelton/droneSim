#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>

#include "Environment.h"
#include "Unit.h"
#include "simulator.h"
#include "Boxes.h"


Environment::Environment() : e2{ m_rd() }, distribution{ 100, 100 }, m_speed_change_frequency{ 2000 }, m_num_packages{10}
{
	m_grid = new Boxes();
	if (packages) {
		initialize_packages();
	}
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
	int random_location[3];
	int random_destination[3];
	for (int i = 0; i < m_num_packages; i++) {
		get_random_position(random_location);
		get_random_position(random_destination);
		m_packages[i] = Package(random_location, random_destination);
	}
}

void Environment::draw_packages() {
	for (int i = 0; i < m_num_packages; i++) {
		Package package = m_packages[i];
		if (package.status == 0) { //draw at init location
			glPushMatrix();
			glTranslatef(package.position[cX], package.position[cY], package.position[cZ]);
			glColor3f(1, 0, 0);
			glutWireCube(.25);
			glPopMatrix();
		}
		if (package.status == 1) { //draw underneath carrying unit
			glPushMatrix();
			glTranslatef(m_unitArray[i].get_location()[cX], m_unitArray[i].get_location()[cY]-0.5, m_unitArray[i].get_location()[cZ]);
			glColor3f(0, 0, 1);
			glutWireCube(0.25);
			glPopMatrix();
		}
		if (package.status == 2) { //draw at destination
			glPushMatrix();
			glTranslatef(package.destination[cX], package.destination[cY], package.destination[cZ]);
			glColor3f(0, 0, 1);
			glutWireCube(.25);
			glPopMatrix();
		}
	}
}

void Environment::draw_unit(int id) {
	//get unit color, radius, location
	Unit& currUnit = m_unitArray[id];
	float unit_color_red = (currUnit.get_color())[0];
	float unit_color_green = (currUnit.get_color())[1];
	float unit_color_blue = (currUnit.get_color())[2];
	glColor3f(unit_color_red, unit_color_green, unit_color_blue);
	float radius = currUnit.get_radius();
	float bufferRadius = currUnit.get_buffer_radius();
	float* location = currUnit.get_location();

	//draw unit
	glPushMatrix();
	glTranslatef(*(location), *(location + 1), *(location + 2));
	//inside
	glutSolidSphere(radius, 10, 10);
	//outside
	glColor3f((float) 0.35, (float) 0.35, (float) 0.35);
	glutWireSphere(bufferRadius, 8, 5);
	glPopMatrix();
}

void Environment::send_message(int id1, int id2) {
	int number = 5;
	void* content = &number;
	Message newMessage = Message(0, id1, 0, content);
	m_unitArray[id1].send(newMessage, id2);
}