#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>

#include "Environment.h"
#include "Unit.h"
#include "simulator.h"
#include "Boxes.h"


Environment::Environment() : e2{ m_rd() }, distribution{ 100, 100 }, m_view{1}
{
	QueryPerformanceCounter(&m_start_time);
	QueryPerformanceFrequency(&m_freq);
	//printf("%d", global_num_units);
	//initialize camera and view
	m_camera[0] = 0;
	m_camera[1] = 0;
	m_camera[2] = globalContainer.get_closeZ() + globalContainer.get_z_dimension();
	m_look[0] = 0;
	m_look[1] = 0;
	m_look[2] = -10 - globalContainer.get_z_dimension() / 2;

	m_grid = Boxes();

	QueryPerformanceCounter(&m_start_time);
}

std::vector<int> Environment::get_box_neighbors(int unitID) {
	Unit unit = m_unitArray[unitID];
	return m_grid.get_units_in_box(unit);
}

void Environment::add_unit(int unitID) {
	m_unitArray[unitID] = Unit(unitID);
	Unit& newUnit = m_unitArray[unitID];

	if (unitID == 0) {
		//m_unitArray[0].set_location((float)-5.1, (float)0, (float)-15);
		//m_unitArray[0].set_dest(0,0,-12.5);
		//m_unitArray[0].generate_destination_event(m_event_queue);
		//m_unitArray[0].change_direction(1, 0, 0);
	}
	if (unitID == 1) {
		m_unitArray[1].set_location(5.5, 0, -15);
		m_unitArray[1].change_direction(-1, 0, 0);
	}

	m_grid.get_box(m_unitArray[unitID]);
	
	//get and add first box transition event
	Event firstBoxEvent = m_grid.get_next_box_event(m_unitArray[unitID]);
	m_event_queue.emplace(firstBoxEvent);

	//get and add any unit collisions
	m_grid.add_collisions(m_unitArray[unitID], m_event_queue);

	//add destination event
	m_unitArray[unitID].generate_destination_event(m_event_queue);
};

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

void Environment::change_view(int direction) {
	if (direction == 97) {
		m_camera[cX] -= 1;
	}
	if (direction == 100) {
		m_camera[cX] += 1;
	}
	if (direction == 119) {
		m_camera[cY] += 1;
	}
	if (direction == 115) {
		m_camera[cY] -= 1;
	}
	if (direction == 101) {
		m_camera[cZ] -= 1;
	}
	if (direction == 113) {
		m_camera[cZ] += 1;
	}
}

void Environment::send_message(int id1, int id2) {
	int number = 5;
	void* content = &number;
	Message newMessage = Message(0, id1, 0, content);
	m_unitArray[id1].send(newMessage, id2);
}