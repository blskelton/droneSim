#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>

#include "Environment.h"
#include "Unit.h"
#include "simulator.h"
#include "Boxes.h"


Environment::Environment() : e2{ m_rd() }, distribution{ 100, 100 }, m_view{ 1 }, m_speed_change_frequency{ 2000 }, m_distance{ 30 }, m_camera_angle{ 0 }
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
	m_look[2] = globalContainer.get_closeZ() - globalContainer.get_z_dimension() / 2;
	m_distance = abs(m_camera[2]);

	m_grid = new Boxes();
	for (int i = 0; i < global_num_units; i++) {
		m_units_colliding[i] = false;
	}
	m_speed_change_counter = 0;
	QueryPerformanceCounter(&m_start_time);

}

void Environment::add_unit(int unitID) {
	m_unitArray[unitID] = Unit(unitID);
	Unit& newUnit = m_unitArray[unitID];

	if (unitID == 0) {
		m_unitArray[0].set_location((float)-1, (float)0, (float)-12);
		m_unitArray[0].set_dest(5,0,-12);
		//m_unitArray[0].generate_destination_event(m_event_queue);
		//m_unitArray[0].change_direction(1, 0, 0);
	}
	if (unitID == 1) {
		m_unitArray[1].set_location(1, 0, -12);
		m_unitArray[1].set_dest(-5, 0, -12);
		//m_unitArray[1].change_direction(-1, 0, 0);
	}

	m_grid->get_box(m_unitArray[unitID]);
	bool has_neighbors = m_grid->has_neighbors(m_unitArray[unitID]);
	while (has_neighbors) {
		m_unitArray[unitID].randomize_location();
		has_neighbors = m_grid->has_neighbors(m_unitArray[unitID]);
	}
	
	//get and add first box transition event
	Event firstBoxEvent = m_grid->get_next_box_event(m_unitArray[unitID]);
	m_event_queue.emplace(firstBoxEvent);

	//get and add any unit collisions
	m_grid->add_collisions(m_unitArray[unitID], m_event_queue);

	//add destination event
	m_unitArray[unitID].generate_eta_event(m_event_queue);
};

void Environment::check_core_collisions(Unit& unit) {
	//vector<int> neighbors;
	for (int i = 0; i < global_num_units; i++) {
		if (i != unit.get_id()) {
			Unit& unitB = m_unitArray[i];
			if (unit.is_colliding(unitB, true)) {
				vector<Event> ABEvents = m_grid->get_events(unit.get_id(), unitB.get_id());
				vector<Event> BAEvents = m_grid->get_events(unitB.get_id(), unit.get_id());

				unit.unprocess(1000);
				unitB.unprocess(1000);
				float time = get_time();
				bool bufferCollision = unit.is_colliding(unitB, false);
				bool coreCollision = unit.is_colliding(unitB, true);
				unit.unit_collision(i, true);

				unit.perform_core_collision();
				unitB.perform_core_collision();
			}
		}
	}
	/*m_neighbors.clear();
	m_grid->get_neighbors(unit, m_neighbors);
	for (int id : m_neighbors) {
		Unit& neighbor_unit = m_unitArray[id];
		if (unit.is_colliding(neighbor_unit, true)) {
			float time = get_time();
			vector<Event> ABEvents = m_grid->get_events(unit.get_id(), neighbor_unit.get_id());
			vector<Event> BAEvents = m_grid->get_events(neighbor_unit.get_id(), unit.get_id());
			unit.perform_core_collision();
			neighbor_unit.perform_core_collision();
		}
	}*/
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

void Environment::change_view(int direction) {
	/*m_distance;
	float dif = m_camera[cX] - 
	float camAngleX = tanf(m_distance / (m_camera[cX]));
	float camX = m_distance * -sinf(camAngleX*(M_PI / 180)) * cosf((camAngleY)*(M_PI / 180));
	float camY = m_distance * -sinf((camAngleY)*(M_PI / 180));
	float camZ = -m_distance * cosf((camAngleX)*(M_PI / 180)) * cosf((camAngleY)*(M_PI / 180));*/
	float alpha = atan(m_camera[cY] / m_distance);
	float x = m_distance * sin(alpha);
	float z = m_distance * cos(alpha);// +globalContainer.get_closeZ() + globalContainer.get_z_dimension();


	if (direction == 97) {
		/*m_camera_angle++;
		alpha = atan(m_camera_angle / m_distance);
		//m_camera[cX] -= 1;
		m_camera[cX] = m_distance * sin(alpha);
		m_camera[cZ] = m_distance * cos(alpha) + globalContainer.get_closeZ() + globalContainer.get_z_dimension();*/
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