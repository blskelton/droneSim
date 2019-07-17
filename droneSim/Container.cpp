#include <GL/glew.h>
#include <GL/freeglut.h>

#include "Container.h"
#include "simulator.h"
#include "Environment.h"



Container::Container() {
	//m_size = GLOBAL_SIZE;
	m_Xdimension = CONTAINER_X;
	m_Ydimension = CONTAINER_Y;
	m_Zdimension = CONTAINER_Z;
	m_leftX = -(m_Xdimension / 2);
	m_rightX = (m_Xdimension / 2);
	m_bottomY = -(m_Ydimension / 2);
	m_topY = (m_Ydimension / 2);
	m_closeZ = -10;
	m_farZ = m_closeZ - m_Zdimension;
}

void Container::draw_container() {
	glPushMatrix();

	glColor3f(1.0, 1.0, 0.0);
	glTranslatef(globalEnvironment.get_look()[cX], globalEnvironment.get_look()[cY], globalEnvironment.get_look()[cZ]);
	//glutWireCube(m_Xdimension); //WILL HAVE TO CHANGE
	glPopMatrix();

	//top
	//glRecti(m_leftX, m_closeZ, m_rightX, m_farZ);

	glBegin(GL_POLYGON);
	glVertex3f(m_leftX, m_topY, m_closeZ);
	glVertex3f(m_rightX, m_topY, m_closeZ);
	glEnd();

	//X axis
	glBegin(GL_LINES);
	glVertex3f(m_leftX, 0.0, -10 - m_Xdimension / 2);
	glVertex3f(m_rightX, 0.0, -10 - m_Xdimension / 2);
	glEnd();

	//Y axis
	glBegin(GL_LINES);
	glVertex3f(0.0, m_topY, -10 - m_Ydimension / 2);
	glVertex3f(0.0, m_bottomY, -10 - m_Ydimension / 2);
	glEnd();

	//X axis
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, m_closeZ);
	glVertex3f(0.0, 0.0, m_farZ);
	glEnd();
}
