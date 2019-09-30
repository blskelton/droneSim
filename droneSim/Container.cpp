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

#include "Container.h"
#include "simulator.h"
#include "Environment.h"



Container::Container() {
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
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPushMatrix();

	glColor3f(1.0, 1.0, 0.0);
	glPopMatrix();

	//front
	glPushMatrix();
	glTranslatef(0.0, 0.0, (float)m_closeZ);
	glRecti(m_leftX, m_bottomY, m_rightX, m_topY);
	glPopMatrix();

	//back
	glPushMatrix();
	glTranslatef(0.0, 0.0, (float)m_farZ);
	glRecti(m_leftX, m_bottomY, m_rightX, m_topY);
	glPopMatrix();

	//edges
	glBegin(GL_LINES);
	glVertex3i(m_leftX, m_topY, m_closeZ);
	glVertex3i(m_leftX, m_topY, m_farZ);
	glVertex3i(m_leftX, m_bottomY, m_closeZ);
	glVertex3i(m_leftX, m_bottomY, m_farZ);
	glVertex3i(m_rightX, m_topY, m_closeZ);
	glVertex3i(m_rightX, m_topY, m_farZ);
	glVertex3i(m_rightX, m_bottomY, m_closeZ);
	glVertex3i(m_rightX, m_bottomY, m_farZ);
	glEnd();

	//X axis
	glBegin(GL_LINES);
	glVertex3i(m_leftX, 0, -10 - m_Xdimension / 2);
	glVertex3i(m_rightX, 0, -10 - m_Xdimension / 2);
	glEnd();

	//Y axis
	glBegin(GL_LINES);
	glVertex3i(0, m_topY, -10 - m_Ydimension / 2);
	glVertex3i(0, m_bottomY, -10 - m_Ydimension / 2);
	glEnd();

	//X axis
	glBegin(GL_LINES);
	glVertex3i(0, 0, m_closeZ);
	glVertex3i(0, 0, m_farZ);
	glEnd();
}
