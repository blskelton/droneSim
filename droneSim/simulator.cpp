#ifdef __APPLE__
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <OpenGL/glext.h>
#else
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <corecrt_math_defines.h>
#pragma comment(lib, "glew32.lib")
#endif

#include <vector>
#include <ctime>
#include <windows.h>

#include "Unit.h"
#include "Container.h"
#include "Environment.h"
#include "simulator.h"

//set-up
float win_width = 1000;
float win_height = 1000;

float theta = 1.56999886;
float radius = 30;

float xpos = radius * cos(theta);
float ypos = 0;
float zpos = radius * sin(theta) - 20;
float xdir = -cos(theta);
float ydir = 0;
float zdir = -20 - sin(theta);

//fps
LARGE_INTEGER frequency;
bool useQPC = QueryPerformanceFrequency(&frequency);
int g_current_frame_number;
long long start_time;
float actualFPS;

void init_scene();

//initialize environment and container objects
Container globalContainer;
Environment globalEnvironment;

void init(void) {
	srand(time(NULL));

	LARGE_INTEGER init_time;
	QueryPerformanceCounter(&init_time);
	//total runtime in milliseconds
	start_time = ((1000LL * init_time.QuadPart) / frequency.QuadPart);
	g_current_frame_number = 0;

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA);
	glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	init_scene();
}

void calculateFPS() {
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	auto now_ms = ((1000LL * now.QuadPart) / frequency.QuadPart);
	auto deltaTime = (now_ms-start_time)/1000;
	actualFPS = (float)g_current_frame_number / deltaTime;
}

void idle(void) {
	// draw current frame
	glutPostRedisplay();
	g_current_frame_number = g_current_frame_number + 1;
	calculateFPS();
	printf("%f\n", actualFPS);
}

void init_scene() {
	//initialize vector of units
	for (int i = 0; i < global_num_units; i++) {
		globalEnvironment.add_unit(i);
	}
	globalEnvironment.init_prev_end_timestamp();
}

void drawBox(void) {
	glPushMatrix();
	globalContainer.draw_container();
	glPopMatrix();
}

void display(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.f, win_width / win_height, .1f, 500.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_LINE, GL_FILL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(xpos, ypos, zpos, xdir, ydir, zdir, 0, 1, 0);
	drawBox();

	globalEnvironment.process();

	glutSwapBuffers();
}

void reshape(int w, int h){
	win_width = w;
	win_height = h;
	glViewport(0, 0, w, h);

	glutPostRedisplay();
}

void rotate_left() {
	theta += 0.01;

	xpos = radius * cos(theta);
	ypos = 0;
	zpos = radius * sin(theta) - 20;
	xdir = -cos(theta);
	ydir = 0;
	zdir = -20 - sin(theta);
}

void rotate_right() {
	theta -= 0.01;

	xpos = radius * cos(theta);
	ypos = 0;
	zpos = radius * sin(theta) - 20;
	xdir = -cos(theta);
	ydir = 0;
	zdir = -20 - sin(theta);
}

void decrease_radius() {
	radius -= 0.1;
	xpos = radius * cos(theta);
	ypos = 0;
	zpos = radius * sin(theta) - 20;
	xdir = -cos(theta);
	ydir = 0;
	zdir = -20 - sin(theta);
}

void increase_radius() {
	radius += 0.1;
	xpos = radius * cos(theta);
	ypos = 0;
	zpos = radius * sin(theta) - 20;
	xdir = -cos(theta);
	ydir = 0;
	zdir = -20 - sin(theta);
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27: // Escape key
		exit(0); 
		break;
	case 97: //a
		rotate_left();
		break;
	case 100: //d
		rotate_right();
		break;
	case 119: //w
		decrease_radius();
		break;
	case 115: //s
		increase_radius();
		break;
	}
}

int main(int argc, char * argv[])
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(win_width, win_height);

	glutCreateWindow("droneSim");

	init();

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();

	return 0;
}