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
#include <windows.h>

#include "Unit.h"
#include "Container.h"
#include "Environment.h"
#include "simulator.h"

//set-up
float win_width = 512;
float win_height = 512;

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

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
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
	//total runtime in milliseconds
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
		//set random destination
		//globalEnvironment.get_unit(i).init_dest();
	}
}

void drawBox(void) {
	globalContainer.draw_container();
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
	int* camera = globalEnvironment.get_camera();
	int* look = globalEnvironment.get_look();
	gluLookAt(camera[0], camera[1], camera[2], look[0], look[1], look[2], 0, 1, 0);

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

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27: // Escape key
		exit(0); 
		break;
	case 97: //a - decr. x
		globalEnvironment.change_view(97);
		break;
	case 100: //d - incr. x
		globalEnvironment.change_view(100);
		break;
	case 119: //w - incr. y
		globalEnvironment.change_view(119);
		break;
	case 115: //s - decr. y
		globalEnvironment.change_view(115);
		break;
	case 101: //e - decr. z
		globalEnvironment.change_view(101);
		break;
	case 113: //q - incr. z
		globalEnvironment.change_view(113);
		break;
	case 99: //c
		//globalEnvironment.change_view();
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

	globalEnvironment.init_prev_end_timestamp();

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();

	return 0;
}