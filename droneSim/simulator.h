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

#ifndef simulator_h
#define simulator_h

#include <Windows.h>

//forward declarations
class Environment;
class Container;

extern Environment globalEnvironment;
extern Container globalContainer;

extern constexpr int global_num_units = 100;
extern constexpr int global_num_faulty_units = global_num_units / 5 - 1;

//coordinates for readability
extern constexpr int cX = 0;
extern constexpr int cY = 1;
extern constexpr int cZ = 2;

//event tags
extern constexpr int BOX_EVENT = 0;
extern constexpr int UC_EVENT = 1;
extern constexpr int ACTION_EVENT = 2;
extern constexpr int MESSAGE_EVENT = 3;
extern constexpr int ETA_EVENT = 4;
extern constexpr int WAIT_EVENT = 5;
extern constexpr int SPEED_CHANGE_EVENT = 6;
extern constexpr int PING_EVENT = 7;
extern constexpr int GLOBAL_MESSAGE_EVENT = 8;

//colors
extern constexpr float GREEN[3] = { (float)0.053, (float)0.51, (float)0.147 }; //base color
extern constexpr float YELLOW[3] = {1,1,0}; //in collision avoidance
extern constexpr float RED[3] = {1,0,0}; //collided core
extern constexpr float WHITE[3] = {1,1,1}; //reached destination
extern constexpr float BLUE[3] = { 0,0,1 }; //completed task

//statuses
extern constexpr int NOT_INITIALIZED = -1; //red
extern constexpr int AWAITING_TASK = 0; //blue
//extern constexpr int PERFORMING_TASK = 1; //green
extern constexpr int COLLISION_AVOIDANCE = 2; //yellow
extern constexpr int CORE_COLLIDED = 3; //red
extern constexpr int HEADED_TOWARDS_PICKUP = 1; //also green????
extern constexpr int CARRYING_PACKAGE = 4; 


extern constexpr int TARGET_FPS = 24;

extern constexpr bool packages = false;
extern constexpr int NUMBER_PACKAGES = 30;

#endif /* simulator_h */