#ifndef simulator_h
#define simulator_h

#include <Windows.h>

class Environment;
class Container;

extern Environment globalEnvironment;
extern Container globalContainer;

extern constexpr int global_num_units = 135;

extern constexpr int cX = 0;
extern constexpr int cY = 1;
extern constexpr int cZ = 2;

extern constexpr int BOX_EVENT = 0;
extern constexpr int UC_EVENT = 1;
extern constexpr int ACTION_EVENT = 2;
extern constexpr int MESSAGE_EVENT = 3;
extern constexpr int ETA_EVENT = 4;
extern constexpr int WAIT_EVENT = 5;
extern constexpr int SPEED_CHANGE_EVENT = 6;

extern constexpr float GREEN[3] = { (float)0.053, (float)0.51, (float)0.147 }; //base color
extern constexpr float YELLOW[3] = {1,1,0}; //in collision avoidance
extern constexpr float RED[3] = {1,0,0}; //collided core
extern constexpr float WHITE[3] = {1,1,1}; //reached destination

extern constexpr int TARGET_FPS = 24;

extern constexpr bool packages = false;

#endif /* simulator_h */