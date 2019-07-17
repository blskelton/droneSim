#ifndef simulator_h
#define simulator_h

#include <Windows.h>

class Environment;
class Container;

extern Environment globalEnvironment;
extern Container globalContainer;

extern constexpr int global_num_units = 45;

extern constexpr int cX = 0;
extern constexpr int cY = 1;
extern constexpr int cZ = 2;

extern constexpr int BOX_EVENT = 0;
extern constexpr int UC_EVENT = 1;
extern constexpr int ACTION_EVENT = 2;
extern constexpr int MESSAGE_EVENT = 3;

extern constexpr int TARGET_FPS = 24;

#endif /* simulator_h */