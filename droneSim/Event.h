#include "Geometry.h"
#include "Message.h"

#ifndef Event_h
#define Event_h

struct boxEvent {
	int id;
	int age;
	Box box;
	bool containerCollision;
};

struct ucEvent {
	int idA;
	int ageA;
	int idB;
};

struct actionEvent {
	int id;
};

struct messageEvent {
	int idA;
	int idB;
	Message message;
};

struct Event {
	int tag; //0 if box, 1 if uc, 2 if action, 3 if message
	int idA;
	int age;
	int idB;
	//int ageB;
	float timestamp;
	Box box;
	bool containerCollision;
	Message message;
};

class myEventComparator
{
public:
	int operator() (Event& event1,Event& event2)
	{
		return event1.timestamp > event2.timestamp;
	}
};

#endif /* Event_h */
