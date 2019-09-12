#include "Geometry.h"
#include "Message.h"

#ifndef Event_h
#define Event_h

//event for moving from one sub-box to another
struct BoxEvent {
	int id;
	int age;
	Box box;
	bool containerCollision;
};

//event for unit collision
struct UCEvent {
	int idA;
	int ageA;
	int idB;
};

//event for taking an action (esp. with collision avoidance and upon reaching destination)
struct ActionEvent {
	int id;
};

//event for sending a message
struct MessageEvent {
	int idA;
	int idB;
	Message message;
};

//event for eta at destination
struct ETAEvent {
	int id;
	float epsilon; //maximum acceptable distance between unit and reached destination
};

//event to end a wait period
struct WaitEvent {
	int id;
	int tag;
};

//empty struct to signal a system-wide speed update
struct SpeedChangeEvent {

};

//struct containing a tag indicating the event contents, a timestamp, and one of the above structs
struct Event {
	int tag; 
	float timestamp;
	union contents {
		BoxEvent boxEvent;
		UCEvent ucEvent;
		ActionEvent actionEvent;
		MessageEvent messageEvent;
		ETAEvent etaEvent;
		WaitEvent waitEvent;
		SpeedChangeEvent speedChangeEvent;

		contents(BoxEvent boxEvent) {
			this->boxEvent = boxEvent;
		}
		contents(UCEvent ucEvent) {
			this->ucEvent = ucEvent;
		}
		contents(ActionEvent actionEvent) {
			this->actionEvent = actionEvent;
		}
		contents(MessageEvent messageEvent) {
			this->messageEvent = messageEvent;
		}
		contents(ETAEvent etaEvent) {
			this->etaEvent = etaEvent;
		}
		contents(WaitEvent waitEvent) {
			this->waitEvent = waitEvent;
		}
		contents(SpeedChangeEvent speedChangeEvent) {
			this->speedChangeEvent = speedChangeEvent;
		}
	} data;
	void update_timestamp(float start_time) {
		timestamp = timestamp + start_time;
	}
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
