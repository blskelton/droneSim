#include "Geometry.h"
#include "Message.h"

#ifndef Event_h
#define Event_h

struct BoxEvent {
	int id;
	int age;
	Box box;
	bool containerCollision;
};

struct UCEvent {
	int idA;
	int ageA;
	int idB;
};

struct ActionEvent {
	int id;
};

struct MessageEvent {
	int idA;
	int idB;
	Message message;
};

struct ETAEvent {
	int id;
	float epsilon; //maximum acceptable distance between unit and reached destination
};

struct WaitEvent {
	int id;
};

struct SpeedChangeEvent {

};

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
