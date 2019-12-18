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

#include "Geometry.h"
#include "Message.h"

#ifndef Event_h
#define Event_h

extern constexpr int END_COLLISION_AVOIDANCE = 0;

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
/*struct ActionEvent {
	int id;
};*/

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

struct PingEvent {
	int id;
	int tag; 
};

//empty struct to signal a system-wide speed update
struct SpeedChangeEvent {

};

struct GlobalMessageEvent {
	Message message;
};

//struct containing a tag indicating the event contents, a timestamp, and one of the above structs
struct Event {
	int tag; 
	float timestamp;
	union contents {
		BoxEvent boxEvent;
		UCEvent ucEvent;
		PingEvent pingEvent;
		MessageEvent messageEvent;
		ETAEvent etaEvent;
		WaitEvent waitEvent;
		SpeedChangeEvent speedChangeEvent;
		GlobalMessageEvent globalMessageEvent;

		contents(BoxEvent boxEvent) {
			this->boxEvent = boxEvent;
		}
		contents(UCEvent ucEvent) {
			this->ucEvent = ucEvent;
		}
		contents(PingEvent pingEvent) {
			this->pingEvent = pingEvent;
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
		contents(GlobalMessageEvent globalMessageEvent) {
			this->globalMessageEvent = globalMessageEvent;
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
