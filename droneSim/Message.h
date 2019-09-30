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

#ifndef Message_h
#define Message_h

class Message {
private:
	int m_tag;
	int m_unitID;
	int m_round;
	void* m_content;

public:
	//message constructor
	Message(int, int, int, void*);

	//default constructor (empty)
	Message();

	//returns msg tag
	inline int get_tag() {
		return m_tag;
	};

	//returns msg sender
	inline int get_sender() {
		return m_unitID;
	};

	//returns msg round
	inline int get_round() {
		return m_round;
	};
};

class message_comparator
{
public:
	int operator() (Message& message1, Message& message2)
	{
		return message1.get_round() > message2.get_round();
	}
};

#endif /*Message_h */