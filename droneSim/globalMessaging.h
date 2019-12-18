#pragma once
#include <vector>
#include <array>

#include "Message.h"
#include "simulator.h"

class globalMessaging
{
private:
	std::vector <std::array<Message, global_num_units>> m_messages;
	std::vector <std::vector<Message>> m_first_n_messages;
	std::vector<int> m_messages_per_round;
	int m_highest_valid_round;

public:

	globalMessaging();
	~globalMessaging();

	void write_message(Message);
	//bool recv(int, Message(messages)[global_num_units]);
	std::array<Message, global_num_units>& recv(int, bool&);
	void place_test_message();
	void add_array();
};

