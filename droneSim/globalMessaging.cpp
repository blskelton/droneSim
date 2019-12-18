#include "globalMessaging.h"

const int minimum_messages = global_num_units - global_num_faulty_units;

globalMessaging::globalMessaging(): m_highest_valid_round(-1)
{
	m_messages.emplace_back();
}


globalMessaging::~globalMessaging()
{
}

void globalMessaging::write_message(Message message) {
	int round = message.get_round();
	int sender_id = message.get_sender();
	if (m_messages[round][sender_id].get_tag() == -1) {
		m_messages[round][sender_id] = message;
		m_messages_per_round[round]++;
		if (m_messages_per_round[round] <= minimum_messages) {
			m_first_n_messages[round].emplace_back(message);
		}
		if (round == m_highest_valid_round + 1) {
			if (m_messages_per_round[round] >= minimum_messages) {
				m_highest_valid_round = round;
				while (m_messages_per_round[m_highest_valid_round + 1] >= minimum_messages) {
					m_highest_valid_round++;
				}
			}
		}
	}
}

std::array<Message, global_num_units>& globalMessaging::recv(int round, bool &populated) {
	if (round < m_highest_valid_round || m_highest_valid_round == -1) {
		populated = false;
		std::array<Message, global_num_units> empty_array;
		return empty_array;
	}
	else {
		populated = true;
		return m_messages[round];
	}
}

void globalMessaging::place_test_message() {
	Message test_message = Message(8, 0, 0, 0);
	for (int i = 0; i < global_num_units; i++) {
		m_messages[0][i] = test_message;
	}
}

/*bool globalMessaging::recv(int round, Message(messages)[global_num_units]) {
	if (round < m_highest_valid_round || m_highest_valid_round == -1) {
		return false;
	}
	else {
		for (Message m : m_first_n_messages[round]) {
			int id = m.get_sender();
			messages[id] = m;
		}
		return true;
	}
}*/

void globalMessaging::add_array() {
	m_messages.emplace_back();
	m_first_n_messages.emplace_back();
	m_messages_per_round.emplace_back(0);
}