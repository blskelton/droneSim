#include "Message.h"

Message::Message(int tag, int unitID, int round, void* content): m_tag{tag}, m_unitID{unitID}, m_round{round}, m_content{content}
{
}

Message::Message() {
	m_tag = 0;
	m_unitID = 0;
	m_round = 0;
}