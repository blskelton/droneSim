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