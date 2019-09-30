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

#include "Message.h"

Message::Message(int tag, int unitID, int round, void* content): m_tag{tag}, m_unitID{unitID}, m_round{round}, m_content{content}
{
}

Message::Message() {
	m_tag = 0;
	m_unitID = 0;
	m_round = 0;
}