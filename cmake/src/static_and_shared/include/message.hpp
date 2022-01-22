#pragma once

#include <iostream>
#include <string>


class Message{

	public :
		Message(std::string msg);
		~Message();
		friend std::ostream& operator<<(std::ostream& os, const Message& msg);
	
	private :

		std::string val="Default msg";



};


