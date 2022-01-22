#include "message.hpp"


int main(){

	Message say_hi("Anil says Hi");
	Message say_bye("Anil says Goodbye");

	std::cout << say_hi << say_bye;

	return 0;
}
