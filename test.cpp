#include "RadioDevice.hpp"

#include <time.h>


int main() {
	RadioDevice radio("/dev/ttyUSB0");
	for(int i  = 0; i < 1000; i++) {
		usleep(100000); 
		printf(radio.latest());
	}
	return 0

	
}