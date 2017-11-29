#include "RadioDevice.hpp"

#include <time.h>


int main(int argc, char* argv[]) {
	RadioDevice radio(argv[1]);

	for(int i  = 0; i < 1000; i++) {
		usleep(100000); 
		
		unsigned lat = radio.latest();
		printf("%u\n",lat);
	}
	return 0;

	
}