#include "RadioDevice.hpp"

#include <time.h>


int main(int argc, char* argv[]) {
	if (argc != 3) {
		printf("Usage is ./test [directory to device] [directory to second device]\n");
		exit(1);
	}
	RadioDevice radio1(argv[1], 1);
	RadioDevice radio2(argv[2], 0);
	int count = 0;
	int reversals = 0;
	int total = 5000;
	unsigned prev = 0;
	for(int i  = 0; i < total; i++) {
		usleep(15000); 
		unsigned lat = radio1.latest();
		if (prev != lat) {
			count++;
		}
		if (prev > lat) {
			reversals++;
			printf("Reversed\n");
		}
		prev = lat;
		printf("%u\n",lat);
	}
	printf("count:%d total:%d fraction:%f\n", count, total, count*1.0/total);
	printf("reversals:%d\n", reversals);
	printf("test done\n");
	return 0;

	
}