
#ifndef RADIODEVICE_HPP
#define RADIODEVICE_HPP



#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <utility>


class RadioDevice {
private:
	int set_interface_attribs (int fd, int speed, int parity);
	void set_blocking (int fd, int should_block);
	void* send();
	void* recv();

	std::atomic<unsigned> data;
	char* m_address;
	int m_fd;

	pthread_t threads[2];

	std::thread sendthread;
	std::thread recvthread; 

public:
	RadioDevice(char* address);
	RadioDevice(char* address, int type);
	~RadioDevice();

	unsigned latest();

};

#endif