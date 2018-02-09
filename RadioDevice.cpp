
#include "RadioDevice.hpp"

#include <unistd.h>

//std::atomic<unsigned> RadioDevice::data;

RadioDevice::RadioDevice(char* address) {
	RadioDevice(address, 2);
}

RadioDevice::RadioDevice(char* address, int type) {
	m_address = address;
	m_fd = open (m_address, O_RDWR | O_NOCTTY | O_SYNC);
	int ready = 0;

	// pthread_mutex_init(&mutex, NULL);
	// pthread_cond_init(&cond, NULL);
	got_data = false;
	if (type == 0 || type == 2) {
		printf("LOGGING: Radio sending.\n");
		sendthread = std::thread(&RadioDevice::send,this);
	} else if (type == 1 || type == 2) {
		printf("LOGGING: Radio receiving.\n");
		recvthread = std::thread(&RadioDevice::recv,this);
	}
	// pthread_create(&threads[0], NULL, send, NULL);

	// pthread_create(&threads[1], NULL, recv, NULL);
	// user

}

RadioDevice::~RadioDevice() {
	sendthread.join();
	recvthread.join();
	printf("Done\n");
}


int RadioDevice::set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        //tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
                return -1;
        }
        return 0;
}

void RadioDevice::set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                return;
}

void* RadioDevice::send() {
	set_blocking (m_fd, 0);                // set no blocking

	for (unsigned i = 0; i < 100000; i+= 1) {
		// try to send data
		//printf("Sending: %d\n", i);
		unsigned total_written = 0;
		while(sizeof(unsigned) != total_written) { // unsure why, but the equivalent blocking version of this code doesn't work as well (gets some trash values)
			usleep(15000); // some rate limiting necessary
			ssize_t current = write(m_fd, ((char*)&i)+total_written, sizeof(unsigned)-total_written);
			//printf("Sent: %08x\n", i);
			if (current < 0) {
				printf("Fatal error\n");
				exit(1);
			} else {
				total_written+= current;
				//printf("other%u\n", total_written);
			}
		}
		//write (m_fd, &i, sizeof(unsigned));           // send 8 character greeting
		
	}
	printf("Send done\n");
	return NULL;
}

void* RadioDevice::recv() {
	set_interface_attribs (m_fd, B230400, 0);  // set speed to 234000 bps, 8n1 (no parity)
	set_blocking (m_fd, 0);   
	unsigned local_data = 0;
	while(local_data < 999000) {
		//printf("wat2: %d\n",  data);
		// try to receive data
		unsigned total_read = 0;
		while(sizeof(unsigned) != total_read) {
			ssize_t current = read(m_fd, ((char*)&local_data)+total_read, sizeof(unsigned)-total_read);
			if (current < 0) {
				printf("Fatal error\n");
				exit(1);
			} else {
				total_read += current;
				//printf("thing%u\n", total_read);
			}
		}
		// printf("Received: %08x\n", local_data);
		// data received
		//printf("Recv: %d\n", local_data);
		//printf("Recv Pointer: %p\n", &this->data);
		// update to most recent data
		data.store(local_data, std::memory_order_relaxed);
		mutex.unlock();
	}
	//printf("Received: %d\n", local_data);
	//printf("Recv done\n");
	exit(1);
	return NULL;
}



unsigned RadioDevice::latest() {
	//printf("latest");
	//printf("Lat: %d\n", this->data.load());
	//printf("Lat  Pointer: %p\n", &this->data);
	unsigned ret;
		// update to most recent data
	// sprintf(localdata,"%08x",data);
	ret = this->data.load(std::memory_order_relaxed);
	//printf("returning");
	return ret;
}