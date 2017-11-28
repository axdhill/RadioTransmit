
#include "RadioDevice.hpp"






#include <unistd.h>

RadioDevice::RadioDevice(char* address) {
	m_address = address;
	m_fd = open (m_address, O_RDWR | O_NOCTTY | O_SYNC);
	int ready = 0;

	pthread_mutex_init(&mutex, NULL);
	pthread_cond_init(&cond, NULL);

	sendthread = std::thread(&RadioDevice::send,this);
	recvthread = std::thread(&RadioDevice::recv,this);

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
	for (unsigned i = 999; i < 100000; i++) {
		// try to send data
		unsigned total_written = 0;
		char buf[64];
		memset(&buf, 0, sizeof(buf) );
		sprintf(buf,"%0.8f %0.8f %0.8f",0.235,0.11,0.55);

		while(sizeof(unsigned) != total_written) {
			usleep(100000); // some rate limiting necessary
			unsigned current = write(m_fd, buf, sizeof(buf));
			if (current < 0) {
				printf("Fatal error\n");
				exit(1);
			} else {
				total_written+= current;
				//printf("other%u\n", total_written);
			}
		}
		write (m_fd, &i, sizeof(unsigned));           // send 8 character greeting
		printf("Sent: %u\n", i);
	}
	printf("Send done\n");
	return NULL;
}

void* RadioDevice::recv() {
	set_interface_attribs (m_fd, B57600, 0);  // set speed to 57600 bps, 8n1 (no parity)
	set_blocking (m_fd, 0);   
	unsigned local_data = 0;
	char buf[64];
	memset(&buf,0, sizeof(buf) );
	while(local_data < 99900) {
		// try to receive data
		unsigned total_read = 0;
		while(sizeof(unsigned) != total_read) {
			unsigned current = read(m_fd, buf, sizeof(buf));
			if (current < 0) {
				printf("Fatal error\n");
				exit(1);
			} else {
				total_read += current;
				//printf("thing%u\n", total_read);
			}
		}
		printf("Received: %s\n", buf);
		// data received
		pthread_mutex_lock(&mutex);
		// update to most recent data
		data = local_data;
		ready = 1;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&mutex);
	}
	printf("Recv done\n");
	//exit(1);
	return NULL;
}



unsigned RadioDevice::latest() {
	unsigned localdata;
	pthread_mutex_lock(&mutex);
		// update to most recent data
	localdata = data;
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);
	return localdata;
}