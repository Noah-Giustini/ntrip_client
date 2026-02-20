/*
https://github.com/Noah-Giustini/ntrip_client/

MIT License

Copyright (c) 2026 Noah Giustini

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>  
#include <fcntl.h>   
#include <string.h> 
#include <stdlib.h>  
#include <sys/times.h> 
#include <sys/types.h>  
#include <termios.h>		//termios, tcgetattr(), tcsetattr()   
#include <unistd.h>   
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/ioctl.h> 

#include <memory>
#include <iostream>
#include <utility>
#include <fstream>

#include "serial.h" 

using namespace serial;

Serial::Serial(const portinfo_t& portinfo) : portinfo_(portinfo) {
	std::cout << "[Serial] Serial object successfully created (portinfo)" << std::endl;
}

Serial::Serial(const std::string& devpath, int baudrate)
	: portinfo_({0, baudrate, '8', 0, 0, '0', '0', '1', 0, devpath}), fdcom_(-1), port_(devpath) {
	std::cout << "[Serial] Serial object successfully created (devpath) for " << devpath << std::endl;
}

void Serial::get_port() {
	if (!portinfo_.devpath.empty()) {
		port_ = portinfo_.devpath;
		return;
	}
}

int Serial::convbaud(unsigned long int baudrate) {
	switch (baudrate) {
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	default:
		return B9600;
	}
}

int Serial::PortSet() {
	struct termios termios_old, termios_new;
	int     baudrate, tmp;
	char    databit, stopbit, parity, fctl;

	bzero(&termios_old, sizeof(termios_old));
	bzero(&termios_new, sizeof(termios_new));
	cfmakeraw(&termios_new);
	tcgetattr(fdcom_, &termios_old);         //get the serial port attributions   

	//baudrates   
	baudrate = convbaud(portinfo_.baudrate);
	cfsetispeed(&termios_new, baudrate);
	cfsetospeed(&termios_new, baudrate);
	termios_new.c_cflag |= CLOCAL;
	termios_new.c_cflag |= CREAD;

	//flow control   
	fctl = portinfo_.fctl;
	switch (fctl) {
	case '0':
		termios_new.c_cflag &= ~CRTSCTS;        //no flow control   
		break;
	case '1':
		termios_new.c_cflag |= CRTSCTS;         //hardware flow control   
		break;
	case '2':
		termios_new.c_iflag |= IXON | IXOFF | IXANY; //software flow control   
		break;
	}

	//data bits   
	termios_new.c_cflag &= ~CSIZE;
	databit = portinfo_.databit;
	switch (databit) {
	case '5':
		termios_new.c_cflag |= CS5;
	case '6':
		termios_new.c_cflag |= CS6;
	case '7':
		termios_new.c_cflag |= CS7;
	default:
		termios_new.c_cflag |= CS8;
	}

	//parity check   
	parity = portinfo_.parity;
	switch (parity) {
	case '0':
		termios_new.c_cflag &= ~PARENB;     //no parity check   
		break;
	case '1':
		termios_new.c_cflag |= PARENB;      //odd check   
		termios_new.c_cflag &= ~PARODD;
		break;
	case '2':
		termios_new.c_cflag |= PARENB;      //even check   
		termios_new.c_cflag |= PARODD;
		break;
	}

	//stop bits   
	stopbit = portinfo_.stopbit;
	if (stopbit == '2') {
		termios_new.c_cflag |= CSTOPB;  //2 stop bits   
	}
	else {
		termios_new.c_cflag &= ~CSTOPB; //1 stop bits   
	}

	//other attributions default   
	termios_new.c_oflag &= ~OPOST;
	termios_new.c_cc[VMIN] = 1;
	termios_new.c_cc[VTIME] = 1;	//unit: (1/10)second   

	tcflush(fdcom_, TCIFLUSH);
	tmp = tcsetattr(fdcom_, TCSANOW, &termios_new);
	return tmp;
}

int Serial::PortOpen() {

	get_port();
	if (!port_.empty()) {
		fdcom_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);// | O_NONBLOCK);
	}
	else {
		std::cout << "[Serial] port_ is empty string" << std::endl;
		return -1;
	}


	return fdcom_;
}

void Serial::PortClose() {
	close(fdcom_);
}

int Serial::PortSend(void* data, int datalen)
{
	int len = 0;

	len = write(fdcom_, data, datalen);
	if (len == datalen) {
		return (len);
	}
	else {
		tcflush(fdcom_, TCOFLUSH);
		return -1;
	}
}

int Serial::PortRecv(void* data, int datalen, int baudrate)
{
	int readlen, fs_sel;
	fd_set  fs_read;
	struct timeval tv_timeout;

	FD_ZERO(&fs_read);
	FD_SET(fdcom_, &fs_read);
	tv_timeout.tv_sec = 10;//TIMEOUT_SEC(datalen, baudrate);  
	tv_timeout.tv_usec = TIMEOUT_USEC;

	fs_sel = select(fdcom_ + 1, &fs_read, NULL, NULL, &tv_timeout);
	if (fs_sel) {
		readlen = read(fdcom_, data, datalen);
		return(readlen);
	}
	else {
		return -1;
	}

	return readlen;
}

