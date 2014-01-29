/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <csp/drivers/usart.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>

#include <csp/csp.h>
#include <sys/time.h>

#define EPOLL_MAX_EVENTS 16
#define MAX_USARTS 4

typedef struct {
	int fd;
	usart_callback_t usart_callback;
} usart_linux_t;

usart_linux_t usarts[MAX_USARTS];

static void *serial_rx_thread(void *vptr_args);

void usart_init(int handle, struct usart_conf * conf) {

	struct termios options;
	pthread_t rx_thread;

	usarts[handle].fd = open(conf->device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (usarts[handle].fd < 0) {
		printf("Failed to open %s: %s\r\n", conf->device, strerror(errno));
		return;
	}

	int brate = 0;
    switch(conf->baudrate) {
    case 4800:    brate=B4800;    break;
    case 9600:    brate=B9600;    break;
    case 19200:   brate=B19200;   break;
    case 38400:   brate=B38400;   break;
    case 57600:   brate=B57600;   break;
    case 115200:  brate=B115200;  break;
    case 230400:  brate=B230400;  break;
#ifndef CSP_MACOSX
    case 460800:  brate=B460800;  break;
    case 500000:  brate=B500000;  break;
    case 921600:  brate=B921600;  break;
    case 1000000: brate=B1000000; break;
    case 1500000: brate=B1500000; break;
    case 2000000: brate=B2000000; break;
    case 2500000: brate=B2500000; break;
    case 3000000: brate=B3000000; break;
#endif
    }

	tcgetattr(usarts[handle].fd, &options);
	cfsetispeed(&options, brate);
	cfsetospeed(&options, brate);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 1;
	tcsetattr(usarts[handle].fd, TCSANOW, &options);
	if (tcgetattr(usarts[handle].fd, &options) == -1)
		perror("error setting options");
	fcntl(usarts[handle].fd, F_SETFL, 0);

	/* Flush old transmissions */
	if (tcflush(usarts[handle].fd, TCIOFLUSH) == -1)
		printf("Error flushing serial port - %s(%d).\n", strerror(errno), errno);

	if (pthread_create(&rx_thread, NULL, serial_rx_thread, &usarts[handle]) != 0)
		return;

}

void usart_set_callback(int handle, usart_callback_t callback) {
	usarts[handle].usart_callback = callback;
}

void usart_insert(int handle, char c, void * pxTaskWoken) {
	putchar(c);
}

void usart_putstr(int handle, char * buf, int len) {
	if (write(usarts[handle].fd, buf, len) != len)
		return;
}

void usart_putc(int handle, char c) {
	if (write(usarts[handle].fd, &c, 1) != 1)
		return;
}

char usart_getc(int handle) {
	char c;
	if (read(usarts[handle].fd, &c, 1) != 1) return 0;
	return c;
}

int usart_messages_waiting(int handle) {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

static void *serial_rx_thread(void *vptr_args) {
	int efd, n_events, i;
	struct epoll_event event;
	struct epoll_event events[EPOLL_MAX_EVENTS];
	ssize_t count;
	char buf[512];
	uint32_t e;
	usart_linux_t *usart = (usart_linux_t *) vptr_args;

	efd = epoll_create1(0);
	event.data.fd = usart->fd;
	event.events = EPOLLIN | EPOLLET;
	if (epoll_ctl(efd, EPOLL_CTL_ADD, usart->fd, &event) == -1) {
		printf("Error adding epoll descriptor: %s\r\n", strerror(errno));
		return NULL;
	}

	// Receive loop
	while (1) {
		n_events = epoll_wait(efd, events, EPOLL_MAX_EVENTS, -1);
		for (i = 0; i < n_events; i++) {
			e = events[i].events;
			if ((e & EPOLLERR) || (e & EPOLLHUP) || (!(e & EPOLLIN))) {
				printf("Epoll file descriptor error\n");
				close(events[i].data.fd);
				continue;
			}

			if (usart->fd != events[i].data.fd) {
				continue;
			}

			while (1) {
				count = read(events[i].data.fd, buf, sizeof(buf));
				if (count <= 0 || !usart->usart_callback) {
					break;
				}

				usart->usart_callback(buf, count, NULL);
			}
		}
	}

	return NULL;
}
