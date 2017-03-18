#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>
#include <math.h>

#include <wiringPi.h>

#define MYPORT "4950"    // the port users will be connecting to

#define MAXBUFLEN 100

#define Motor1A 0
#define Motor1B 2
#define Motor1E 3
#define Motor2A 4
#define Motor2B 5
#define Motor2E 6

using namespace std;

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void init_pins()
{
	//Left Wheel
	pinMode(Motor1A, OUTPUT);
	pinMode(Motor1B, OUTPUT);
	pinMode(Motor1E, OUTPUT);
	
	//Right Wheel
	pinMode(Motor2A, OUTPUT);
	pinMode(Motor2B, OUTPUT);
	pinMode(Motor2E, OUTPUT);
	
	pinMode(1, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS); 
	pwmSetClock(384); //clock at 50kHz (20us tick)
	pwmSetRange(1000); //range at 1000 ticks (20ms) 
}

int main(void)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
    struct sockaddr_storage their_addr;
    char buf[MAXBUFLEN];
    socklen_t addr_len;
    char s[INET6_ADDRSTRLEN];

	//enable GPIO for Raspberry Pi
	wiringPiSetup();
	init_pins();

	//enable both wheels
	digitalWrite(Motor1E, HIGH);
	digitalWrite(Motor2E, HIGH); 

	//this variable has two usage, the first one is the time(ms) that wheels turns
	//the second usage is the angle of the robot arm
	int time;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, MYPORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("listener: socket");
            continue;
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("listener: bind");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "listener: failed to bind socket\n");
        return 2;
    }

    freeaddrinfo(servinfo);
	while(1)
	{
		printf("listener: waiting to recvfrom...\n");

		addr_len = sizeof their_addr;
		if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
			(struct sockaddr *)&their_addr, &addr_len)) == -1) {
			perror("recvfrom");
			exit(1);
		}

		printf("listener: got packet from %s\n",
			inet_ntop(their_addr.ss_family,
				get_in_addr((struct sockaddr *)&their_addr),
				s, sizeof s));
		printf("listener: packet is %d bytes long\n", numbytes);
		buf[numbytes] = '\0';

		//print the received massage
		//valid message should have a number come after a uppercase character
		//for example : F1000, B500, L1200, R550, A100
		printf("listener: packet contains \"%s\"\n", buf);
		printf("%c \n", buf[0]);

		const char *message;
		message = buf;
		string data(message);

		cout << data << endl; 
		string sub_data = data.substr(1,5);

		time = stoi(sub_data) + 1;
		string hi = to_string(time);
		time = stoi(hi)+1;
		const char *cstr = hi.c_str();
		cout << cstr << endl;
		
		//if the first character of the received message is "F"
		//control wheels to move Forward
		if(buf[0] == 'F')
		{
			printf("%c \n", buf[0]);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, HIGH);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, HIGH);
			delay(time);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, LOW);
		}

		//if the first character of the received message is "L"
		//control wheels to turn Left
		if(buf[0] == 'L')
		{
			printf("%c \n", buf[0]);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, HIGH);
			digitalWrite(Motor2A, HIGH);
			digitalWrite(Motor2B, LOW);
			delay(time);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, LOW);
		}

		//if the first character of the received message is "R"
		//control wheels to turn Right
		if(buf[0] == 'R')
		{
			printf("%c \n", buf[0]);
			digitalWrite(Motor1A, HIGH);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, HIGH);
			delay(time);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, LOW);
		}
	
		//if the first character of the received message is "B"
		//control wheels to move Backward
		if(buf[0] == 'B')
		{
			printf("%c \n", buf[0]);
			digitalWrite(Motor1A, HIGH);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, HIGH);
			digitalWrite(Motor2B, LOW);
			delay(time);
			digitalWrite(Motor1A, LOW);
			digitalWrite(Motor1B, LOW);
			digitalWrite(Motor2A, LOW);
			digitalWrite(Motor2B, LOW);
		}

		//if the first character of the received message is "F"
		//control servo motor of the robot arm
		if(buf[0] == 'A')
		{
			printf("%c \n", buf[0]);
			pwmWrite(1, time);
		}

		//quit the program
		if(buf[0] == 'Q')
		{
			printf("%c", buf[0]);
			break;
		}
	
	}
    close(sockfd);

    return 0;
}
