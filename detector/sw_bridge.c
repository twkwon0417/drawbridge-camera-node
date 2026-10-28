#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include "config.h"
#include "mqtt_pub.h"

#define DEV_PATH "/dev/manual_sw"

static volatile int running = 1;

static void sig_handler(int s) { (void)s; running = 0; }

int main(int argc, char *argv[])
{
	char broker[128] = MQTT_BROKER;
	int  port        = MQTT_PORT;
	int  opt;

	while ((opt = getopt(argc, argv, "b:h")) != -1) {
		switch (opt) {
		case 'b': {
			char *colon = strchr(optarg, ':');
			if (colon) { *colon = '\0'; port = atoi(colon + 1); }
			strncpy(broker, optarg, sizeof(broker) - 1);
			broker[sizeof(broker) - 1] = '\0';
			break;
		}
		case 'h':
		default:
			fprintf(stderr,
			        "사용: %s [-b broker[:port]]\n"
			        "  -b  Pi2 MQTT 브로커 주소 (예: 10.10.10.11 또는 10.10.10.11:1883)\n",
			        argv[0]);
			return 1;
		}
	}

	int fd = open(DEV_PATH, O_RDONLY);
	if (fd < 0) {
		perror("open " DEV_PATH);
		fprintf(stderr, "manual_switch.ko 가 적재돼 있는지 확인: lsmod | grep manual_switch\n");
		return 1;
	}

	mqtt_start(broker, port, MQTT_TOPIC_CTRL);

	signal(SIGINT,  sig_handler);
	signal(SIGTERM, sig_handler);

	printf("sw_bridge: 시작 — %s  토픽=%s\n", DEV_PATH, MQTT_TOPIC_CTRL);
	fflush(stdout);

	struct pollfd pfd = { .fd = fd, .events = POLLIN };

	while (running) {
		int r = poll(&pfd, 1, 500);
		if (r < 0) break;
		if (r == 0) continue;

		char c;
		if (read(fd, &c, 1) != 1) break;

		const char *msg = (c == 'U') ? "MANUAL_UP" :
		                  (c == 'D') ? "MANUAL_DOWN" : NULL;
		if (!msg) continue;

		printf("sw_bridge: %s\n", msg);
		fflush(stdout);
		mqtt_pub_msg(MQTT_TOPIC_CTRL, msg);
	}

	mqtt_stop();
	close(fd);
	printf("sw_bridge: 종료\n");
	return 0;
}
