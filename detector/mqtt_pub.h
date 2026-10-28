#ifndef MQTT_PUB_H
#define MQTT_PUB_H

#include <string.h>
#include <stdio.h>
#include "config.h"

#ifdef USE_MQTT
#include <mosquitto.h>

static struct mosquitto *g_mosq = NULL;
static const char *g_mqtt_topic = MQTT_TOPIC;

static int mqtt_start(const char *broker, int port, const char *topic)
{
	if (!broker || !broker[0]) return 0;
	if (topic && topic[0]) g_mqtt_topic = topic;
	mosquitto_lib_init();
	g_mosq = mosquitto_new(NULL , true , NULL);
	if (!g_mosq) { fprintf(stderr, "MQTT: mosquitto_new 실패\n"); return -1; }
	if (mosquitto_connect(g_mosq, broker, port, 60) != MOSQ_ERR_SUCCESS) {
		fprintf(stderr, "MQTT: 브로커 연결 실패 %s:%d (영상/검출은 계속)\n", broker, port);
		mosquitto_destroy(g_mosq); g_mosq = NULL;
		return -1;
	}
	mosquitto_loop_start(g_mosq);
	fprintf(stderr, "MQTT: 연결됨 %s:%d  topic=%s\n", broker, port, g_mqtt_topic);
	return 0;
}

static void mqtt_pub_status(const int *active, int n)
{
	char msg[N_SECTIONS + 1];
	if (n > N_SECTIONS) n = N_SECTIONS;
	for (int i = 0; i < n; i++) msg[i] = active[i] ? '1' : '0';
	msg[n] = '\0';

	if (g_mosq) {
		mosquitto_publish(g_mosq, NULL, g_mqtt_topic, (int)strlen(msg), msg, 1 , false );
		printf("[MQTT] 전송 %s → topic=%s\n", msg, g_mqtt_topic);
	} else {
		printf("[MQTT] 전송 %s → topic=%s (오프라인: 로컬 로그만)\n", msg, g_mqtt_topic);
	}
	fflush(stdout);
}

static void mqtt_pub_msg(const char *topic, const char *payload)
{
	if (!g_mosq || !topic || !payload) return;
	mosquitto_publish(g_mosq, NULL, topic,
	                  (int)strlen(payload), payload, 1 , false );
}

static void mqtt_stop(void)
{
	if (g_mosq) {
		mosquitto_loop_stop(g_mosq, true);
		mosquitto_disconnect(g_mosq);
		mosquitto_destroy(g_mosq);
		g_mosq = NULL;
	}
	mosquitto_lib_cleanup();
}

#else
static int  mqtt_start(const char *broker, int port, const char *topic)
{ (void)broker; (void)port; (void)topic; return 0; }

static void mqtt_pub_status(const int *active, int n)
{
	char msg[N_SECTIONS + 1];
	if (n > N_SECTIONS) n = N_SECTIONS;
	for (int i = 0; i < n; i++) msg[i] = active[i] ? '1' : '0';
	msg[n] = '\0';
	printf("[MQTT] 전송 %s → topic=%s (오프라인: 빌드 비활성)\n", msg, MQTT_TOPIC);
	fflush(stdout);
}

static void mqtt_pub_msg(const char *topic, const char *payload) { (void)topic; (void)payload; }
static void mqtt_stop(void) {}
#endif

#endif
