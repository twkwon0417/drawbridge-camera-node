CC      ?= cc
CFLAGS  ?= -O2 -Wall

MQTTLIB =
ifdef MQTT
CFLAGS  += -DUSE_MQTT
MQTTLIB  = -lmosquitto
endif

all: bridge_view bridge_detect sw_bridge

bridge_view: bridge_view.c config.h tiny_jpeg.h mqtt_pub.h
	$(CC) $(CFLAGS) -o $@ bridge_view.c -lm $(MQTTLIB)

bridge_detect: bridge_detect.c config.h mqtt_pub.h
	$(CC) $(CFLAGS) -o $@ bridge_detect.c $(MQTTLIB)

sw_bridge: sw_bridge.c config.h mqtt_pub.h
	$(CC) $(CFLAGS) -o $@ sw_bridge.c $(MQTTLIB)

clean:
	rm -f bridge_view bridge_detect sw_bridge

.PHONY: all clean
