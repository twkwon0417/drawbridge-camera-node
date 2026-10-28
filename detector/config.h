#ifndef KUSTAR_CONFIG_H
#define KUSTAR_CONFIG_H
#include <stdint.h>

#define FOURCC(a,b,c,d) ((uint32_t)(a)|((uint32_t)(b)<<8)|((uint32_t)(c)<<16)|((uint32_t)(d)<<24))
#define CFG_FOURCC   FOURCC('B','A','8','1')
#define CFG_W        640
#define CFG_H        480

#define N_SECTIONS    3
#define SECTION_WIDTH 150
#define SECTION_TOP   18

#define DEF_FG_THR    18
#define DEF_MOT_THR   14
#define DEF_OCC_PM    120
#define DEF_MOT_PM    60
#define DEF_BG_ALPHA_SHIFT 6
#define DEF_WARMUP_SEC 2.0

#define DEF_HTTP_PORT    8080
#define DEF_JPEG_QUALITY 80

#define MQTT_BROKER      "10.10.10.13"
#define MQTT_PORT        1883

#define MQTT_TOPIC       "drawbridge/pi3/event"
#define MQTT_TOPIC_CTRL  "drawbridge/pi3/event"
#define MQTT_STATUS_SEC  1.0

#endif
