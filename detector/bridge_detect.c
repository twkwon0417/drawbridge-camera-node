#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#if defined(__linux__)
#define HAVE_V4L2 1
#include <linux/videodev2.h>
#else
#define HAVE_V4L2 0
#endif
#include "config.h"
#include "mqtt_pub.h"

typedef struct { int x, y, w, h; } rect_t;

static char   cfg_dev[256]  = "/dev/video0";
static char   cfg_file[256] = "";
static int    cfg_W = CFG_W, cfg_H = CFG_H;
static int    cfg_loop_file = 1;

static uint32_t cfg_fourcc = CFG_FOURCC;
static double cfg_warmup_sec = DEF_WARMUP_SEC;
static char   cfg_mqtt_broker[128] = MQTT_BROKER;
static int    cfg_mqtt_port = MQTT_PORT;

static int    FG_THR  = DEF_FG_THR;
static int    MOT_THR = DEF_MOT_THR;
static int    OCC_PM  = DEF_OCC_PM;
static int    MOT_PM  = DEF_MOT_PM;
static int    BG_ALPHA_SHIFT = DEF_BG_ALPHA_SHIFT;

static int    cfg_secw = SECTION_WIDTH;
static rect_t sectionRectFull[N_SECTIONS];

#ifdef USE_GPIO
static char cfg_gpiochip[64] = "gpiochip0";
static int  cfg_bridge_line  = 17;
static int  cfg_road_line    = 27;
#endif

static int      GW, GH;
static uint8_t *g_cur, *g_prev;
static int32_t *g_bg;
static int      bg_init = 0;
static rect_t   sectionR[N_SECTIONS];

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

static double now_sec(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

#ifdef USE_GPIO
#include <gpiod.h>
static struct gpiod_chip *g_chip;
static struct gpiod_line *g_ln_bridge, *g_ln_road;

static int gpio_setup(void)
{
	g_chip = gpiod_chip_open_by_name(cfg_gpiochip);
	if (!g_chip) { perror("gpiod_chip_open"); return -1; }
	g_ln_bridge = gpiod_chip_get_line(g_chip, cfg_bridge_line);
	g_ln_road   = gpiod_chip_get_line(g_chip, cfg_road_line);
	if (!g_ln_bridge || !g_ln_road) { perror("get_line"); return -1; }
	if (gpiod_line_request_output(g_ln_bridge, "bridge", 0) < 0 ||
	    gpiod_line_request_output(g_ln_road,   "road",   0) < 0) {
		perror("request_output");
		return -1;
	}
	return 0;
}
static void gpio_close(void) { if (g_chip) gpiod_chip_close(g_chip); }
static void apply_outputs(int up, int stop)
{
	if (g_ln_bridge) gpiod_line_set_value(g_ln_bridge, up   ? 1 : 0);
	if (g_ln_road)   gpiod_line_set_value(g_ln_road,   stop ? 1 : 0);
}
#else
static int  gpio_setup(void) { return 0; }
static void gpio_close(void) {}
static void apply_outputs(int up, int stop) { (void)up; (void)stop; }
#endif

static rect_t half_rect(rect_t r)
{
	rect_t o = { r.x / 2, r.y / 2, r.w / 2, r.h / 2 };
	return o;
}
static void clamp_rect(rect_t *r)
{
	if (r->x < 0) r->x = 0;
	if (r->y < 0) r->y = 0;
	if (r->x + r->w > GW) r->w = GW - r->x;
	if (r->y + r->h > GH) r->h = GH - r->y;
	if (r->w < 0) r->w = 0;
	if (r->h < 0) r->h = 0;
}
static int setup_buffers(int W, int H)
{
	GW = W / 2;
	GH = H / 2;
	g_cur  = calloc((size_t)GW * GH, 1);
	g_prev = calloc((size_t)GW * GH, 1);
	g_bg   = calloc((size_t)GW * GH, sizeof(int32_t));
	if (!g_cur || !g_prev || !g_bg) { fprintf(stderr, "메모리 부족\n"); return -1; }

	int secw = cfg_secw; if (secw < 8) secw = 8; if (secw * N_SECTIONS > W) secw = W / N_SECTIONS;
	int gap = (W - secw * N_SECTIONS) / (N_SECTIONS + 1); if (gap < 0) gap = 0;

	int L1 = gap + secw, L2 = W - gap - secw;
	int sx[3] = { 0,  L1,      L2     };
	int sw[3] = { L1, L2 - L1, W - L2 };
	for (int i = 0; i < N_SECTIONS; i++) {
		rect_t r = { sx[i], SECTION_TOP, sw[i], H - SECTION_TOP };
		sectionRectFull[i] = r;
		sectionR[i] = half_rect(r);
		clamp_rect(&sectionR[i]);
	}
	bg_init = 0;
	return 0;
}

static void downsample_bayer(const uint8_t *data, int stride)
{
	for (int by = 0; by < GH; by++) {
		const uint8_t *r0 = data + (size_t)(2 * by)     * stride;
		const uint8_t *r1 = data + (size_t)(2 * by + 1) * stride;
		uint8_t *out = g_cur + (size_t)by * GW;
		for (int bx = 0; bx < GW; bx++) {
			int c0 = 2 * bx, c1 = 2 * bx + 1;
			int s = r0[c0] + r0[c1] + r1[c0] + r1[c1];
			out[bx] = (uint8_t)(s >> 2);
		}
	}
}

static void roi_stats(rect_t r, int *occ_pm, int *mot_pm)
{
	long area = 0, fg = 0, mot = 0;
	for (int y = r.y; y < r.y + r.h; y++) {
		const uint8_t *cu = g_cur  + (size_t)y * GW;
		const uint8_t *pv = g_prev + (size_t)y * GW;
		const int32_t *bg = g_bg   + (size_t)y * GW;
		for (int x = r.x; x < r.x + r.w; x++) {
			int c = cu[x];
			int b = bg[x] >> 8;
			int d = c - b;  if (d < 0) d = -d;
			if (d > FG_THR) fg++;
			int dm = c - pv[x]; if (dm < 0) dm = -dm;
			if (dm > MOT_THR) mot++;
			area++;
		}
	}
	*occ_pm = area ? (int)((fg  * 1000) / area) : 0;
	*mot_pm = area ? (int)((mot * 1000) / area) : 0;
}

static void init_models(void)
{
	size_t n = (size_t)GW * GH;
	for (size_t i = 0; i < n; i++)
		g_bg[i] = ((int32_t)g_cur[i]) << 8;
	memcpy(g_prev, g_cur, n);
	bg_init = 1;
}
static void update_models(void)
{
	size_t n = (size_t)GW * GH;
	for (size_t i = 0; i < n; i++) {
		int32_t target = ((int32_t)g_cur[i]) << 8;

		g_bg[i] += (target - g_bg[i]) >> BG_ALPHA_SHIFT;
	}
	memcpy(g_prev, g_cur, n);
}

static void on_frame(const uint8_t *data, int stride)
{
	static double last_status = 0;
	static double warmup_until = -1;
	double t = now_sec();

	downsample_bayer(data, stride);

	if (!bg_init) {
		init_models();
		warmup_until = t + cfg_warmup_sec;
		return;
	}

	if (t < warmup_until) {
		init_models();
		if (t - last_status >= 1.0) {
			last_status = t;
			printf("[%.3f] 워밍업... 배경 학습 중 (판단 보류, %.1fs 남음)\n",
			       t, warmup_until - t);
			fflush(stdout);
		}
		return;
	}

	int sec_occ[N_SECTIONS], sec_mot[N_SECTIONS], sec_active[N_SECTIONS];
	int any_active = 0;
	for (int i = 0; i < N_SECTIONS; i++) {
		roi_stats(sectionR[i], &sec_occ[i], &sec_mot[i]);
		sec_active[i] = (sec_occ[i] > OCC_PM) || (sec_mot[i] > MOT_PM);
		if (sec_active[i]) any_active = 1;
	}

	static int prev_active[N_SECTIONS] = { 0 };
	static int first_done = 0;
	for (int i = 0; i < N_SECTIONS; i++) {
		if (first_done && sec_active[i] != prev_active[i]) {
			printf("[%.3f] 섹션%d %s (occ %d‰ mot %d‰)\n",
			       t, i, sec_active[i] ? "변화" : "정지", sec_occ[i], sec_mot[i]);
			fflush(stdout);
		}
		prev_active[i] = sec_active[i];
	}
	first_done = 1;

	apply_outputs(0, any_active);

	if (t - last_status >= MQTT_STATUS_SEC) {
		last_status = t;
		printf("[%.3f] 섹션", t);
		for (int i = 0; i < N_SECTIONS; i++)
			printf(" [%d:%s %d‰/%d‰]", i,
			       sec_active[i] ? "감지" : "빔",
			       sec_occ[i], sec_mot[i]);
		printf("\n");
		mqtt_pub_status(sec_active, N_SECTIONS);
	}

	update_models();
}

#if HAVE_V4L2
static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;
	do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
	return r;
}

struct vbuf { void *start; size_t length; };

static int run_v4l2(void)
{
	int fd = open(cfg_dev, O_RDWR | O_NONBLOCK);
	if (fd < 0) { perror("open(dev)"); return -1; }

	struct v4l2_capability cap;
	memset(&cap, 0, sizeof(cap));
	if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("QUERYCAP"); goto err; }
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
	    !(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "캡처/스트리밍 미지원 장치\n");
		goto err;
	}

	struct v4l2_format fmt;
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = cfg_W;
	fmt.fmt.pix.height      = cfg_H;
	fmt.fmt.pix.pixelformat = cfg_fourcc;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); goto err; }

	int W = fmt.fmt.pix.width, H = fmt.fmt.pix.height;
	int stride = fmt.fmt.pix.bytesperline ? (int)fmt.fmt.pix.bytesperline : W;
	if (fmt.fmt.pix.pixelformat != cfg_fourcc)
		fprintf(stderr, "경고: 요청과 다른 포맷 반환(0x%08x)\n",
			fmt.fmt.pix.pixelformat);
	printf("V4L2: %dx%d stride=%d fourcc=0x%08x (검출은 Y/gray 평면 사용)\n",
	       W, H, stride, fmt.fmt.pix.pixelformat);
	if (setup_buffers(W, H) < 0) goto err;

	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(req));
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) { perror("REQBUFS"); goto err; }
	if (req.count < 2) { fprintf(stderr, "버퍼 부족\n"); goto err; }

	struct vbuf *bufs = calloc(req.count, sizeof(*bufs));
	for (unsigned i = 0; i < req.count; i++) {
		struct v4l2_buffer b;
		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); goto err; }
		bufs[i].length = b.length;
		bufs[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
				     MAP_SHARED, fd, b.m.offset);
		if (bufs[i].start == MAP_FAILED) { perror("mmap"); goto err; }
	}
	for (unsigned i = 0; i < req.count; i++) {
		struct v4l2_buffer b;
		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); goto err; }
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); goto err; }
	printf("스트리밍 시작 (Ctrl-C 로 종료)\n");

	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	while (!g_stop) {
		int pr = poll(&pfd, 1, 1000);
		if (pr < 0) { if (errno == EINTR) continue; perror("poll"); break; }
		if (pr == 0) continue;

		struct v4l2_buffer b;
		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
			if (errno == EAGAIN) continue;
			perror("DQBUF"); break;
		}
		on_frame(bufs[b.index].start, stride);
		if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); break; }
	}

	xioctl(fd, VIDIOC_STREAMOFF, &type);
	for (unsigned i = 0; i < req.count; i++)
		if (bufs[i].start && bufs[i].start != MAP_FAILED)
			munmap(bufs[i].start, bufs[i].length);
	free(bufs);
	close(fd);
	return 0;
err:
	close(fd);
	return -1;
}

#else
static int run_v4l2(void)
{
	return -1;
}
#endif

static int run_file(void)
{
	if (setup_buffers(cfg_W, cfg_H) < 0) return -1;
	FILE *f = fopen(cfg_file, "rb");
	if (!f) { perror("fopen(file)"); return -1; }

	size_t fsz = (size_t)cfg_W * cfg_H;
	uint8_t *buf = malloc(fsz);
	if (!buf) { fclose(f); return -1; }

	printf("파일 재생: %s (%dx%d, %zu bytes/frame, ~15fps 흉내)\n",
	       cfg_file, cfg_W, cfg_H, fsz);

	while (!g_stop) {
		size_t n = fread(buf, 1, fsz, f);
		if (n < fsz) {
			if (cfg_loop_file && n == 0) { rewind(f); continue; }
			break;
		}
		on_frame(buf, cfg_W);
		struct timespec ts = { 0, 66 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}
	free(buf);
	fclose(f);
	return 0;
}

static void usage(const char *p)
{

}

int main(int argc, char **argv)
{
	int c;
	while ((c = getopt(argc, argv, "d:f:W:H:s:t:m:o:M:p:u:b:1h")) != -1) {
		switch (c) {
		case 'd': snprintf(cfg_dev,  sizeof(cfg_dev),  "%s", optarg); break;
		case 'f': snprintf(cfg_file, sizeof(cfg_file), "%s", optarg); break;
		case 'W': cfg_W = atoi(optarg); break;
		case 'H': cfg_H = atoi(optarg); break;
		case 's': cfg_secw = atoi(optarg); break;
		case 't': FG_THR  = atoi(optarg); break;
		case 'm': MOT_THR = atoi(optarg); break;
		case 'o': OCC_PM  = atoi(optarg); break;
		case 'M': MOT_PM  = atoi(optarg); break;
		case 'p':
			if (strlen(optarg) >= 4)
				cfg_fourcc = FOURCC(optarg[0], optarg[1],
						    optarg[2], optarg[3]);
			break;
		case 'u': cfg_warmup_sec = atof(optarg); break;
		case 'b': { char *cl = strchr(optarg, ':'); if (cl) { *cl = 0; cfg_mqtt_port = atoi(cl + 1); } snprintf(cfg_mqtt_broker, sizeof(cfg_mqtt_broker), "%s", optarg); } break;
		case '1': cfg_loop_file = 0; break;
		case 'h': default: usage(argv[0]); return (c == 'h') ? 0 : 1;
		}
	}

	signal(SIGINT,  on_sigint);
	signal(SIGTERM, on_sigint);

	printf("세로 3섹션 감지기 시작 — driver 미수정, raw Bayer 소비\n");
	printf("섹션: %d개, 각 폭 %dpx (런타임 -s 로 조절)\n", N_SECTIONS, cfg_secw);
	mqtt_start(cfg_mqtt_broker, cfg_mqtt_port, MQTT_TOPIC);

	if (gpio_setup() < 0)
		fprintf(stderr, "경고: GPIO 초기화 실패 — 출력 없이 진행\n");

	int rc = cfg_file[0] ? run_file() : run_v4l2();

	apply_outputs(0, 0);
	gpio_close();
	mqtt_stop();
	printf("\n종료\n");
	return rc ? 1 : 0;
}
