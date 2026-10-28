#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <linux/videodev2.h>
#include "config.h"
#include "tiny_jpeg.h"
#include "mqtt_pub.h"

static char cfg_dev[256] = "/dev/video0";
static int  cfg_W = CFG_W, cfg_H = CFG_H, cfg_port = DEF_HTTP_PORT, cfg_quality = DEF_JPEG_QUALITY;
static int  FG_THR = DEF_FG_THR, MOT_THR = DEF_MOT_THR, OCC_PM = DEF_OCC_PM, MOT_PM = DEF_MOT_PM, BG_ALPHA_SHIFT = DEF_BG_ALPHA_SHIFT;
static double cfg_warmup_sec = DEF_WARMUP_SEC;
static uint32_t cfg_fourcc = CFG_FOURCC;
static char cfg_mqtt_broker[128] = MQTT_BROKER;
static int  cfg_mqtt_port = MQTT_PORT;

typedef struct { int x, y, w, h; } rect_t;

static int cfg_secw = SECTION_WIDTH;
static rect_t sectionRectFull[N_SECTIONS];

static int GW, GH;
static uint8_t *g_cur, *g_prev;
static int32_t *g_bg;
static int bg_init = 0;
static rect_t sectionR[N_SECTIONS];

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s){ (void)s; g_stop = 1; }
static double now_sec(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec + t.tv_nsec*1e-9; }
static int xioctl(int fd, unsigned long r, void *a){ int x; do{x=ioctl(fd,r,a);}while(x==-1&&errno==EINTR); return x; }

static rect_t half_rect(rect_t r){ rect_t o={r.x/2,r.y/2,r.w/2,r.h/2}; return o; }
static void clamp_rect(rect_t*r){ if(r->x<0)r->x=0; if(r->y<0)r->y=0; if(r->x+r->w>GW)r->w=GW-r->x; if(r->y+r->h>GH)r->h=GH-r->y; if(r->w<0)r->w=0; if(r->h<0)r->h=0; }
static int setup_buffers(int W,int H){
	GW=W/2; GH=H/2;
	g_cur=calloc((size_t)GW*GH,1); g_prev=calloc((size_t)GW*GH,1); g_bg=calloc((size_t)GW*GH,sizeof(int32_t));
	if(!g_cur||!g_prev||!g_bg) return -1;

	int secw=cfg_secw; if(secw<8)secw=8; if(secw*N_SECTIONS>W)secw=W/N_SECTIONS;
	int gap=(W-secw*N_SECTIONS)/(N_SECTIONS+1); if(gap<0)gap=0;
	int L1=gap+secw, L2=W-gap-secw;
	int sx[3]={ 0, L1, L2 }, sw[3]={ L1, L2-L1, W-L2 };
	for(int i=0;i<N_SECTIONS;i++){
		rect_t r={ sx[i], SECTION_TOP, sw[i], H-SECTION_TOP };
		sectionRectFull[i]=r;
		sectionR[i]=half_rect(r); clamp_rect(&sectionR[i]);
	}
	bg_init=0; return 0;
}

static void downsample_y(const uint8_t *y, int stride){
	for(int by=0;by<GH;by++){ const uint8_t*r0=y+(size_t)(2*by)*stride, *r1=y+(size_t)(2*by+1)*stride; uint8_t*o=g_cur+(size_t)by*GW;
		for(int bx=0;bx<GW;bx++){ int c0=2*bx,c1=2*bx+1; o[bx]=(uint8_t)((r0[c0]+r0[c1]+r1[c0]+r1[c1])>>2); } }
}
static void roi_stats(rect_t r,int*occ,int*mot){ long area=0,fg=0,mo=0;
	for(int y=r.y;y<r.y+r.h;y++){ const uint8_t*cu=g_cur+(size_t)y*GW,*pv=g_prev+(size_t)y*GW; const int32_t*bg=g_bg+(size_t)y*GW;
		for(int x=r.x;x<r.x+r.w;x++){ int c=cu[x],b=bg[x]>>8,d=c-b; if(d<0)d=-d; if(d>FG_THR)fg++; int dm=c-pv[x]; if(dm<0)dm=-dm; if(dm>MOT_THR)mo++; area++; } }
	*occ=area?(int)(fg*1000/area):0; *mot=area?(int)(mo*1000/area):0;
}
static void init_models(void){ size_t n=(size_t)GW*GH; for(size_t i=0;i<n;i++) g_bg[i]=((int32_t)g_cur[i])<<8; memcpy(g_prev,g_cur,n); bg_init=1; }
static void update_models(void){ size_t n=(size_t)GW*GH; for(size_t i=0;i<n;i++){ int32_t t=((int32_t)g_cur[i])<<8; g_bg[i]+=(t-g_bg[i])>>BG_ALPHA_SHIFT; } memcpy(g_prev,g_cur,n); }

static double warmup_until=-1;
static int o_warm=1;
static int sec_occ[N_SECTIONS], sec_mot[N_SECTIONS], sec_active[N_SECTIONS];

static void detect_step(const uint8_t *y, int stride){
	double t = now_sec();
	downsample_y(y, stride);
	if(!bg_init){ init_models(); warmup_until=t+cfg_warmup_sec; o_warm=1; return; }
	if(t<warmup_until){ init_models(); o_warm=1; return; }
	o_warm=0;
	for(int i=0;i<N_SECTIONS;i++){
		roi_stats(sectionR[i], &sec_occ[i], &sec_mot[i]);
		sec_active[i] = (sec_occ[i]>OCC_PM) || (sec_mot[i]>MOT_PM);
	}

	static double last_pub=0;
	if(t-last_pub >= MQTT_STATUS_SEC){ last_pub=t; mqtt_pub_status(sec_active, N_SECTIONS); }
	update_models();
}

static void yuv420_to_rgb(const uint8_t *buf, int stride, int W, int H, uint8_t *rgb){
	const uint8_t *Y=buf, *U=buf+(size_t)stride*H, *V=U+(size_t)(stride/2)*(H/2);
	int cs=stride/2;
	for(int y=0;y<H;y++){ for(int x=0;x<W;x++){
		int yy=Y[(size_t)y*stride+x]-16; int u=U[(size_t)(y/2)*cs+(x/2)]-128, v=V[(size_t)(y/2)*cs+(x/2)]-128;
		int c=298*yy; int r=(c+409*v+128)>>8, g=(c-100*u-208*v+128)>>8, b=(c+516*u+128)>>8;
		uint8_t*p=rgb+((size_t)y*W+x)*3;
		p[0]=r<0?0:r>255?255:r; p[1]=g<0?0:g>255?255:g; p[2]=b<0?0:b>255?255:b;
	} }
}

static void bayer_to_rgb(const uint8_t *buf, int stride, int W, int H, uint8_t *rgb){
	for(int by=0;by<H/2;by++){
		const uint8_t *r0=buf+(size_t)(2*by)*stride, *r1=buf+(size_t)(2*by+1)*stride;
		for(int bx=0;bx<W/2;bx++){
			int B=r0[2*bx], G=(r0[2*bx+1]+r1[2*bx])>>1, R=r1[2*bx+1];
			for(int dy=0;dy<2;dy++){ uint8_t*p=rgb+((size_t)(2*by+dy)*W+2*bx)*3;
				p[0]=R;p[1]=G;p[2]=B; p[3]=R;p[4]=G;p[5]=B; }
		}
	}
}

static void px(uint8_t*rgb,int W,int H,int x,int y,int r,int g,int b){ if(x<0||y<0||x>=W||y>=H)return; uint8_t*p=rgb+((size_t)y*W+x)*3; p[0]=r;p[1]=g;p[2]=b; }
static void draw_rect(uint8_t*rgb,int W,int H,rect_t r,int th,int cr,int cg,int cb){
	for(int t=0;t<th;t++){ for(int x=r.x;x<r.x+r.w;x++){ px(rgb,W,H,x,r.y+t,cr,cg,cb); px(rgb,W,H,x,r.y+r.h-1-t,cr,cg,cb);} for(int y=r.y;y<r.y+r.h;y++){ px(rgb,W,H,r.x+t,y,cr,cg,cb); px(rgb,W,H,r.x+r.w-1-t,y,cr,cg,cb);} }
}
static void fill_bar(uint8_t*rgb,int W,int H,int h,int cr,int cg,int cb){ for(int y=0;y<h&&y<H;y++)for(int x=0;x<W;x++)px(rgb,W,H,x,y,cr,cg,cb); }

static void annotate(uint8_t*rgb,int W,int H){
	int anyact=0; for(int i=0;i<N_SECTIONS;i++) if(sec_active[i]) anyact=1;

	if(o_warm)       fill_bar(rgb,W,H,16, 80,80,80);
	else if(!anyact) fill_bar(rgb,W,H,16, 0,170,0);
	else             fill_bar(rgb,W,H,16, 220,0,0);

	for(int i=0;i<N_SECTIONS;i++){
		int cr=0,cg=170,cb=0;
		if(sec_active[i]){ cr=220;cg=0;cb=0; }
		draw_rect(rgb,W,H,sectionRectFull[i],3,cr,cg,cb);
	}
}

static int write_all(int fd,const void*b,size_t n){ const char*p=b; while(n){ ssize_t w=write(fd,p,n); if(w<0){ if(errno==EINTR)continue; return -1;} if(w==0)return -1; p+=w; n-=w; } return 0; }
#define BND "bvframe"
struct vbuf { void*start; size_t len; };

static void serve_jpeg(int cfd,const uint8_t*jpg,unsigned long jn){
	char h[200]; int hn=snprintf(h,sizeof h,
		"HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n"
		"Cache-Control: no-store\r\nConnection: close\r\n\r\n",jn);
	if(write_all(cfd,h,hn)==0) write_all(cfd,jpg,jn);
}

static void serve_html(int cfd){
	char pg[1500]; int n=snprintf(pg,sizeof pg,
		"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
		"<!doctype html><html><head><meta charset=utf-8><title>도개교 실시간</title></head>"
		"<body style='margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center'>"
		"<img id=v style='max-width:100%%;height:auto'>"
		"<script>"
		"var v=document.getElementById('v');"
		"function nx(){v.src='/snapshot?t='+Date.now();}"
		"v.onload=function(){setTimeout(nx,120);};"
		"v.onerror=function(){setTimeout(nx,1000);};"
		"nx();"
		"</script>"
		"<div style='padding:8px;font-size:14px'>"
		"세로 3섹션 검출 &nbsp;|&nbsp; 각 섹션: <b style='color:#0a0'>초록=비어있음</b> / <b style='color:#e00'>빨강=감지(점유·움직임)</b>"
		" &nbsp;|&nbsp; 상태바: 초록=전부 비어있음 / 빨강=하나라도 감지</div></body></html>");
	write_all(cfd,pg,n);
}

struct cam { int fd; struct vbuf *bufs; unsigned nbuf; int W,H,stride; uint32_t vfmt; };
static void cam_close(struct cam *c){
	if(c->fd<0) return;
	enum v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(c->fd,VIDIOC_STREAMOFF,&t);
	if(c->bufs){ for(unsigned i=0;i<c->nbuf;i++) if(c->bufs[i].start&&c->bufs[i].start!=MAP_FAILED) munmap(c->bufs[i].start,c->bufs[i].len); free(c->bufs); c->bufs=NULL; }
	close(c->fd); c->fd=-1;
}
static int cam_open(struct cam *c){
	memset(c,0,sizeof *c); c->fd=open(cfg_dev,O_RDWR|O_NONBLOCK); if(c->fd<0){ perror("open"); return -1; }
	struct v4l2_format fmt; memset(&fmt,0,sizeof fmt);
	fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; fmt.fmt.pix.width=cfg_W; fmt.fmt.pix.height=cfg_H;
	fmt.fmt.pix.pixelformat=cfg_fourcc; fmt.fmt.pix.field=V4L2_FIELD_NONE;
	if(xioctl(c->fd,VIDIOC_S_FMT,&fmt)<0){ perror("S_FMT"); close(c->fd); c->fd=-1; return -1; }
	c->W=fmt.fmt.pix.width; c->H=fmt.fmt.pix.height;
	c->stride=fmt.fmt.pix.bytesperline?fmt.fmt.pix.bytesperline:c->W; c->vfmt=fmt.fmt.pix.pixelformat;
	if(c->vfmt!=V4L2_PIX_FMT_YUV420 && c->vfmt!=V4L2_PIX_FMT_SBGGR8){ fprintf(stderr,"미지원 포맷 0x%08x\n",c->vfmt); close(c->fd); c->fd=-1; return -1; }
	struct v4l2_requestbuffers req; memset(&req,0,sizeof req); req.count=4; req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory=V4L2_MEMORY_MMAP;
	if(xioctl(c->fd,VIDIOC_REQBUFS,&req)<0||req.count<2){ perror("REQBUFS"); close(c->fd); c->fd=-1; return -1; }
	c->nbuf=req.count; c->bufs=calloc(req.count,sizeof *c->bufs);
	for(unsigned i=0;i<req.count;i++){ struct v4l2_buffer b; memset(&b,0,sizeof b); b.type=req.type; b.memory=V4L2_MEMORY_MMAP; b.index=i;
		if(xioctl(c->fd,VIDIOC_QUERYBUF,&b)<0){ perror("QUERYBUF"); } c->bufs[i].len=b.length;
		c->bufs[i].start=mmap(NULL,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,c->fd,b.m.offset);
		struct v4l2_buffer qb; memset(&qb,0,sizeof qb); qb.type=req.type; qb.memory=V4L2_MEMORY_MMAP; qb.index=i; xioctl(c->fd,VIDIOC_QBUF,&qb); }
	enum v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(xioctl(c->fd,VIDIOC_STREAMON,&t)<0){ perror("STREAMON"); cam_close(c); return -1; }
	return 0;
}

int main(int argc,char**argv){
	int c;
	while((c=getopt(argc,argv,"d:W:H:P:q:u:p:s:b:h"))!=-1){ switch(c){
		case 'd': snprintf(cfg_dev,sizeof cfg_dev,"%s",optarg); break;
		case 'W': cfg_W=atoi(optarg); break; case 'H': cfg_H=atoi(optarg); break;
		case 'P': cfg_port=atoi(optarg); break; case 'q': cfg_quality=atoi(optarg); break;
		case 'u': cfg_warmup_sec=atof(optarg); break;
		case 'p': if(strlen(optarg)>=4) cfg_fourcc=FOURCC(optarg[0],optarg[1],optarg[2],optarg[3]); break;
		case 's': cfg_secw=atoi(optarg); break;
		case 'b': { char*cl=strchr(optarg,':'); if(cl){*cl=0; cfg_mqtt_port=atoi(cl+1);} snprintf(cfg_mqtt_broker,sizeof cfg_mqtt_broker,"%s",optarg); } break;
		default: fprintf(stderr,"사용: %s [-d dev][-W w][-H h][-P port][-q jpegq][-u warmup][-p fourcc(YU12/BA81)][-s 섹션폭px][-b mqtt브로커host[:port]]\n",argv[0]); return c=='h'?0:1;
	} }
	signal(SIGINT,on_sig); signal(SIGTERM,on_sig); signal(SIGPIPE,SIG_IGN);

	if(setup_buffers(cfg_W,cfg_H)<0){ fprintf(stderr,"buffers\n"); return 1; }
	uint8_t *rgb=malloc((size_t)cfg_W*cfg_H*3);

	int srv=socket(AF_INET,SOCK_STREAM,0); int one=1; setsockopt(srv,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
	struct sockaddr_in sa; memset(&sa,0,sizeof sa); sa.sin_family=AF_INET; sa.sin_addr.s_addr=INADDR_ANY; sa.sin_port=htons(cfg_port);
	if(bind(srv,(struct sockaddr*)&sa,sizeof sa)<0){ perror("bind"); return 1; }
	if(listen(srv,4)<0){ perror("listen"); return 1; }
	printf("준비됨. http://<Pi-IP>:%d/  (포맷 0x%08x, JPEG q=%d, Ctrl-C 종료)\n",cfg_port,cfg_fourcc,cfg_quality); fflush(stdout);
	mqtt_start(cfg_mqtt_broker, cfg_mqtt_port, MQTT_TOPIC);

	struct cam cam;
	if(cam_open(&cam)<0){ fprintf(stderr,"카메라 열기 실패(/dev/video0)\n"); return 1; }
	fcntl(srv,F_SETFL,O_NONBLOCK);
	uint8_t *latest=NULL; unsigned long latest_n=0;
	enum { MAXCL=6 };
	int sclients[MAXCL]; for(int i=0;i<MAXCL;i++) sclients[i]=-1;
	struct pollfd pf[2]; pf[0].fd=srv; pf[0].events=POLLIN; pf[1].fd=cam.fd; pf[1].events=POLLIN;
	while(!g_stop){
		pf[0].revents=pf[1].revents=0;
		if(poll(pf,2,1000)<0){ if(errno==EINTR)continue; break; }

		if(pf[0].revents&POLLIN){
			int nfd=accept(srv,NULL,NULL);
			if(nfd>=0){
				struct timeval cto={.tv_sec=5,.tv_usec=0};
				setsockopt(nfd,SOL_SOCKET,SO_SNDTIMEO,&cto,sizeof cto);
				setsockopt(nfd,SOL_SOCKET,SO_RCVTIMEO,&cto,sizeof cto);
				int one2=1; setsockopt(nfd,IPPROTO_TCP,TCP_NODELAY,&one2,sizeof one2);
				char rq[1024]; ssize_t rn=read(nfd,rq,sizeof rq-1); if(rn>0)rq[rn]=0; else rq[0]=0;
				if(strstr(rq,"GET /snapshot")){
					if(latest) serve_jpeg(nfd,latest,latest_n);
					else { const char*e="HTTP/1.0 503 No Frame\r\nConnection: close\r\n\r\n"; write_all(nfd,e,strlen(e)); }
					close(nfd);
				} else if(strstr(rq,"GET /stream")){
					static const char h[]="HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace; boundary=" BND "\r\n\r\n";
					int put=0;
					if(write_all(nfd,h,sizeof h-1)==0)
						for(int i=0;i<MAXCL;i++) if(sclients[i]<0){ sclients[i]=nfd; put=1; break; }
					if(!put) close(nfd);
				} else { serve_html(nfd); close(nfd); }
			}
		}

		if(pf[1].revents&POLLIN){
			struct v4l2_buffer b; memset(&b,0,sizeof b); b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
			if(xioctl(cam.fd,VIDIOC_DQBUF,&b)==0){
				detect_step(cam.bufs[b.index].start, cam.stride);
				if(cam.vfmt==V4L2_PIX_FMT_SBGGR8) bayer_to_rgb(cam.bufs[b.index].start, cam.stride, cam.W, cam.H, rgb);
				else                              yuv420_to_rgb(cam.bufs[b.index].start, cam.stride, cam.W, cam.H, rgb);
				annotate(rgb,cam.W,cam.H);
				uint8_t *jpg=NULL; unsigned long jn=0;
				tinyjpeg_encode_rgb(rgb,cam.W,cam.H,cfg_quality,&jpg,&jn);
				xioctl(cam.fd,VIDIOC_QBUF,&b);
				if(jpg&&jn){
					free(latest); latest=jpg; latest_n=jn;
					char part[160]; int pn=snprintf(part,sizeof part,"--" BND "\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n",jn);
					for(int i=0;i<MAXCL;i++) if(sclients[i]>=0){
						int ok=(write_all(sclients[i],part,pn)==0)&&(write_all(sclients[i],latest,jn)==0)&&(write_all(sclients[i],"\r\n",2)==0);
						if(!ok){ close(sclients[i]); sclients[i]=-1; }
					}
				}
			}
		}
	}
	for(int i=0;i<MAXCL;i++) if(sclients[i]>=0) close(sclients[i]);
	mqtt_stop();
	free(latest); cam_close(&cam); close(srv); free(rgb);
	return 0;
}
