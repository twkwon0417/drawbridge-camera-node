#ifndef TINY_JPEG_H
#define TINY_JPEG_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ── 표준 양자화 테이블 (Annex K.1 / K.2, natural order) ── */
static const int tj_qy[64] = {
	16,11,10,16,24,40,51,61, 12,12,14,19,26,58,60,55,
	14,13,16,24,40,57,69,56, 14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77, 24,35,55,64,81,104,113,92,
	49,64,78,87,103,121,120,101, 72,92,95,98,112,100,103,99
};
static const int tj_qc[64] = {
	17,18,24,47,99,99,99,99, 18,21,26,66,99,99,99,99,
	24,26,56,99,99,99,99,99, 47,66,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99
};

/* zigzag 순서 */
static const int tj_zig[64] = {
	0,1,8,16,9,2,3,10, 17,24,32,25,18,11,4,5,
	12,19,26,33,40,48,41,34, 27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36, 29,22,15,23,30,37,44,51,
	58,59,52,45,38,31,39,46, 53,60,61,54,47,55,62,63
};

/* ── 표준 허프만 테이블 (Annex K.3): bits[0..16], vals[] ── */
static const uint8_t tj_dc_lum_bits[17] = {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const uint8_t tj_dc_lum_val[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t tj_dc_chr_bits[17] = {0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const uint8_t tj_dc_chr_val[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t tj_ac_lum_bits[17] = {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const uint8_t tj_ac_lum_val[162] = {
	0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
	0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
	0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
	0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
	0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
	0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
	0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
	0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
	0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
	0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
	0xf9,0xfa
};
static const uint8_t tj_ac_chr_bits[17] = {0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const uint8_t tj_ac_chr_val[162] = {
	0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
	0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
	0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
	0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
	0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
	0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
	0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
	0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
	0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
	0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
	0xf9,0xfa
};

/* 허프만 코드/길이 테이블 (심볼 → {code,size}) 을 bits/vals 에서 생성 */
struct tj_huff { uint16_t code[256]; uint8_t size[256]; };
static void tj_build_huff(struct tj_huff *h, const uint8_t *bits, const uint8_t *val)
{
	uint16_t code = 0;
	int k = 0;
	memset(h, 0, sizeof(*h));
	for (int len = 1; len <= 16; len++) {
		for (int i = 0; i < bits[len]; i++) {
			h->code[val[k]] = code;
			h->size[val[k]] = (uint8_t)len;
			code++; k++;
		}
		code <<= 1;
	}
}

/* 동적 출력 버퍼 */
struct tj_buf { uint8_t *p; size_t n, cap; };
static void tj_put(struct tj_buf *b, int v)
{
	if (b->n >= b->cap) { b->cap = b->cap ? b->cap * 2 : 65536; b->p = realloc(b->p, b->cap); }
	b->p[b->n++] = (uint8_t)v;
}
static void tj_put2(struct tj_buf *b, int v) { tj_put(b, (v >> 8) & 0xff); tj_put(b, v & 0xff); }

/* 비트 라이터 (0xFF 바이트 스터핑) */
struct tj_bw { struct tj_buf *b; int acc, cnt; };
static void tj_wbits(struct tj_bw *w, int code, int size)
{
	w->acc |= (code & ((1 << size) - 1)) << (24 - w->cnt - size);
	w->cnt += size;
	while (w->cnt >= 8) {
		int c = (w->acc >> 16) & 0xff;
		tj_put(w->b, c);
		if (c == 0xff) tj_put(w->b, 0x00);   /* 스터핑 */
		w->acc = (w->acc << 8) & 0xffffff;
		w->cnt -= 8;
	}
}
static void tj_flushbits(struct tj_bw *w) { if (w->cnt > 0) tj_wbits(w, 0x7f, 7 - ((w->cnt - 1) & 7)); }

/* 8x8 분리형 DCT 기저: M[u][x] = 0.5*C(u)*cos((2x+1)u*pi/16) */
static void tj_dct_basis(double M[8][8])
{
	for (int u = 0; u < 8; u++)
		for (int x = 0; x < 8; x++)
			M[u][x] = 0.5 * (u == 0 ? 0.70710678118 : 1.0) *
				  cos((2 * x + 1) * u * 3.14159265358979 / 16.0);
}

/* 한 블록 처리: 공간(레벨시프트됨) → DCT → 양자화 → 허프만 인코드 */
static void tj_block(struct tj_bw *w, double M[8][8], const double in[64], const int *q,
		     int *dc_prev, const struct tj_huff *hdc, const struct tj_huff *hac)
{
	double t[64], F[64];
	/* 행 변환 */
	for (int y = 0; y < 8; y++)
		for (int u = 0; u < 8; u++) {
			double s = 0; for (int x = 0; x < 8; x++) s += M[u][x] * in[y*8+x];
			t[y*8+u] = s;
		}
	/* 열 변환 */
	for (int u = 0; u < 8; u++)
		for (int v = 0; v < 8; v++) {
			double s = 0; for (int y = 0; y < 8; y++) s += M[v][y] * t[y*8+u];
			F[v*8+u] = s;
		}
	/* 양자화 (natural order) */
	int Q[64];
	for (int i = 0; i < 64; i++) {
		double d = F[i] / q[i];
		Q[i] = (int)(d < 0 ? d - 0.5 : d + 0.5);
	}
	/* DC */
	int diff = Q[0] - *dc_prev; *dc_prev = Q[0];
	int adiff = diff < 0 ? -diff : diff, s = 0, vv = diff;
	while (adiff) { s++; adiff >>= 1; }
	if (diff < 0) vv = diff - 1;   /* 음수: (1<<s)-1 + diff */
	tj_wbits(w, hdc->code[s], hdc->size[s]);
	if (s) tj_wbits(w, vv & ((1 << s) - 1), s);
	/* AC (zigzag) */
	int run = 0;
	for (int k = 1; k < 64; k++) {
		int c = Q[tj_zig[k]];
		if (c == 0) { run++; continue; }
		while (run > 15) { tj_wbits(w, hac->code[0xf0], hac->size[0xf0]); run -= 16; }
		int a = c < 0 ? -c : c, sz = 0, val = c;
		while (a) { sz++; a >>= 1; }
		if (c < 0) val = c - 1;
		int sym = (run << 4) | sz;
		tj_wbits(w, hac->code[sym], hac->size[sym]);
		tj_wbits(w, val & ((1 << sz) - 1), sz);
		run = 0;
	}
	if (run > 0) tj_wbits(w, hac->code[0x00], hac->size[0x00]);  /* EOB */
}

static void tj_write_dqt(struct tj_buf *b, int id, const int *q)
{
	tj_put2(b, 0xffdb); tj_put2(b, 67); tj_put(b, id);
	for (int i = 0; i < 64; i++) tj_put(b, q[tj_zig[i]]);
}
static void tj_write_dht(struct tj_buf *b, int id, const uint8_t *bits, const uint8_t *val, int nval)
{
	int len = 19 + nval;
	tj_put2(b, 0xffc4); tj_put2(b, len); tj_put(b, id);
	for (int i = 1; i <= 16; i++) tj_put(b, bits[i]);
	for (int i = 0; i < nval; i++) tj_put(b, val[i]);
}

/* RGB888 → 베이스라인 JPEG(4:4:4). 성공 시 0, *out=malloc 버퍼(호출자 free) */
static int tinyjpeg_encode_rgb(const uint8_t *rgb, int W, int H, int quality,
			       uint8_t **out, unsigned long *outlen)
{
	if (quality < 1) quality = 1; if (quality > 100) quality = 100;
	int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
	int qy[64], qc[64];
	for (int i = 0; i < 64; i++) {
		int a = (tj_qy[i] * scale + 50) / 100; qy[i] = a < 1 ? 1 : a > 255 ? 255 : a;
		int c = (tj_qc[i] * scale + 50) / 100; qc[i] = c < 1 ? 1 : c > 255 ? 255 : c;
	}
	double M[8][8]; tj_dct_basis(M);
	struct tj_huff hYdc, hYac, hCdc, hCac;
	tj_build_huff(&hYdc, tj_dc_lum_bits, tj_dc_lum_val);
	tj_build_huff(&hYac, tj_ac_lum_bits, tj_ac_lum_val);
	tj_build_huff(&hCdc, tj_dc_chr_bits, tj_dc_chr_val);
	tj_build_huff(&hCac, tj_ac_chr_bits, tj_ac_chr_val);

	struct tj_buf b = {0};
	tj_put2(&b, 0xffd8);                                  /* SOI */
	tj_put2(&b, 0xffe0); tj_put2(&b, 16);                 /* APP0/JFIF */
	tj_put(&b,'J');tj_put(&b,'F');tj_put(&b,'I');tj_put(&b,'F');tj_put(&b,0);
	tj_put(&b,1);tj_put(&b,1);tj_put(&b,0);tj_put2(&b,1);tj_put2(&b,1);tj_put(&b,0);tj_put(&b,0);
	tj_write_dqt(&b, 0, qy);
	tj_write_dqt(&b, 1, qc);
	/* SOF0: 3 comp, 모두 1x1 샘플링(4:4:4) */
	tj_put2(&b, 0xffc0); tj_put2(&b, 17); tj_put(&b, 8);
	tj_put2(&b, H); tj_put2(&b, W); tj_put(&b, 3);
	tj_put(&b,1);tj_put(&b,0x11);tj_put(&b,0);   /* Y  q0 */
	tj_put(&b,2);tj_put(&b,0x11);tj_put(&b,1);   /* Cb q1 */
	tj_put(&b,3);tj_put(&b,0x11);tj_put(&b,1);   /* Cr q1 */
	tj_write_dht(&b, 0x00, tj_dc_lum_bits, tj_dc_lum_val, 12);
	tj_write_dht(&b, 0x10, tj_ac_lum_bits, tj_ac_lum_val, 162);
	tj_write_dht(&b, 0x01, tj_dc_chr_bits, tj_dc_chr_val, 12);
	tj_write_dht(&b, 0x11, tj_ac_chr_bits, tj_ac_chr_val, 162);
	/* SOS */
	tj_put2(&b, 0xffda); tj_put2(&b, 12); tj_put(&b, 3);
	tj_put(&b,1);tj_put(&b,0x00); tj_put(&b,2);tj_put(&b,0x11); tj_put(&b,3);tj_put(&b,0x11);
	tj_put(&b,0);tj_put(&b,63);tj_put(&b,0);

	struct tj_bw w = { &b, 0, 0 };
	int dcY = 0, dcCb = 0, dcCr = 0;
	double by[64], bcb[64], bcr[64];
	for (int my = 0; my < H; my += 8) {
		for (int mx = 0; mx < W; mx += 8) {
			for (int yy = 0; yy < 8; yy++) {
				for (int xx = 0; xx < 8; xx++) {
					int px = mx + xx < W ? mx + xx : W - 1;
					int py = my + yy < H ? my + yy : H - 1;
					const uint8_t *p = rgb + (py * W + px) * 3;
					double R = p[0], G = p[1], B = p[2];
					by[yy*8+xx]  = ( 0.299*R + 0.587*G + 0.114*B) - 128.0;
					bcb[yy*8+xx] = (-0.168736*R - 0.331264*G + 0.5*B);
					bcr[yy*8+xx] = ( 0.5*R - 0.418688*G - 0.081312*B);
				}
			}
			tj_block(&w, M, by,  qy, &dcY,  &hYdc, &hYac);
			tj_block(&w, M, bcb, qc, &dcCb, &hCdc, &hCac);
			tj_block(&w, M, bcr, qc, &dcCr, &hCdc, &hCac);
		}
	}
	tj_flushbits(&w);
	tj_put2(&b, 0xffd9);                                  /* EOI */
	*out = b.p; *outlen = b.n;
	return 0;
}

#endif /* TINY_JPEG_H */
