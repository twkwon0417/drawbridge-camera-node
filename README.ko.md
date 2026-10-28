# Drawbridge 감시 시스템 (커스텀 OV5647 드라이버 버전)

펌웨어 없이 커스텀 V4L2 센서 드라이버로 OV5647 카메라를 구동하고,
영상을 세로 3구역으로 나눠 변화를 검출하여 MQTT로 전송한다.
수동 조작 스위치 2개도 커널 드라이버로 받아 MQTT로 전송한다.

대상: Raspberry Pi 3B / Raspbian Buster / 커널 4.19.y

## 폴더 구성
- `driver/`   커널 드라이버: 카메라(`ov5647_skel`) + 수동 스위치(`manual_switch`)
- `overlay/`  DT overlay (cam0 25MHz + 연속 클럭 + CAM_GPIO0)
- `detector/` userspace: `bridge_view`(웹+검출), `bridge_detect`(헤드리스), `sw_bridge`(스위치→MQTT)
- `systemd/`  부팅 자동시작 서비스 4개

## 설치 위치
이 폴더 내용을 라즈베리파이의 `~/ku_star/` 에 둔다 (`/home/pi/ku_star`).

## 1) DT overlay 설치
```
dtc -@ -I dts -O dtb -o /tmp/ov5647-skel-clk.dtbo overlay/ov5647-skel-clk-overlay.dts
sudo cp /tmp/ov5647-skel-clk.dtbo /boot/overlays/ov5647-skel-clk.dtbo
```

## 2) /boot/config.txt 끝에 추가
```
start_x=0
dtoverlay=ov5647-skel-clk
gpio=5,12=ip,pd
```

## 3) 빌드
```
cd ~/ku_star/driver   && make
cd ~/ku_star/detector && make MQTT=1
```
(MQTT 사용 시 `libmosquitto-dev` 필요)

## 4) 설정값
`detector/config.h` 에서 변경:
- `MQTT_BROKER`  브로커(수신 측) IP
- `MQTT_TOPIC` / `MQTT_TOPIC_CTRL`  publish 토픽
- `SECTION_WIDTH`  구역 기준 폭, 검출 임계값 등

## 5) 자동시작 등록
```
sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ov5647-skel bridge-view manual-switch sw-bridge
```
재부팅 후 웹: `http://<Pi_IP>:8080/`

## MQTT payload
- 구역 변화: 3자리 문자열 (예 `110` = 좌·가운데 감지, 우 없음)
- 수동 스위치: `MANUAL_UP` / `MANUAL_DOWN`
