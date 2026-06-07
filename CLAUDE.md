# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**대화·설명·코드 주석은 모두 한국어로 작성한다.**

## 프로젝트 개요

자작 4WD 차동구동 로봇을 ROS2 Jazzy에 통합하는 프로젝트. **분산 구조**로 동작한다:

- **Raspberry Pi 4** (로봇, hostname `aeriPC`, user `aeri`, IP `192.168.75.100` — DHCP라 공유기에 따라 바뀔 수 있음): 센서 수집·모터 제어. ROS2 `ros-base`. 노드: `base_controller`, `rplidar_node`, `camera_node`. 워크스페이스는 `~/ros2_ws`(데스크탑과 별도, git 아님).
- **데스크탑** (연산 기기, user `aeri`, GTX 1660 SUPER GPU): 무거운 연산. ROS2 `desktop`(rviz2 포함) + slam_toolbox + Nav2 + YOLO(CUDA). **이 작업 디렉터리(`/home/aeri`)가 데스크탑이다.**

Pi가 `/odom /scan /imu /tf`를 발행하면 데스크탑이 받아 SLAM/Nav을 돌리고, 데스크탑이 `/cmd_vel`을 Pi로 보낸다. 둘은 DDS로 통신한다.

현재 위치: SLAM(slam_toolbox) 직전. Phase 1~2(펌웨어·base_controller·teleop·RPLIDAR·분산통신)까지 완료.

## 빌드 / 실행

```bash
# 빌드 (워크스페이스 루트에서)
cd ~/autonomous-exploration-robot/ros2_ws
colcon build --symlink-install      # symlink: 파이썬 수정 시 재빌드 불필요
source install/setup.bash

# 노드 실행 (패키지명: autonomous_robot)
ros2 run autonomous_robot base_controller        # 메인: cmd_vel↔시리얼, odom/imu 발행
ros2 run autonomous_robot teleop_node            # 키보드 수동조종 (w/x/a/d/s)
ros2 run autonomous_robot camera_node            # Pi 카메라
ros2 run autonomous_robot base_controller_mock   # 하드웨어 없이 테스트

# 통합 실행 (robot_state_publisher + base_controller + camera_node)
ros2 launch autonomous_robot robot.launch.py

# 라이다 (별도 터미널, A1은 115200 — A3의 256000 아님)
sudo chmod 666 /dev/ttyUSB0
ros2 launch rplidar_ros rplidar.launch.py \
  serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=laser
```

## 분산 통신 설정 (Pi·데스크탑 양쪽 동일)

```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET   # ROS_LOCALHOST_ONLY 는 Jazzy에서 deprecated
```

## 반드시 지킬 규칙 (모르면 망가짐)

- **ROS2 의존 패키지는 apt로만 설치한다. pip으로 numpy/opencv 설치 절대 금지** — `cv_bridge`와 충돌함.
- **패키지 설치/수정 전 웹 검색으로 검증**한다 (호환성 이슈 이력 있음).
- **시리얼 프로토콜의 진실**: ROS 노드(`base_controller.py`)는 `M,<left>,<right>\n` 에 **PWM 값 -255~255** 를 보낸다. 일치하는 펌웨어는 **`firmware/base_controller_pwm.ino.ino`(현재 사용)**. `firmware/base_controller.ino`(방향 -1/0/1)는 **구버전이니 기준으로 삼지 말 것**.
- **아두이노 Nano 33 BLE Sense는 3.3V 보드 — 핀에 5V 신호 절대 금지** (5V tolerant 아님, 보드 손상). 엔코더도 3.3V로 구동.
- **UART는 `/dev/ttyAMA0`** (115200). `ttyS0`(mini UART)는 GPIO 미연결 — `/boot/firmware/config.txt`에 `dtoverlay=disable-bt` 필요. 권한은 `dialout` 그룹.
- **회전 시 한쪽 바퀴만 돌면** 회전 속도를 올려 양쪽 모두 최소 PWM(약 100)을 넘겨야 한다.
- **포트 점유 충돌**(ttyAMA0/ttyUSB0): `sudo lsof <port>` → `kill <PID>`.

## 오도메트리 보정값 (`base_controller.py`)

실측값이므로 함부로 바꾸지 말 것. **좌우가 다르고 왼쪽 엔코더는 부호 반전**되어 있다.

- `COUNTS_PER_REV_L = 1971.5`, `COUNTS_PER_REV_R = 2196.3` (좌우 다름)
- `WHEEL_RADIUS = 0.0325`, `WHEEL_BASE = 0.162`
- 왼쪽 엔코더 델타 부호 반전: `dl = -(lcnt - last_lcnt)` (배선 보정)

## 저장소 관례

- 솔로 프로젝트 — `main`에 직접 커밋. 커밋 메시지는 한국어/영어 혼용 가능.
- 빌드 산출물(`ros2_ws/install`, `build`, `log`)과 `__pycache__`는 `.gitignore` 처리됨.

## 진행 단계

다음 작업: ① URDF에 `laser` 프레임 추가 → `map→odom→base_link→laser` TF 정리 → ② 데스크탑에서 `slam_toolbox` online_async로 맵핑 → ③ Nav2 → ④ YOLO(CUDA) → ⑤ Unity UI.
