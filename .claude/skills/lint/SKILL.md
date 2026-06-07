---
name: lint
description: autonomous_robot ROS2 패키지의 ament 린터(flake8/pep257)를 실행해 코드 스타일을 검사한다. 커밋 전이나 파이썬 노드를 수정한 뒤 사용한다.
---

# 코드 린트 (ament)

이 저장소의 유일한 자동 검사는 ROS2 기본 ament 린터다 (`test/` 안의 stub들이 이 린터를 호출한다). 실제 기능 단위테스트는 없다.

## 실행 방법

워크스페이스 루트에서:

```bash
cd ~/autonomous-exploration-robot/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select autonomous_robot --event-handlers console_direct+
colcon test-result --verbose          # 실패 항목 상세 보기
```

`colcon`/ROS 환경이 없으면 (예: SLAM 데스크탑이 아닌 곳) flake8 단독으로 빠르게 검사:

```bash
flake8 ros2_ws/src/autonomous_robot/autonomous_robot --max-line-length 99
```

## 처리 지침

- flake8 위반은 직접 고친다. 단 **줄 길이 등 기능과 무관한 사소한 경고로 동작 코드를 바꾸지 말 것** — 의미 있는 위반만 수정한다.
- pep257(docstring) 경고는 참고용. 노드 동작에는 영향 없으므로 사용자가 원할 때만 손댄다.
- 검사 결과를 한국어로 요약해 보고한다.
