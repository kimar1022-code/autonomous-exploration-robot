#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select, threading

# ===== 속도 설정 =====
# 시작/증가 단위 (회전 가능한 수준부터 시작)
LINEAR_STEP  = 0.15    # w/x 누를 때마다 전진/후진 속도 증가량 (m/s)
ANGULAR_STEP = 1.5     # a/d 누를 때마다 회전 속도 증가량 (rad/s)

LINEAR_MAX   = 0.6     # 최대 전진/후진 속도
ANGULAR_MAX  = 4.0     # 최대 회전 속도

PUBLISH_RATE = 10.0    # 초당 명령 발행 횟수 (계속 이동 유지)

MSG = """
==================================
   로봇 키보드 조종 (누적 속도형)
==================================
   w : 전진 (누를수록 빨라짐)
   x : 후진 (누를수록 빨라짐)
   a : 좌회전 (누를수록 빨라짐)
   d : 우회전 (누를수록 빨라짐)
   s : 정지

   q : 종료
==================================
* s 누르기 전까지 계속 이동합니다
* 같은 키 여러 번 = 속도 단계 상승
==================================
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.0   # 현재 전진 속도 (음수=후진)
        self.ang = 0.0   # 현재 회전 속도 (양수=좌, 음수=우)
        self.lock = threading.Lock()
        # 주기적으로 현재 속도를 계속 발행 (멈추라기 전까지 이동 유지)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_cmd)

    def publish_cmd(self):
        with self.lock:
            msg = Twist()
            msg.linear.x = self.lin
            msg.angular.z = self.ang
            self.pub.publish(msg)

    def update(self, key):
        with self.lock:
            if key == 'w':
                self.lin = min(self.lin + LINEAR_STEP, LINEAR_MAX) if self.lin >= 0 else LINEAR_STEP
            elif key == 'x':
                self.lin = max(self.lin - LINEAR_STEP, -LINEAR_MAX) if self.lin <= 0 else -LINEAR_STEP
            elif key == 'a':
                self.ang = min(self.ang + ANGULAR_STEP, ANGULAR_MAX) if self.ang >= 0 else ANGULAR_STEP
            elif key == 'd':
                self.ang = max(self.ang - ANGULAR_STEP, -ANGULAR_MAX) if self.ang <= 0 else -ANGULAR_STEP
            elif key == 's':
                self.lin = 0.0
                self.ang = 0.0
            return self.lin, self.ang


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if r else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Teleop()

    # ROS 스핀을 별도 스레드에서 (타이머 발행용)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(MSG)
    try:
        while True:
            key = get_key(settings)
            if key == 'q':
                node.update('s')
                print('\n종료')
                break
            elif key in ('w', 'x', 'a', 'd', 's'):
                lin, ang = node.update(key)
                print(f'\r전진:{lin:+.2f} m/s   회전:{ang:+.2f} rad/s        ', end='', flush=True)
    except Exception as e:
        print(e)
    finally:
        node.update('s')
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
