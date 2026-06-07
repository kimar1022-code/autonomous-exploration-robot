#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import TransformBroadcaster
import math
import threading
import serial

# ===== 보정 파라미터 (나중에 실측으로 조정) =====
WHEEL_RADIUS = 0.0325        # 바퀴 반지름 (m) = 32.5mm
WHEEL_BASE   = 0.162         # 좌우 바퀴 간격 (m) = 162mm
COUNTS_PER_REV_L = 1971.5    # 왼쪽 바퀴 1회전당 엔코더 카운트 (실측: 15772/8)
COUNTS_PER_REV_R = 2196.3    # 오른쪽 바퀴 1회전당 엔코더 카운트 (실측: 13178/6)
MAX_WHEEL_SPEED = 0.5        # 바퀴 최대 선속도 (m/s) - PWM 255에 대응 (추정)
MIN_PWM = 100                # 모터가 실제로 도는 최소 PWM (이하는 안 돎 → 데드밴드로 보상)

SERIAL_PORT = '/dev/ttyAMA0'
BAUD = 115200

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')

        # EKF(robot_localization)를 쓰면 odom→base_link TF는 EKF가 발행한다.
        # publish_tf=False(기본)면 여기선 TF를 안 보내 충돌을 피한다. (mock/단독구동 시 True)
        self.declare_parameter('publish_tf', False)
        self.publish_tf = self.get_parameter('publish_tf').value

        # 시리얼 연결
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            self.get_logger().info(f'Serial opened: {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            raise

        # 구독/발행
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 위치 상태
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 엔코더 상태
        self.last_lcnt = None
        self.last_rcnt = None
        self.last_time = self.get_clock().now()

        # 시리얼 수신 스레드
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('Base Controller (real UART) started')

    def _deadband(self, pwm):
        # 0이 아닌데 |pwm| < MIN_PWM 이면 최소 구동값으로 끌어올린다 (안 그러면 모터가 안 돎).
        if pwm == 0:
            return 0
        if abs(pwm) < MIN_PWM:
            return MIN_PWM if pwm > 0 else -MIN_PWM
        return pwm

    # ===== cmd_vel → M,<left>,<right> 전송 =====
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x       # 전진 속도 (m/s)
        w = msg.angular.z      # 회전 속도 (rad/s)

        # 차동구동 역기구학: 좌우 바퀴 선속도
        v_left  = v - (w * WHEEL_BASE / 2.0)
        v_right = v + (w * WHEEL_BASE / 2.0)

        # 선속도(m/s) → PWM(-255~255)
        left_pwm  = int(max(-255, min(255, v_left  / MAX_WHEEL_SPEED * 255)))
        right_pwm = int(max(-255, min(255, v_right / MAX_WHEEL_SPEED * 255)))

        # 데드밴드 보상: 느린 명령(Nav2 등)도 최소 구동 PWM 으로 끌어올림
        left_pwm = self._deadband(left_pwm)
        right_pwm = self._deadband(right_pwm)

        cmd = f'M,{left_pwm},{right_pwm}\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # ===== 시리얼 수신 루프 (별도 스레드) =====
    def read_serial_loop(self):
        buf = b''
        while self.running:
            try:
                data = self.ser.read(64)
                if data:
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        self.parse_line(line.decode(errors='ignore').strip())
            except Exception:
                pass

    def parse_line(self, line):
        if not line:
            return
        parts = line.split(',')
        tag = parts[0]

        if tag == 'E' and len(parts) == 3:
            self.handle_encoder(int(parts[1]), int(parts[2]))
        elif tag == 'I' and len(parts) == 7:
            self.handle_imu([float(x) for x in parts[1:7]])
        elif tag == 'Mag' and len(parts) == 4:
            self.handle_mag([float(x) for x in parts[1:4]])

    # ===== 엔코더 → odom 계산 =====
    def handle_encoder(self, lcnt, rcnt):
        now = self.get_clock().now()

        if self.last_lcnt is None:
            self.last_lcnt = lcnt
            self.last_rcnt = rcnt
            self.last_time = now
            return

        dl = -(lcnt - self.last_lcnt)
        dr = rcnt - self.last_rcnt
        self.last_lcnt = lcnt
        self.last_rcnt = rcnt

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        # 카운트 → 이동거리(m)
        dist_l = (dl / COUNTS_PER_REV_L) * (2 * math.pi * WHEEL_RADIUS)
        dist_r = (dr / COUNTS_PER_REV_R) * (2 * math.pi * WHEEL_RADIUS)

        d_center = (dist_l + dist_r) / 2.0
        d_theta  = (dist_r - dist_l) / WHEEL_BASE

        # 위치 누적
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        self.publish_odom(now, d_center / dt, d_theta / dt)

    def publish_odom(self, now, vx, vth):
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        # EKF 신뢰도: 전진(vx)은 비교적 신뢰, 회전(vyaw)은 엔코더 슬립으로 부정확 → 크게(덜 신뢰)
        odom.twist.covariance[0] = 0.04     # vx 분산
        odom.twist.covariance[35] = 0.25    # vyaw 분산
        self.odom_pub.publish(odom)

        # odom→base_link TF: EKF 사용 시(publish_tf=False)엔 EKF가 발행하므로 생략
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = q_z
            t.transform.rotation.w = q_w
            self.tf_broadcaster.sendTransform(t)

    # ===== IMU =====
    def handle_imu(self, vals):
        ax, ay, az, gx, gy, gz = vals
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        # 가속도: g → m/s^2
        imu.linear_acceleration.x = ax * 9.81
        imu.linear_acceleration.y = ay * 9.81
        imu.linear_acceleration.z = az * 9.81
        # 자이로: deg/s → rad/s
        imu.angular_velocity.x = gx * math.pi / 180.0
        imu.angular_velocity.y = gy * math.pi / 180.0
        imu.angular_velocity.z = gz * math.pi / 180.0
        # EKF 신뢰도: 자이로 z(회전)는 정확 → 작게(신뢰). EKF는 imu0 에서 vyaw 만 사용.
        imu.angular_velocity_covariance[8] = 0.02
        self.imu_pub.publish(imu)

    # ===== 지자계 =====
    def handle_mag(self, vals):
        mx, my, mz = vals
        mag = MagneticField()
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = 'imu_link'
        # uT → Tesla
        mag.magnetic_field.x = mx * 1e-6
        mag.magnetic_field.y = my * 1e-6
        mag.magnetic_field.z = mz * 1e-6
        self.mag_pub.publish(mag)

    def destroy_node(self):
        self.running = False
        try:
            self.ser.write(b'M,0,0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
