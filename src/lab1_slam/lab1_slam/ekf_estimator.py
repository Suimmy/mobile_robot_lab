import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import math

class EKFSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # ค่านามธรรมของหุ่นยนต์ (เช็คจากใบงานอีกทีนะครับ)
        self.R = 0.033
        self.L = 0.160
        
        # สถานะเริ่มต้นของหุ่นยนต์
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # สร้าง Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # สร้าง Publisher และ TF Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        #บันทึกตัวเลขลงไฟล์ CSV (เพื่อเอาไปทำรายงาน)
        self.csv_file = open('ekf_data.csv', 'w')
        self.csv_file.write('timestamp,x,y,theta\n')

    def joint_callback(self, msg):
        # 1. คำนวณเวลาที่ผ่านไป (dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 2. ดึงความเร็วล้อ (ตรวจสอบ index ของล้อซ้าย/ขวาใน msg.velocity)
        # สมมติ index 0 คือซ้าย, 1 คือขวา
        v_left = msg.velocity[0] * self.R
        v_right = msg.velocity[1] * self.R

        v = (v_right + v_left) / 2.0
        # ใน Part 1 นี้ เราจะยังไม่ใช้ความเร็วการหมุนจากล้อ แต่จะรอใช้จาก IMU ในขั้นตอนถัดไป
        
        # 3. อัปเดตตำแหน่ง (Basic Dead Reckoning)
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        
        self.publish_odom(current_time)

    def imu_callback(self, msg):
        # รับค่าความเร็วการหมุน (Angular Velocity) รอบแกน Z
        self.th += msg.angular_velocity.z * 0.05 # สมมติ dt ของ IMU คือ 20Hz (0.05s)
        # ในขั้นสูง เราจะใช้ EKF เพื่อฟิวชั่นค่านี้ แต่ตอนนี้เอาแบบ Simple ก่อนครับ

    def publish_odom(self, current_time):
        # ส่งข้อมูล Odometry และประกาศ TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        # แปลงมุม Euler เป็น Quaternion (ตัดทอนเพื่อความกระชับ)
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Estimated Pose: x={self.x:.2f}, y={self.y:.2f}, th={self.th:.2f}')
        # บันทึกตัวเลขลงไฟล์ทุกครั้งที่มีการคำนวณ
        ts = current_time.to_msg().sec + current_time.to_msg().nanosec * 1e-9
        self.csv_file.write(f'{ts},{self.x},{self.y},{self.th}\n')


def main(args=None):
    rclpy.init(args=args)
    node = EKFSlamNode()
    rclpy.spin(node)
    rclpy.shutdown()