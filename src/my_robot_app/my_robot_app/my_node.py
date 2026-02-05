
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyFirstNode(Node):
    def __init__(self):
        # ตั้งชื่อ Node ว่า 'mac_m4_publisher'
        super().__init__('mac_m4_publisher')
        
        # สร้าง Publisher ส่งข้อมูลประเภท String ไปที่ Topic ชื่อ '/robot_status'
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)
        
        # ตั้งเวลาให้ทำงานทุกๆ 1 วินาที (Timer)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'M4 Robot is running! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
