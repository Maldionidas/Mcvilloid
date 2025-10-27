import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MiRobot(Node):
    def __init__(self):
        super().__init__('mi_robot_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hola desde mi robot en Webots 🤖'
        self.publisher.publish(msg)
        self.get_logger().info(f'Enviado: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MiRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
