import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SimplePubSub(Node):
    def __init__(self, namespace=''):
        super().__init__('simple_pub_sub', namespace=namespace)
        self.publisher = self.create_publisher(Image, f'/{namespace}/depth/image_processed', 10)
        self.subscription = self.create_subscription(
            Image, f'/{namespace}/depth/image_raw', self.img_callback, 10)
        self.bridge = CvBridge()

    def img_callback(self, msg):
        #self.get_logger().info('processing image....')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Convert 16-bit depth image to 8-bit
        depth_8u = cv2.convertScaleAbs(cv_image, alpha=(255.0/65535.0))
        
        # Convert OpenCV image back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(depth_8u, encoding='8UC1  ')
        
        self.publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    namespace = 'turtlebot'
    simple_pub_sub = SimplePubSub(namespace=namespace)
    rclpy.spin(simple_pub_sub)
    simple_pub_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()