import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class EndEffectorTracker(Node):
    def __init__(self):
        super().__init__('end_effector_tracker')
        
        # Gazebo Fortress topic bridged to ROS 2
        self.subscription = self.create_subscription(
            Image, 
            '/zed2i/camera/image', 
            self.image_callback, 
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Vision Tracking Node started. Awaiting frames...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- Insert tracking logic here (OpenCV, HSV threshold, MediaPipe, etc.) ---

            cv2.imshow("ZED 2i Gazebo Feed", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()