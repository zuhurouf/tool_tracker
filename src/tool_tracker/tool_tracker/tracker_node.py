import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



class TrackerNode(Node):
    def __init__(self):
        super().__init__("TrackerNode")
        self.subscrption = self.create_subscription(Image, "/zed2i/camera/image", self.img_handler, 10)
        self.cv_bridge = CvBridge()
        self.get_logger().info("Tracker node instantiated successfully")


    def img_handler(self, msg):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Live", cv_img)
            cv2.waitKey(1)
        except Exception as err:
            self.get_logger().info(f"Error::TrackerNode::Image hanler failed: {err}")



def main(args=None):
    rclpy.init(args=args)
    tracker = TrackerNode()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
