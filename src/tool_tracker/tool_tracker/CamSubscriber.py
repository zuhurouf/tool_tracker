import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data



class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("CameraSubscriber")
        self.cam_subscriber = self.create_subscription(
            msg_type=Image,
            topic="camera/image_raw",
            qos_profile=qos_profile_sensor_data,
            callback=self.handler
        )
        self.cv_bridge = CvBridge()
        self.get_logger().info("CameraSubscriber:: Node instantiated")


    def handler(self, msg):
        try:
            current_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Live camera", current_frame)
            cv2.waitKey(1)
        except Exception as err:
            self.get_logger().error("CameraSubscriber::Failed in processing the image data")
            self.get_logger().error(f"CameraSubscriber::{err}")



def main(args = None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node=node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
