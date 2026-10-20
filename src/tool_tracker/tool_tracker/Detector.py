import cv2
import rclpy
import apriltag
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data



class Detector(Node):
    def __init__(self):
        super().__init__("AprilTagDetector")
        self.RADIUS = 5
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11")
        )
        self.cam_subscriber = self.create_subscription(
            msg_type=Image,
            topic="camera/image_raw",
            qos_profile=qos_profile_sensor_data,
            callback=self.handler
        )
        self.detection_publisher = self.create_publisher(
            Image,
            "camera/detections_image",
            qos_profile_sensor_data,
        )
        self.cv_bridge = CvBridge()
        self.get_logger().info("End-effector detector instantiated")


    def handler(self, msg):
        try:
            current_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            gray_img = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray_img)
            for r in results:
                corners = r.corners.astype(int)
                cv2.polylines(
                    current_frame,
                    [corners],
                    isClosed=True,
                    color=self.GREEN,
                    thickness=3,
                )
                label_position = tuple(corners[0])
                cv2.putText(
                    current_frame,
                    f"ID: {r.tag_id}",
                    label_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    self.GREEN,
                    2,
                )

            annotated_msg = self.cv_bridge.cv2_to_imgmsg(
                current_frame,
                encoding="bgr8",
            )
            annotated_msg.header = msg.header
            self.detection_publisher.publish(annotated_msg)

            cv2.imshow("Tool tracker", current_frame)
            cv2.waitKey(1)
        except Exception as err:
            self.get_logger().error("CameraSubscriber::Failed in processing the image data")
            self.get_logger().error(f"CameraSubscriber::{err}")



def main(args = None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
