import numpy as np
# --- NumPy 1.24+ Compatibility Patch for Older Libraries ---
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'bool'):
    np.bool = bool
# ------------------------------------------------------------
import cv2
import rclpy
from rclpy.node import Node
import tf_transformations as tf
from geometry_msgs.msg import Transform
from rclpy.qos import qos_profile_sensor_data
from apriltag_msgs.msg import AprilTagDetection



class PoseEstimator(Node):
    ## Focal length
    ## fx = width/2.tan(HFOV/2); fx = fy
    Fx, Fy = 421.84, 421.84
    ## Optical center
    ## img_height/2, img_weight/2
    Cx, Cy = 320, 240

    TAG_SIZE = 0.06
    HALF_SIZE = TAG_SIZE/2
    def __init__(self):
        super().__init__("PoseEstimator")
        self.coordinates_subscriber = self.create_subscription(
            msg_type=AprilTagDetection,
            topic="tag_coordinates",
            qos_profile=qos_profile_sensor_data,
            callback=self.handler
        )
        self.pose_publisher = self.create_publisher(
            msg_type=Transform,
            topic="tag_pose",
            qos_profile=qos_profile_sensor_data
        )
        self.get_logger().info("Pose estimator instantiated")


    def handler(self, msg):
        camera_matrix = np.array([
            [PoseEstimator.Fx, 0, PoseEstimator.Cx],
            [0, PoseEstimator.Fy, PoseEstimator.Cy],
            [0, 0, 1]
        ], dtype=np.float32)
        distortion_coeffs = np.zeros((4, 1), dtype=np.float32)
        object_points = np.array([
            [-PoseEstimator.HALF_SIZE, -PoseEstimator.HALF_SIZE, 0.0],
            [PoseEstimator.HALF_SIZE, -PoseEstimator.HALF_SIZE, 0.0],
            [PoseEstimator.HALF_SIZE, PoseEstimator.HALF_SIZE, 0.0],
            [-PoseEstimator.HALF_SIZE, PoseEstimator.HALF_SIZE, 0.0],
        ])
        img_points = np.array([
            [corner.x, corner.y] for corner in msg.corners
        ], dtype=np.float32)

        success, rot, transl = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=img_points,
            cameraMatrix=camera_matrix,
            distCoeffs=distortion_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if success:
            msg = Transform()
            msg.translation.x = float(transl[0])
            msg.translation.y = float(transl[1])
            msg.translation.z = float(transl[2])
            q = tf.quaternion_from_euler(rot[0], rot[1], rot[2])
            msg.rotation.x = float(q[0])
            msg.rotation.y = float(q[1])
            msg.rotation.z = float(q[2])
            msg.rotation.w = float(q[3])
            self.pose_publisher.publish(msg)
        else:
            self.get_logger().warning("Estimation didn't work")




def main(args=None):
    rclpy.init(args=args)
    estimator = PoseEstimator()
    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        pass
    finally:
        estimator.destroy_node()
        rclpy.shutdown()