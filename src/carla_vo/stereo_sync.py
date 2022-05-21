import rclpy
from rclpy.node import Node

# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# import cv2
import math

# from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo  # , Image, PointCloud2, PointField


class StereoSync(Node):
    def __init__(self):
        super().__init__("stereo_synchronizer")
        self.sub_info_right = self.create_subscription(
            CameraInfo,
            "carla/ego_vehicle/rgb_right/camera_info",
            self.listener_right_info_callback,
            10,
        )
        self.sub_info_right  # prevent unused variable warning
        self.pub_info_right_ = self.create_publisher(
            CameraInfo, "/right/camera_info", 10
        )
        # setup timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        from rcl_interfaces.msg import ParameterDescriptor

        baseline_parameter_descriptor = ParameterDescriptor(
            description="Distance between stereo camera pair"
        )

        # declare baseline parameter
        self.declare_parameter("baseline", 0.5, baseline_parameter_descriptor)
        self.baseline = (
            self.get_parameter("baseline").get_parameter_value().double_value
        )

        self.camInfo_right = CameraInfo()

        # self.camInfo_right.header.stamp = now
        # msg.header.stamp = node.get_clock().now().to_msg()

    def timer_callback(self):
        self.pub_info_right_.publish(self.camInfo_right)

    def listener_right_info_callback(self, info_msg):
        # self.pub_info_right_.publish(info_msg)
        self.camInfo_right = info_msg
        cx = info_msg.width / 2.0
        cx = info_msg.width / 2.0
        cy = info_msg.height / 2.0
        # fx = info_msg.width / (2.0 * math.tan(float(90.0) * math.pi / 360.0))
        fx = info_msg.width / 2.0
        fy = fx
        # baseline
        Ty = 0.0  # left camera is center of stereo cam calculations
        # B = 1.0 # y left - y right from /carla-ros-bridge/src/ros-bridge/carla_spawn_objects/config/objects.json
        B = self.baseline
        Tx = -fx * B  # where B is the baseline between the cameras
        # Tx = -B #where B is the baseline between the cameras
        self.camInfo_right.p = [fx, 0.0, cx, Tx, 0.0, fy, cy, Ty, 0.0, 0.0, 1.0, 0.0]
        # self.pub_info_right_.publish(self.camInfo_right)


def main(args=None):
    rclpy.init(args=args)

    stereo_sync = StereoSync()

    rclpy.spin(stereo_sync)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stereo_sync.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
