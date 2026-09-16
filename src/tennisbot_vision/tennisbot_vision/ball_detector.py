import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_srvs.srv import Empty

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__("ball_detector")
        self.ball_diameter = 0.067
        self.fx = None

        #Image recognition
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.info_callback, qos_profile_sensor_data)
        
        self.lower1 = np.array([31, 65 , 86])
        self.upper1 = np.array([52, 255, 255])
        self.lower2 = np.array([3, 198, 104])
        self.upper2 = np.array([15, 255, 255])
        self.kernel = np.ones((11, 11), np.uint8)

        #Cancel Patroling 
        self.cancel_patrol = self.create_client(Empty, "ball_detection")
        self.isball_counter = 0
        self.cancel_request = False

        #Locate tennis ball
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Go to ball
        self.gotoball_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")  
        self.goal_handle_ = None
        self.nav_ongoing = False
        self.ball_approach_done = False

        #Gotoball button
        self.enable_gotoball_sub = self.create_subscription(Joy, "joy", self.enable_gotoball_callback, 10)
        self.enable_gotoball_button = 7
        self.enable_gotoball = False
        self.previous_gotoball_button = False
        
        self.get_logger().info("Ball detector started !")

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera info: fx={self.fx:.2f}, fy={self.fy:.2f}, " 
                               f"cx={self.cx:.2f}, cy={self.cy:.2f}")
        self.destroy_subscription(self.info_sub)
        self.info_sub = None

    def image_callback(self, msg: Image):
        if self.fx is None:
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # image_raw to CV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        #Close
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = frame.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
        # cv2.imshow("Contour", contour_image)
        if self.enable_gotoball:
            cv2.putText(frame, "Enable find ball", (370, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, "Disable find ball", (370, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            perimeter = cv2.arcLength(largest_contour, True)

            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
            else:
                circularity = 0                
            
            if area > 100 and circularity > 0.60 and self.enable_gotoball:
                #Locate tennis ball
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)

                center = (int(x), int(y))
                radius = int(radius)
                diameter_pixel = radius * 2

                #Calculate distance
                distance = self.fx * self.ball_diameter / diameter_pixel
                d_X = (x - self.cx) * distance / self.fx
                d_Y = (y - self.cy) *distance /self.fy

                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                cv2.putText(frame, f"Center: {center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Diameter: {diameter_pixel} px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Circularity: {circularity:.2f} ", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {distance:.2f} ", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"X: {d_X:.2f}, Y: {d_Y:.2f} ", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                #Cancle patroling
                self.isball_counter += 1
                if self.isball_counter >= 3 and not self.cancel_request:
                    request = Empty.Request()
                    self.cancel_patrol.call_async(request)
                    self.cancel_request = True

                #Locate tennis ball 
                if self.cancel_request is True and not self.nav_ongoing and not self.ball_approach_done:
                    
                    ball_camera = PointStamped()
                    ball_camera.header.frame_id = "camera_optical_link"
                    ball_camera.header.stamp = msg.header.stamp
                    ball_camera.point.x = d_X
                    ball_camera.point.y = d_Y
                    ball_camera.point.z = distance

                    try:
                        transform = self.tf_buffer.lookup_transform("map", "camera_optical_link", Time.from_msg(msg.header.stamp))   #Find transform rule
                        ball_map = do_transform_point(ball_camera, transform)  #Do transform
                        self.get_logger().info(
                            f"Ball map: "
                            f"x={ball_map.point.x:.2f}, "
                            f"y={ball_map.point.y:.2f}"
                        )
                        goal = NavigateToPose.Goal()
                        goal.pose.header.frame_id = "map"
                        goal.pose.header.stamp = self.get_clock().now().to_msg() 
                        goal.pose.pose.position.x = ball_map.point.x
                        goal.pose.pose.position.y = ball_map.point.y
                        goal.pose.pose.orientation.w = 1.0

                        send_goal_future = self.gotoball_client.send_goal_async(goal)
                        send_goal_future.add_done_callback(self.goal_response_callback)
                        self.nav_ongoing = True

                    except Exception as e:
                        self.get_logger().warn(f"TF failed: {e}")

                if distance < 0.3 and self.goal_handle_ is not None and not self.ball_approach_done:
                    self.get_logger().warn("reach < 0.3m")
                    self.ball_approach_done = True
                    self.goal_handle_.cancel_goal_async()

            else:
                self.isball_counter = 0
                self.cancel_request = False
        else:
            self.isball_counter = 0
            self.cancel_request = False

        cv2.imshow("Webcam", frame)
        # cv2.imshow("Mask", mask)

        cv2.waitKey(1)

    ### For NavigateToPose Action
    def goal_response_callback(self, future):
        self.goal_handle_ = future.result()
        if not self.goal_handle_.accepted:
            self.get_logger().warn("Goal rejected")
            self.nav_ongoing = False
            return
        self.get_logger().info("GoToBall goal accepted")
        self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.nav_ongoing = False
        self.goal_handle_ = None
        self.get_logger().info("GoToBall finished")

    #Enable gotoball button
    def enable_gotoball_callback(self, msg: Joy):
        if msg.buttons[self.enable_gotoball_button] == 1 and not self.previous_gotoball_button:
            if self.enable_gotoball is False:
                self.get_logger().warn("Enable gotoball function")
                self.enable_gotoball = True
            else:
                self.get_logger().warn("Disable gotoball function")
                self.enable_gotoball = False
                #reset
                if self.goal_handle_ is not None:
                    self.goal_handle_.cancel_goal_async()
                self.isball_counter = 0
                self.cancel_request = False
                self.ball_approach_done = False
        self.previous_gotoball_button = msg.buttons[self.enable_gotoball_button] == 1

def main(args=None): 
    rclpy.init(args=args) 
    node = BallDetectorNode()  
    rclpy.spin(node)    
    rclpy.shutdown() 

if __name__ == "__main__":  
    main()