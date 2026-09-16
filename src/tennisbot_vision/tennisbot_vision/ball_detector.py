import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__("ball_detector")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data)

        self.lower1 = np.array([31, 65 , 86])
        self.upper1 = np.array([52, 255, 255])

        self.lower2 = np.array([3, 198, 104])
        self.upper2 = np.array([15, 255, 255])

        self.kernel = np.ones((11, 11), np.uint8)
        self.get_logger().info("Ball detector started !")

    def image_callback(self, msg):
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
        cv2.imshow("Contour", contour_image)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            perimeter = cv2.arcLength(largest_contour, True)

            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
            else:
                circularity = 0
            
            if area > 100 and circularity > 0.60:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)

                center = (int(x), int(y))
                radius = int(radius)
                diameter_pixel = radius * 2

                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                cv2.putText(frame, f"Center: {center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Diameter: {diameter_pixel} px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Circularity: {circularity:.2f} ", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Webcam", frame)
        # cv2.imshow("Mask", mask)

        cv2.waitKey(1)
    

def main(args=None): 
    rclpy.init(args=args) 
    node = BallDetectorNode()  
    rclpy.spin(node)    
    rclpy.shutdown() 

if __name__ == "__main__":  
    main()