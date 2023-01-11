import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2

class BallDeliverer(Node):

    def __init__(self):
        super().__init__('ball_deliverer')
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_listener_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.image_publisher = self.create_publisher(Image, 'camera/image_threshold', 10)

        self.declare_parameter('Kp', 2)
        self.declare_parameter('f', 265.23)
        self.declare_parameter('u0', 160.5)
    
    def image_listener_callback(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg)
        imGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(imGray, 25) 
        minDist = 100
        param1 = 30 
        param2 = 25
        minRadius = 5
        maxRadius = 250
        
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        #image_msg = CvBridge().cv2_to_imgmsg(blurred)
        nearest_circle = [0, 0, 0]

        #self.image_publisher.publish(image_msg)
        if circles is not None:
            for circle in circles[0]:
                if circle[2] > nearest_circle[2]:
                    nearest_circle = circle

            self.rotate_robot(self.find_object_angle(nearest_circle[0]))
    

    def find_object_angle(self, x_coordinate):
        u0 = float(self.get_parameter('u0').value)
        f = float(self.get_parameter('f').value)
        return np.arctan((u0 - x_coordinate)/f)
    
    def rotate_robot(self, goal_angle):
        Kp = float(self.get_parameter('Kp').value)

        if goal_angle > np.pi:
            goal_angle -= 2*np.pi
        if goal_angle <= -np.pi:
            goal_angle += 2*np.pi

        command = Twist()
        if not 0 <= abs(goal_angle) < 0.01:            
            command.angular = Vector3(x=0.0, y=0.0, z=Kp*goal_angle)
        else:
            command.angular = Vector3(x=0.0, y=0.0, z=0.0)
            #self.move_robot_forward()
        
        self.velocity_publisher.publish(command)
    
    #def move_robot_forward(self):



def main(args=None):
    rclpy.init(args=args)    

    blob_tracker = BallDeliverer()    
    rclpy.spin(blob_tracker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()