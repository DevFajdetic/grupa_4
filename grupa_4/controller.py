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
        self.image_subscriber = self.create_subscription(Image, '/image', self.image_listener_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_threshold', 10)
        self.got_ball =  False
        self.found_ball = False
        self.counter = 0
        self.declare_parameter('Kp', 0.7)
        self.declare_parameter('f', 265.23)
        self.declare_parameter('u0', 160.5)
        self.color = [
            [2, True], #blue
            [1, True], #green
            [1, False] #red
        ]
    
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
        if circles is not None and self.got_ball is False:
            self.found_ball = True
            self.counter = 0
            for circle in circles[0]:
                if circle[2] > nearest_circle[2]:
                    nearest_circle = circle
            #self.get_logger().info(f'Teske boje: {image[int(nearest_circle[1]), int(nearest_circle[0])]}')
            self.rotate_robot(self.find_object_angle(nearest_circle[0]))
        elif self.found_ball is False:
            #Tražim
            rotate_velocity = Twist()
            self.get_logger().info('Tražim šefe')
            rotate_velocity.angular = Vector3(x=0.0, y=0.0, z=0.15)
            rotate_velocity.linear = Vector3(x=0.0, y=0.0, z=0.0)
            self.velocity_publisher.publish(rotate_velocity)
        elif circles is None:
            if self.counter == 6:
                self.got_ball = True
                self.get_logger().info('Tražim cilj šefe')
                rotate_velocity = Twist()
                if self.find_goal(image, 10, self.color[0]) is False:
                    self.get_logger().info('Nisam pronašao!')
                    rotate_velocity.angular = Vector3(x=0.0, y=0.0, z=0.15)
                    rotate_velocity.linear = Vector3(x=0.0, y=0.0, z=0.0)
                else:
                    self.get_logger().info('Pronađoh!')
                    rotate_velocity.angular = Vector3(x=0.0, y=0.0, z=0.0)
                    rotate_velocity.linear = Vector3(x=0.1, y=0.0, z=0.0)
                self.velocity_publisher.publish(rotate_velocity)
            else:
                self.counter +=1
                self.get_logger().info('Ne vidim šefe')
                stop_velocity = Twist()            
                stop_velocity.angular = Vector3(x=0.0, y=0.0, z=0.0)
                stop_velocity.linear = Vector3(x=0.0, y=0.0, z=0.0)
                self.velocity_publisher.publish(stop_velocity)

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
        if not 0 <= abs(goal_angle) < 0.3:
            self.get_logger().info(f'Čekaj da se rotiram')           
            command.angular = Vector3(x=0.0, y=0.0, z=Kp*goal_angle/2)
        else:
            self.get_logger().info(f'Juriš! U boj!')
            command.linear = Vector3(x=0.1, y=0.0, z=0.0)
        
        self.velocity_publisher.publish(command)
    
    def find_goal(self, img, strip_width, color):
        img = img[0:img.shape[0], int(img.shape[1]/2 - strip_width):int(img.shape[1]/2 + strip_width)]
        lab_channel = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)[:,:,color[0]]

        if color[1] is False:
            threshold_type = cv2.THRESH_BINARY_INV
        else:
            threshold_type = cv2.THRESH_BINARY

        thresholded_img = cv2.threshold(lab_channel, 127, 255, threshold_type)[1]
        img = cv2.bitwise_and(img, img, mask = thresholded_img)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        self.image_publisher.publish(CvBridge().cv2_to_imgmsg(img, encoding='bgr8'))

        zero_array = np.zeros((img.shape[1],3))
        for i in range(img.shape[0]):
            if np.array_equal(img[i], zero_array):
                return True

        return False




def main(args=None):
    rclpy.init(args=args)    

    blob_tracker = BallDeliverer()    
    rclpy.spin(blob_tracker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
