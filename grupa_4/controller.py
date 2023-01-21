import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2


class BallDeliverer(Node):

    def __init__(self):
        super().__init__('ball_deliverer')
        qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.image_subscriber = self.create_subscription(Image, '/image', self.image_listener_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_threshold', 10)
        self.lidar_scanner = self.create_subscription(LaserScan,'/scan', self.scan_callback, qos_profile=qos_profile)
        self.got_ball =  False
        self.found_ball = False
        self.ball_delivered = False
        self.close_to_obstacle = False
        self.image_error_threshold = 0
        self.current_color = 0
        self.color = [
            [2, True], #blue
            [1, True], #green
            [1, False] #red
        ]
        self.distance_matrix = [4, 4, 4] #[front, left, right]
        self.declare_parameter('Kp', 0.7)
        self.declare_parameter('f', 265.23)
        self.declare_parameter('u0', 160.5)

    def image_listener_callback(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg)
        imGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.medianBlur(imGray, 25)

        if self.found_ball is False:
            self.find_ball(blurred_img)
        elif self.got_ball is False:
            self.goto_ball(blurred_img, image)
        elif self.ball_delivered is False:
            self.find_goal(image)
        elif self.distance_matrix[0] <= 0.25 and self.distance_matrix[0] != 0:
            self.move_robot('x')
        else:
            self.move_robot('s')

    
    def find_nearest_circle_from_image(self, blurrred_image):
        minDist = 100
        param1 = 30 
        param2 = 25
        minRadius = 5
        maxRadius = 250

        found_circles = cv2.HoughCircles(blurrred_image, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        nearest_circle = [0, 0, 0]

        if found_circles is not None:
            for circle in found_circles[0]:
                    if circle[2] > nearest_circle[2]:
                        nearest_circle = circle

        return nearest_circle


    def find_ball(self, blurred_img): # dodati rotiranje s obzirom na lidar
        self.move_robot('a')
        self.found_ball = not np.array_equal(self.find_nearest_circle_from_image(blurred_img), np.zeros(3))
    

    def goto_ball(self, blurred_image, image):
        nearest_circle = self.find_nearest_circle_from_image(blurred_image)        
        if not np.array_equal(self.find_nearest_circle_from_image(blurred_image), np.zeros(3)):
            self.image_error_threshold = 0
            self.set_current_color(image[int(nearest_circle[1]), int(nearest_circle[0])])
            self.get_logger().info(f'Teske boje: {image[int(nearest_circle[1]), int(nearest_circle[0])]}')
            
            rotated = self.rotate_robot(self.find_object_angle(nearest_circle[0]))
            if rotated is True:
                self.move_robot('w')
        else:
            self.image_error_threshold += 1
    
            if self.image_error_threshold == 2:
                self.move_robot('s')
                self.got_ball = True


    def find_goal(self, image):
        if self.found_goal(image, 10, self.color[self.current_color]) is False:
            self.get_logger().info('Nisam pronašao!')
            self.move_robot('d')
        else:
            self.get_logger().info('Pronađoh!')
            self.move_robot('s')
            if self.close_to_obstacle:
                self.move_robot('s')
                self.ball_delivered = True
            else:
                self.move_robot('w')

    def move_robot(self, direction):
        movement = Twist()
        if direction == 'w':
            movement.linear = Vector3(x=0.1, y=0., z=0.)
        if direction == 'd':
            movement.linear = Vector3(x=0., y=0., z=0.)
            movement.angular = Vector3(x=0., y=0., z=-0.15)
        if direction == 'a':
            movement.linear = Vector3(x=0., y=0., z=0.)
            movement.angular = Vector3(x=0., y=0., z=0.15)
        if direction == 's':
            movement.linear = Vector3(x=0., y=0., z=0.)
            movement.angular = Vector3(x=0., y=0., z=0.)
        if direction == 'x':
            movement.linear = Vector3(x=-0.1, y=0., z=0.)
            movement.angular = Vector3(x=0., y=0., z=0.)
        self.velocity_publisher.publish(movement)

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

        rotated = False
        if not 0 <= abs(goal_angle) < 0.3:
            self.get_logger().info(f'Čekaj da se rotiram')
            if goal_angle < 0:         
                self.move_robot('d')
            else:
                self.move_robot('a')
            return rotated
        else:
            self.get_logger().info(f'Gledam ravno u lopticu šefe!')
            self.move_robot('s')
            rotated = True
            return rotated
        
    def found_goal(self, img, strip_width, color):
        img = img[0:img.shape[0], int(img.shape[1]/2 - strip_width):int(img.shape[1]/2 + strip_width)]
        lab_channel = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)[:,:,color[0]]

        if color[1] is False:
            threshold_type = cv2.THRESH_BINARY_INV
        else:
            threshold_type = cv2.THRESH_BINARY

        thresholded_img = cv2.threshold(lab_channel, 127, 255, threshold_type)[1]
        img = cv2.bitwise_and(img, img, mask = thresholded_img)

        self.image_publisher.publish(CvBridge().cv2_to_imgmsg(img, encoding='bgr8'))

        zero_array = np.zeros((img.shape[1],3))
        for i in range(img.shape[0]):
            if np.array_equal(img[i], zero_array):
                return True

        return False
    
    def set_current_color(self, pixel):
        self.current_color = np.argmax(pixel)

    def scan_callback(self, laser_scan):
        angle_matrix = [0, 90, 270]
        ranges = laser_scan.ranges
        for i in range(len(angle_matrix)):
            index = int(np.deg2rad(angle_matrix[i]) / laser_scan.angle_increment)
            self.distance_matrix[i] = ranges[index]

        self.get_logger().info('Ja sam u lidaru')
        if self.distance_matrix[0] <= 0.12 and self.distance_matrix[0] != 0.:
            self.get_logger().info('Makni zid šefe')
            self.close_to_obstacle = True

def main(args=None):
    rclpy.init(args=args)    

    blob_tracker = BallDeliverer()    
    rclpy.spin(blob_tracker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
