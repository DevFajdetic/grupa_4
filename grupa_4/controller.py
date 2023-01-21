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
        self.color_picked = False
        self.image_error_threshold = 0
        self.find_ball_error_threshold = 0
        self.current_color_index = 2
        self.colors = [
            [2, True, 125], #blue
            #[1, True, 110], #green
            [1, False, 135], #red
            [-1, False] #no valid color
        ]
        self.delivered_colors = [False, False, False]
        self.distance_matrix = [4, 4, 4] #[front, left, right]
        self.declare_parameter('Kp', 0.7)
        self.declare_parameter('f', 265.23)
        self.declare_parameter('u0', 160.5)

    def image_listener_callback(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg)
        imGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.medianBlur(imGray, 25)

        self.get_logger().info(f'Current_color: {self.current_color_index}')

        if self.found_ball is False:
            self.find_ball(blurred_img)
        elif self.got_ball is False:
            self.goto_ball(blurred_img, image)
        elif self.ball_delivered is False:
            self.find_goal(image)
        elif self.distance_matrix[0] <= 0.25 and self.distance_matrix[0] != 0:
            self.move_robot('x')
        else:
            self.reset()
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
        if not np.array_equal(self.find_nearest_circle_from_image(blurred_img), np.zeros(3)):
            self.find_ball_error_threshold += 1
            if self.find_ball_error_threshold == 2:
                self.found_ball = True
            return
        self.find_ball_error_threshold = 0
    

    def goto_ball(self, blurred_image, image):
        nearest_circle = self.find_nearest_circle_from_image(blurred_image)        
        circle_found = not np.array_equal(nearest_circle, np.zeros(3))
        
        if circle_found:
            self.image_error_threshold = 0
            circle_center_pixel = image[int(nearest_circle[1]), int(nearest_circle[0])]

            if not self.color_picked:
                self.pick_color(circle_center_pixel)
            elif np.argmax(circle_center_pixel) != self.current_color_index:
                return 
            
            if not self.found_ball:
                return

            is_rotated = self.rotate_robot(self.find_object_angle(nearest_circle[0]))
            if is_rotated:
                self.move_robot('w')
        else:
            self.image_error_threshold += 1
    
            if self.image_error_threshold == 2:
                self.move_robot('s')
                self.got_ball = True


    def pick_color(self, pixel):
        self.set_current_color(pixel)
        self.get_logger().info(f'Teske boje: {pixel}')

    def find_goal(self, image):
        if self.found_goal(image, 10, self.colors[self.current_color_index]):
            self.get_logger().info('Pronađoh!')
            self.move_robot('s')
            if self.close_to_obstacle:
                self.move_robot('s')
                self.ball_delivered = True
            else:
                self.move_robot('w')
        else:
            self.get_logger().info('Nisam pronašao!')
            self.move_robot('d')

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
        if color[0] == -1:
            self.get_logger().info(f'Nemam boju da trazim goal sefe')
            return False

        self.get_logger().info(f'{color}')
        img = img[0:img.shape[0], int(img.shape[1]/2 - strip_width):int(img.shape[1]/2 + strip_width)]
        lab_channel = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)[:,:,color[0]]

        if color[1] is False:
            threshold_type = cv2.THRESH_BINARY_INV
        else:
            threshold_type = cv2.THRESH_BINARY

        thresholded_img = cv2.threshold(lab_channel, color[2], 255, threshold_type)[1]
        img = cv2.bitwise_and(img, img, mask = thresholded_img)

        self.image_publisher.publish(CvBridge().cv2_to_imgmsg(img, encoding='bgr8'))
        height = 0
        continuous = False
        zero_array = np.zeros((img.shape[1],3))
        for i in range(img.shape[0]):
            if np.array_equal(img[i], zero_array):
                if continuous is False:
                    continuous = True
                
                if continuous is True:
                    height +=1
                    if height == 40:
                        return True
            else:
                continuous = False

        return False
    
    def set_current_color(self, pixel):
        color_index = np.argmax(pixel)
        if color_index == 2:
            color_index = 1
        black = [75, 75, 75]
        white = [175, 175, 175]
        is_black = np.all(np.greater(black, pixel))
        is_white = np.all(np.less(white, pixel))
        if is_black or is_white:
            return False
        if self.delivered_colors[color_index] is False:
            self.current_color_index = color_index
            self.color_picked = True
            return True
        else:
            self.found_ball = False
        
        return False
        

    def scan_callback(self, laser_scan):
        angle_matrix = [0, 90, 270]
        ranges = laser_scan.ranges
        for i in range(len(angle_matrix)):
            index = int(np.deg2rad(angle_matrix[i]) / laser_scan.angle_increment)
            self.distance_matrix[i] = ranges[index]

        if self.distance_matrix[0] <= 0.18 and self.distance_matrix[0] != 0.:
            self.get_logger().info('Makni zid šefe')
            self.close_to_obstacle = True

    def reset(self):
        self.delivered_colors[self.current_color_index] = True
        self.got_ball =  False
        self.found_ball = False
        self.ball_delivered = False
        self.close_to_obstacle = False
        self.color_picked = False
        self.image_error_threshold = 0
        self.find_ball_error_threshold = 0
        self.current_color_index = 2


def main(args=None):
    rclpy.init(args=args)    

    blob_tracker = BallDeliverer()    
    rclpy.spin(blob_tracker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
