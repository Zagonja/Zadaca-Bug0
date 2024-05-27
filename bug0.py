import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry




class Bug0(Node):
    #Initialization
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.isNormalized = False
        self.isPastObstacle = False
        self.corneringDone = False
        self.normal_threshold = 0.05
        self.cornering_omega = 0.3
        self.cornering_radius = 0.8
        self.corner_theta = None
        self.cornering_vel = self.cornering_radius * self.cornering_omega
        self.cmd_vel_msg = Twist()
        #dodano
        self.obstacle_threshold = 0.45 # Threshold distance to consider an obstacle
        self.state = 'MOVE_TO_GOAL' #'MOVE_TO_GOAL' 'FOLLOW_OBSTACLE' 'CORNERING' 'NORMALIZING'

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range
        print(self.fl_sensor_value)   
    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range
        print(self.fr_sensor_value)  
    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None or self.current_x is None or self.current_y is None:
            return     

        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)
        print("Currently doing: ", self.state)
        
        goal_distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        # Postavljanje rada algoritma
        if goal_distance < 0.15:        
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            self.isNormalized = False
            self.isPastObstacle = False
            self.corneringDone = False
            self.state = 'MOVE_TO_GOAL'
            print("Goal reached!")
            return

        if self.state == 'MOVE_TO_GOAL' :  
            if (self.fl_sensor_value < self.obstacle_threshold or self.fr_sensor_value < self.obstacle_threshold) and abs(goal_angle-self.current_theta) < 0.1:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'NORMALIZING'
                self.normalize()
            else:
                self.move_to_goal(goal_angle)

        if self.state == 'NORMALIZING':
            if self.isNormalized == True:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'FOLLOW_OBSTACLE'
                self.follow_obstacle()
            else:
                self.normalize()

        if self.state == 'FOLLOW_OBSTACLE':
            if self.isPastObstacle == True:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'CORNERING'
                self.corner()
            else:
                self.follow_obstacle()

        if self.state == 'CORNERING':
            if self.corneringDone == True:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.isNormalized = False
                self.isPastObstacle = False
                self.corneringDone = False
                self.state = 'MOVE_TO_GOAL'
                self.move_to_goal(goal_angle)
            else:
                self.corner()
            



    def normalize(self):    
        if(abs(self.fl_sensor_value - self.fr_sensor_value) < self.normal_threshold):
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            self.isNormalized = True
            self.normalTheta = self.current_theta
            return
        if(self.fl_sensor_value < self.fr_sensor_value):
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.1
            self.cmd_pub.publish(self.cmd_vel_msg)
            return
        if(self.fl_sensor_value > self.fr_sensor_value):
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = -0.1
            self.cmd_pub.publish(self.cmd_vel_msg)
            return

    def follow_obstacle(self):  # pratnja prepreke
        self.cmd_vel_msg.linear.x = 0.0 
        angle_diff = self.normalize_angle((self.normalTheta-math.pi/2) - self.current_theta)
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            if angle_diff >0:
                self.cmd_vel_msg.angular.z = 0.1
            else:
                self.cmd_vel_msg.angular.z = -0.1
        else:
            if self.fl_sensor_value < 0.8:
                self.cmd_vel_msg.linear.x = 0.15
            else:
                self.isPastObstacle = True
                self.corner_theta = self.current_theta
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
        
        self.cmd_pub.publish(self.cmd_vel_msg)
        return
    
    def corner(self):      
        angle_diff = self.normalize_angle((self.corner_theta + 2*math.pi/3) - self.current_theta)
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = self.cornering_vel
            self.cmd_vel_msg.angular.z = self.cornering_omega
        else:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.corneringDone = True
        
        self.cmd_pub.publish(self.cmd_vel_msg)
        return
        
    def move_to_goal(self, goal_angle):     
        self.cmd_vel_msg.linear.x = 0.0 
        angle_diff = self.normalize_angle(goal_angle - self.current_theta)
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            if angle_diff >0:
                self.cmd_vel_msg.angular.z = 0.3
            else:
                self.cmd_vel_msg.angular.z = -0.3
            
        else:
            if self.fl_sensor_value >= self.obstacle_threshold and self.fr_sensor_value >= self.obstacle_threshold:   
                self.cmd_vel_msg.linear.x = 0.2    
                self.cmd_vel_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_vel_msg)


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
            
def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug0()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()