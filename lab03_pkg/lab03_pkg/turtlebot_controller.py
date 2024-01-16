from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
from rclpy.qos import qos_profile_sensor_data
import math
import time

class Bump_and_Go_Controller(Node):
    def __init__(self):
        super().__init__('Turtlebot_Controller')
        self.scan_subscription=self.create_subscription(LaserScan, '/scan', self.scan_callback,qos_profile_sensor_data)
        self.velocity_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.timer=self.create_timer(0.1,self.control_loop)
        self.odom_subscription=self.create_subscription(Odometry,'/diff_drive_controller/odom',self.odom_callback,qos_profile_sensor_data)
        #initialization position
        self.scan=None
        self.yaw=0
        self.x=0
        self.y=0
        #initialization motion variables
        self.v=0.2
        self.w=1.0
        self.wall_detection=1.0
        self.free2go=3.5
        #initialization finite state machine states
        self.FORWARD=0
        self.LEFT=1
        self.RIGHT=2
        self.state=self.FORWARD
        #initialization distances from intermediate and final position as infinite values and relative tolerance
        self.outward=8
        self.return_trip=8
        self.tolerance=1
        #intermediate position
        self.intermediate_x=6.5
        self.intermediate_y=2.5
        self.intermediate_yaw=math.pi/2
        #final position
        self.final_x=0
        self.final_y=0
        self.final_yaw=math.pi
        #control counter
        self.i=1


    def control_loop(self):
        if self.scan is None:
            return
        velocity=Twist() 
        if self.outward<self.tolerance and self.i%2==1:
            if (self.yaw-self.intermediate_yaw)>0.05:
                velocity.angular.z=self.w
            else:
                self.i+=1
                self.get_logger().info(f'intermediate position reached. Setting control loop counter to: {self.i}')
                time.sleep(3)
                return
            
        if self.return_trip<self.tolerance/2 and self.i%2==0:
            if (self.yaw-self.final_yaw)>0.05:
                velocity.angular.z=self.w
                return
            else:
                self.i+=1
                self.get_logger().info(f'final position reached. Setting control loop counter to: {self.i}')
                time.sleep(3)    

        if self.state==self.FORWARD:
            velocity.linear.x=self.v
            if self.forward2left():
                self.state_machine(self.LEFT)
                self.get_logger().info(f'Turning Left. State Machine set to: {self.LEFT}')
            if self.forward2right():
                self.state_machine(self.RIGHT)
                self.get_logger().info(f'Turning Right. State Machine set to: {self.RIGHT}')
            

        elif self.state==self.LEFT:
            velocity.angular.z=self.w                 
            if self.left2forward():
                self.state_machine(self.FORWARD)
                self.get_logger().info(f'Proceeding Forward after left turn. Default state: {self.FORWARD}')

        elif self.state==self.RIGHT:
            velocity.angular.z=-self.w
            if self.right2forward():
                self.state_machine(self.FORWARD)
                self.get_logger().info(f'Proceeding Forward after right turn. Default state: {self.FORWARD}')

        self.velocity_pub.publish(velocity) 

    def odom_callback(self,msgs):
        quat=[msgs.pose.pose.orientation.x,msgs.pose.pose.orientation.y,msgs.pose.pose.orientation.z,msgs.pose.pose.orientation.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat)
        self.x=msgs.pose.pose.position.x
        self.y=msgs.pose.pose.position.y
        self.outward=math.sqrt((self.x-self.intermediate_x)**2+(self.y-self.intermediate_y)**2)
        self.return_trip=math.sqrt((self.x-self.final_x)**2+(self.y-self.final_y)**2)
        

    def scan_callback(self,msgs):
        self.scan=msgs
        self.leftscan=round(len(self.scan.ranges)/4)
        self.rightscan=round(len(self.scan.ranges)*(3/4))


    def state_machine(self,state):
        self.state=state

    def forward2left(self):
        return self.scan.ranges[0]<self.wall_detection and self.scan.ranges[self.leftscan]>self.scan.ranges[self.rightscan]
        
    def forward2right(self):
        return self.scan.ranges[0]<self.wall_detection and self.scan.ranges[self.leftscan]<self.scan.ranges[self.rightscan]

    def left2forward(self):        
        return self.scan.ranges[0]>self.free2go
    
    def right2forward(self):
        return self.scan.ranges[0]>self.free2go 

def main(args=None):
    rclpy.init(args=args)
    bump_and_go_controller=Bump_and_Go_Controller()
    rclpy.spin(bump_and_go_controller)
    rclpy.shutdown()

if __name__=='__main__':
    main()