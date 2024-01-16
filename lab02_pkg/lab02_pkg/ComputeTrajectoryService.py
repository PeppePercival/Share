import rclpy
from rclpy.node import Node
from lab02_interfaces.srv import ComputeTrajectory
from turtlesim.msg import Pose
import math

class ComputeTrajectoryService(Node):

    def __init__(self):
        super().__init__('compute_trajectory_server')
        self.srv = self.create_service(ComputeTrajectory, '/turtle1/compute_trajectory', self.calculate_trajectory)
        # pose subscription try
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info('Starting Service')

    def calculate_orientation(self, current_pose_x, current_pose_y, goal_pose_x, goal_pose_y):
        
        delta_x = goal_pose_x - current_pose_x
        delta_y = goal_pose_y - current_pose_y
        orientation_deg = math.atan2(delta_y, delta_x)
        #orientation_deg = math.degrees(orientation)
        self.get_logger().info('Orientation calculated: %f radians' % orientation_deg)
        self.get_logger().debug('Delta X: %f, Delta Y: %f' % (delta_x, delta_y))
        return orientation_deg

    def calculate_distance(self, current_pose_x, current_pose_y, goal_pose_x, goal_pose_y):
        delta_x = goal_pose_x - current_pose_x
        delta_y = goal_pose_y - current_pose_y
        
        distance = math.sqrt(delta_x**2+delta_y**2)
        self.get_logger().info('Distance calculated: %f units' % distance)
        return distance
    
    def pose_callback(self, msg): 
        self.current_pose_x = msg.x
        self.current_pose_y = msg.y 

    def calculate_trajectory(self, request, response):
        goal_pose_x = request.x  
        goal_pose_y = request.y 

        response.distance = self.calculate_distance(self.current_pose_x, self.current_pose_y, goal_pose_x, goal_pose_y)
        response.orientation = self.calculate_orientation(self.current_pose_x, self.current_pose_y, goal_pose_x, goal_pose_y)
        self.get_logger().info('Received request: Current Pose (x: %f, y: %f), Goal Pose (x: %f, y: %f)' %
                            (self.current_pose_x, self.current_pose_y, goal_pose_x, goal_pose_y))

        return response

def main(args=None):
    rclpy.init(args=args)
    compute_trajectory_server = ComputeTrajectoryService()
    rclpy.spin(compute_trajectory_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
