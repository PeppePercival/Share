import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from lab02_interfaces.action import MoveDistance
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from math import pow, sqrt

class MovingActionServer(Node):
    def __init__(self):
        super().__init__("ActionServer")
        # CALLBACK GROUPS
        # Define a new callback group for the action server.
        # The action server will be assigned to this group, while the subscribers, publishers and
        # timers will be assigned to the default callback group (which is MutuallyExclusive)
        self.action_server_group = ReentrantCallbackGroup()
        self.topic_group = MutuallyExclusiveCallbackGroup()
        # SUBSCRIBERS, PUBLISHERS, TIMERS...
        # Subscribers and publishers can stay in the default group since they are non-blocking, to 
        # do so just declare them as usual.
        #
        # You can also assign them explicitely to a MutuallyExclusiveCallbackGroup if you want to by
        # passing the callback_group argument.
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10, callback_group=self.topic_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10, callback_group=self.topic_group)
        self.velocity_callback_timer = self.create_timer(0.1,self.velocity_callback, callback_group=self.topic_group)
        # The action server must be in a different callback group since it is blocking.
        # The action server callback will sleep at a fixed rate until the goal is completed, failed 
        # or canceled. The action server will use a ReentrantCallbackGroup so that multiple goals 
        # can be handled simultaneously.
        self.action_server = ActionServer(
            self,
            MoveDistance,
            "/turtle1/move_distance",
            self.execute_action_callback,
            callback_group=self.action_server_group,
        )

        
        control_rate_hz = 2
        self.control_rate = self.create_rate(control_rate_hz)

        self.get_logger().info(
            "Move distance action server started with namespace: "
            + self.get_namespace()
            + " ..."
        )
        
        self.declare_parameters('lab02',[
            ('scale_linear', 0.0),
            ('scale_goal', 0.0)
        ])

        self.starting_pose: bool = False
        self.request_arrived: bool = False
        self.initial_x: float = 0.0
        self.initial_y: float = 0.0
        self.velocity_x: float =  0.0
        self.maximum_velocity: float = self.get_parameter_or('lab02.scale_linear', 0.5).value
        self.distance_threshold = self.get_parameter_or('lab02.scale_goal', 0.01).value
        self.elapsed_time: float = 0.0
        self.traveled_distance: float = 0.0
        self.counter = 0
        self.get_logger().info("Initialzed client with max velocity: %.3f and distance treashold: %.5f..." % (self.maximum_velocity, self.distance_threshold))

    def travel_distance(self, current_pose: Pose):
        if(current_pose.x != self.initial_x or current_pose.y != self.initial_y):
            self.traveled_distance = abs((sqrt(pow(abs( current_pose.x - self.initial_x), 2) + pow(abs(current_pose.y - self.initial_y ), 2))))

    def velocity_callback(self):
        if self.request_arrived:
            msg: Twist = Twist(linear=Vector3(x=self.velocity_x))
            self.cmd_vel_publisher.publish(msg)
            self.elapsed_time += 0.1
            
  
    def pose_callback(self, msg: Pose):
        if self.request_arrived:
            if not self.starting_pose:
                self.initial_x = msg.x
                self.initial_y = msg.y
                self.starting_pose = True
                self.get_logger().info("initial X: %.5f, Y: %.5f..." % (self.initial_x, self.initial_y))
            else:
                self.travel_distance(msg)

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        '''
        This function is called when a new goal is received by the action server.
        Pay attention to return the correct result object, otherwise the action client will crash.
        
        Returns:
            result {YourAction.Result} -- Result object
        '''
        self.request_arrived = True
        while not self.starting_pose:
            pass
        self.counter = 0
        self.traveled_distance = 0.0
        self.elapsed_time = 0.0
        self.velocity_x = self.maximum_velocity
        self.get_logger().info("Executing new goal for distance: %.5f..." % goal_handle.request.distance)
        feedback_msg = MoveDistance.Feedback()
        result = MoveDistance.Result()

       
        while round(goal_handle.request.distance - self.traveled_distance,5) > self.distance_threshold:
            feedback_msg.remaining_distance = round(goal_handle.request.distance - self.traveled_distance,5)
            if self.counter % 2 == 0:
                goal_handle.publish_feedback(feedback_msg)
            self.control_rate.sleep()

        self.velocity_x = 0.0
        result.elapsed_time_s = self.elapsed_time
        result.traveled_distance = self.traveled_distance
        self.starting_pose = False
        self.request_arrived = False
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovingActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
