import sys
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import angles

from nav_msgs.msg import Odometry

from enum import Enum
import random
import math

from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator

LINEAR_VELOCITY  = 0.3 #The speed the robot will move when given commands directly by the controller
ANGULAR_VELOCITY = 0.8 #The speed the robot will turn when given commands directly by the controller
#These do not effect the speed of nav2 commands

#Multipliers for making the robot turn left or right, used to make readability easier
LEFT = 1
RIGHT = -1

class State(Enum):
    SELECT = 0
    RANDOM = 1
    
class Random(Enum):
    DRIVE = 0
    TURN = 1

class Command_State(Enum):
    WAITING = 0
    ACTING = 1


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.yaw = 0.0 #The direction the robot is facing - Radians

        initial_pose = PoseStamped() #The initial pose of the robot is the robot's starting location
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        x =self.get_parameter('x').get_parameter_value().double_value #These are used for home pose, search pose, and as adjustments for the robot's coordinate system to the world coordinate system
        y =self.get_parameter('y').get_parameter_value().double_value
        initial_pose.pose.position.x = x #x & y coordinates are recieved from the launch command
        initial_pose.pose.position.y = y 

        (initial_pose.pose.orientation.x,
         initial_pose.pose.orientation.y,
         initial_pose.pose.orientation.z,
         initial_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')
        
        self.navigator = BasicNavigator() #The robot's nav2 controller is started and given the robot's initial pose
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        #Publishers:
        #The cmd publisher publishes movements commands to the cmd_vel topic to make the robot move
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #Odom subscirbes to the robot's odomotry topic to get the believed pose and yaw of the robot
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        #Variable Initiation
        self.main_state = State.SELECT #The main state the robot is in
        self.command_state = Command_State.WAITING #Tracks whether the robot has recieved commands already, or is ready to recieve new commands

        self.pose = Pose()#The robot's current position
        self.previous_pose = Pose()#The robot's position when this variable was last updated, used to track movements

        #Random variables are assigned, then kept through multiple loops to avoid constantly rerolling the values
        self.random_walk_dist = 0.5#How far the robot should move forward
        self.random_turn_angle = 90#How far the robot should turn
        self.random_turn_direction = LEFT #Which direction the robot should turn

        self.random_state = Random.DRIVE

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz #How often the control loop is called
        self.timer = self.create_timer(self.timer_period, self.control_loop) #A timer that calls the control loop every (timer_period) seconds

    def odom_callback(self, msg):
        #The callback function for the odometry topic, updates the pose and yaw when called
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        self.yaw = yaw

    def get_dist(self, x, y): #A function for getting the distance between 2 coordinates when provided the difference
        return math.sqrt(x**2 + y**2)
    
    def turn(self, target, direction):
        #A function called when the robot wants to turn, requires target angle and direction
        #returns false until the robot has turned enough, then returns true
        msg = Twist()
        msg.angular.z = ANGULAR_VELOCITY * direction
        self.cmd_publisher.publish(msg)
        yaw_difference = angles.normalize_angle(self.yaw - self.initial_yaw)
        if math.fabs(yaw_difference) >= math.radians(target):
            return True
        return False

    def drive(self, distance):
        #A function called when the robot wants to drive forward, requires a distance target
        #Checks the target won't cause a crash, reduces the target if it would
        #returns false until the target is reached, then returns true
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY
        self.cmd_publisher.publish(msg)

        diff_x = self.pose.position.x - self.previous_pose.position.x
        diff_y = self.pose.position.y - self.previous_pose.position.y
        #self.get_logger().info(f"x {diff_x}")


        if self.get_dist(diff_x, diff_y) >= distance:
            return True
        return False

    def control_loop(self):
        match self.main_state:
            case State.SELECT: 
                self.main_state = State.RANDOM
                self.command_state = Command_State.WAITING

            case State.RANDOM:
                match self.random_state:
                    case Random.DRIVE:
                        if self.command_state == Command_State.WAITING:
                            self.random_walk_dist = (random.randint(3, 30) / 10)
                            self.previous_pose = self.pose
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.drive(self.random_walk_dist):
                                self.random_state = Random.TURN
                    
                    case Random.TURN:
                        if self.command_state == Command_State.WAITING:
                            self.random_turn_angle = random.uniform(30, 140)
                            self.random_turn_direction = random.choice([LEFT, RIGHT])
                            self.initial_yaw = self.yaw
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.turn(self.random_turn_angle, self.random_turn_direction):
                                self.random_state = Random.DRIVE               

            case _:
                pass

    def destroy_node(self):
        super().destroy_node()
    
def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
