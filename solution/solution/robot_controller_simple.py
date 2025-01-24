import sys
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import angles

from enum import Enum
import random
import math

from geometry_msgs.msg import PoseStamped, Point, Twist, Pose, PoseWithCovarianceStamped, PoseWithCovariance, Vector3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from assessment_interfaces.msg import RobotList, Robot
from solution_interfaces.msg import RobotPosition, CrashCheck, CollisionWarnings, CollisionWarning



LINEAR_VELOCITY  = 0.3 #The speed the robot will move when given commands directly by the controller
ANGULAR_VELOCITY = 0.8 #The speed the robot will turn when given commands directly by the controller
#These do not effect the speed of nav2 commands

#Multipliers for making the robot turn left or right, used to make readability easier
LEFT = 1
RIGHT = -1

SCAN_THRESHOLD = 0.5 #The distance at which scan will trigger for a detected object
#Readability variables for the different scan directions
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

ROBOT_VISION_SIZE = 0.12 #The size a robot has to be in the camera to trigger the vision avoidance state

class State(Enum):
    SELECT = 0
    RANDOM = 1

    OBSTACLE = 4
    ROBOT_VISION = 5
    ROBOT_DISTANCE = 6
    ROLL = 7

class Random(Enum):
    DRIVE = 0
    TURN = 1

class Command_State(Enum):
    #States used to determine whether the robot is waiting for a command, or has recieved a command and is acting on it
    WAITING = 0
    ACTING = 1

class Obstacle_State(Enum):
    #The states the robot can be in when the robot is avoiding an obstacle it has detected
    SELECT = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    REVERSE = 3
    DRIVE = 4

class Robot_Vision_State(Enum):
    #The states the robot can be in when the robot has seen another robot that is too close
    SELECT = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    REVERSE = 3
    STAY = 4
    DRIVE = 5

class Robot_Dist_State(Enum):
    #The states the robot can be in when it has detected another robot is too close.
    PROCESS = 0
    TURN = 1
    CHECK = 2
    DRIVE = 3
    STAY = 4

    


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)


        self.yaw = 0 #The direction the robot is facing - Radians

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

        self.rbt_positions = {'robot1': [-3.5,2], 'robot2': [-3.5,0], 'robot3': [-3.5,-2]}#self.robot_positions contains the known positions of all the robots in the map
        #based on 3 robot spawn positions, will cause issues with 1 and 2 robot systems


        #Robots use the namespace assigned during their generation to determine their id
        name = self.get_namespace()
        self.rbt_id = name.replace('/', '')
        #Publishers:
                
        #The rbt_position publisher is subscribed to by the robot tracker, and updates the robot's position every 0.1 seconds
        self.rbt_position_publisher = self.create_publisher(RobotPosition, 'position', 10)

        #The cmd publisher publishes movements commands to the cmd_vel topic to make the robot move
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        #Subscribers
        #Odom subscirbes to the robot's odomotry topic to get the believed pose and yaw of the robot
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        #Scan subcribes to the Scan topic get the robot's lidar data
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        #Robot spotter subscribes to the robot's robots topic, which provides information on what robots the camera can see, and their size.
        self.robot_spotter_subscriber = self.create_subscription(
            RobotList, 'robots', self.rbt_vision_callback, 10)

        self.crash_detection_subscriber = self.create_subscription(
            CrashCheck, 'crash_detection', self.crash_detection_callback, 10)
        
        self.robot_collisions_subscriber = self.create_subscription(
            CollisionWarnings, '/robot_collisions', self.collision_warning_callback, 10)
        

        #Variable Initiation
        
        self.main_state = State.SELECT #The main state the robot is in
        self.command_state = Command_State.WAITING #Tracks whether the robot has recieved commands already, or is ready to recieve new commands
        self.spin_count = 0#Used to track the number of times the robot has spun, used to allow it to perform 360 degree spins without spinning forever
        self.start_time = self.get_clock().now() #The time when an action was started

        self.obstacle_state = Obstacle_State.SELECT #The obstacle substate is used when the robot is trying to avoid an obstacle
        self.safer_direction = LEFT #Tracks which direction the lidar currently believes is safer, does not account for other robot positions unless they are detected by lidar
        self.safe_distance = 0.5 #The maximum distance that can be travelled in the safe direction before an obstacle is encountered, used to preemptively avoid obstacles
        self.scan_triggered = [False] * 4 #Tracks which directions of scan have been triggered.

        self.robot_vision_state = Robot_Vision_State.SELECT #The robot vision substate is used when the robot is trying to avoid another robot that it has seen
        self.rbt_vision_threat = Robot() #Tracks where the offending robot is in the camera frame, so appropriate action can be taken
        self.rbt_vision_collision = False #Used to trigger avoiding action if a robot is too large in the camera

        self.robot_dist_state = Robot_Dist_State.PROCESS #The robot distance substate is used when the robot is trying to avoid another robot when their believed coordinates are too close
        self.rbt_dist_collision = False #Used to trigger avoiding action when a robot is believed to be too close
        self.escape_angle = 0 #How far the robot needs to turn to face away from and evade the threat
        self.target_yaw = 0 #The direction of the threat the robot is avoiding
        self.escape_direction = LEFT #Which turning direction is quicker to reach the escape angle
        self.start_adjustment = [x, y]#Adjusment values for the robots pose, used so all the robots know the true positions of the other robots and itself

        self.search_pose = PoseStamped() #Contains the coordinates of where the robot should start its search for balls from, if it can't see any valid targets already
        self.search_pose.header.frame_id = 'map'
        self.search_pose.header.stamp = self.get_clock().now().to_msg()
        self.search_pose.pose.position.x = x + 3.75 #Bases the search start position off of the robot's spawn position
        self.search_pose.pose.position.y = y

        (self.search_pose.pose.orientation.x,
        self.search_pose.pose.orientation.y,
        self.search_pose.pose.orientation.z,
        self.search_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')

        self.pose = Pose()#The robot's current position
        self.previous_pose = Pose()#The robot's position when this variable was last updated, used to track movements

        #Random variables are assigned, then kept through multiple loops to avoid constantly rerolling the values
        self.random_walk_dist = 0.5#How far the robot should move forward
        self.random_turn_angle = 90#How far the robot should turn
        self.random_turn_direction = LEFT #Which direction the robot should turn

        self.random_state = Random.DRIVE

        self.crashed = False

        #debug variables:
        self.count = 0
        self.prev_main_state = self.main_state
        


        self.timer_period = 0.1 # 100 milliseconds = 10 Hz #How often the control loop is called
        self.timer = self.create_timer(self.timer_period, self.control_loop) #A timer that calls the control loop every (timer_period) seconds

    

        



    def get_dist(self, x, y): #A function for getting the distance between 2 coordinates when provided the difference
        return math.sqrt(x**2 + y**2)
    
    def odom_callback(self, msg):
        #The callback function for the odometry topic, updates the pose and yaw when called
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        self.yaw = yaw


    def crash_detection_callback(self, msg):
        self.crashed = msg.crashed
        if self.crashed:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_publisher.publish(msg)
            self.navigator.cancelTask()

    def scan_callback(self, msg):
        #The callback function for the scan topic, updates scan triggered, and safe distance and direction, when called
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
        left_ranges  = msg.ranges[31:90]
        back_ranges  = msg.ranges[91:270]
        right_ranges = msg.ranges[271:330]
        focus_front_ranges = msg.ranges[346:359] + msg.ranges[0:15]

        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

        self.safe_distance = min(focus_front_ranges)
        if min(left_ranges) > min(right_ranges):
            self.safer_direction = LEFT
        else: 
            self.safer_direction = RIGHT

    def rbt_vision_callback(self, msg):
        #The callback function for robots, tracks the robots that the camera can see, triggers rbt_vision_collision if a robot is seen too close, and saves the information on that robot
        self.rbt_vision_collision = False 
        for eachItem in msg.data:
            if eachItem.size > ROBOT_VISION_SIZE:
                self.rbt_vision_collision = True
                self.rbt_vision_threat = eachItem

    def collision_warning_callback(self, msg):
        for eachItem in msg.data:
            if self.rbt_id == eachItem.robot_id:
                self.rbt_dist_collision = eachItem.threat
                self.target_yaw = eachItem.angle

    def rbt_position_update(self):
        #publishers the robot's position to /robot_tracker so the information can be passed to the other robots
        rbt_position = RobotPosition()
        rbt_position.robot_id = self.rbt_id
        rbt_position.x = self.pose.position.x + self.start_adjustment[0]
        rbt_position.y = self.pose.position.y + self.start_adjustment[1]
        self.rbt_position_publisher.publish(rbt_position)                     
    
    def get_angle(self, angle):
        #A simple function to avoid angles being assigned that are unachieveable
        while angle > 360:
            angle = angle - 360
        while angle < 0:
            angle += 360
        return angle
    
    def exit_evasion(self):
        #resets main state and command state when the robot has finished evading threats
        self.command_state = Command_State.WAITING
        self.main_state = State.SELECT

    def enter_evasion(self, state):
        #performs a series of checks when the robot enters an evasion state so that it doesn't get stuck swapping between threats and not acting.
        if state != State.ROLL: #If statement ensure's rolls from collisions take priority
            #Second layer of if statements means robot distance overrules other forms of avoidance, except roll
            if state != State.ROBOT_DISTANCE and self.main_state != State.ROBOT_DISTANCE:
                #Third layer of if statements ensures localisation can't overrule obstacle or vision avoidance
                if not self.main_state in [State.ROBOT_VISION, State.OBSTACLE] and self.main_state != state:
                    self.reset_states(state)

            elif self.main_state != state:
                self.reset_states(state)

        elif self.main_state != state:
            self.reset_states(state)
    
    def reset_states(self, state):
        msg = Twist()
        msg.linear.x = float(0)
        msg.angular.z = float(0)
        self.cmd_publisher.publish(msg)
        self.command_state = Command_State.WAITING
        self.main_state = state
        try:
            self.navigator.cancelTask()
        except:
            pass

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
        #self.get_logger().info(f"y {diff_y}")

        if distance > self.safe_distance - SCAN_THRESHOLD:
            distance = self.safe_distance - (SCAN_THRESHOLD + 0.05)
        #self.get_logger().info(f"dist {distance}")

        if self.get_dist(diff_x, diff_y) >= distance:
            return True
        return False

    def reverse(self):
        #A function for doing a 180 degree turn, returns true only when the turn is complete
        if self.command_state == Command_State.WAITING:
            self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
            self.command_state = Command_State.ACTING
        else:
            if self.navigator.isTaskComplete():
                self.command_state = Command_State.WAITING
                return True
            return False
        






    def control_loop(self):
        #A debugging mode to disable robot 2, allows for easier teleop control
        debug = False
        if debug and self.rbt_id == 'robot2':
            return
        

        #publishers the robot's position to /robot_tracker so the information can be passed to the other robots
        self.rbt_position_update()

        #Debugging information
        if self.count >= 3 or self.main_state != self.prev_main_state:
            self.count = 0
            self.prev_main_state = self.main_state
            #self.get_logger().info(f"State: {self.main_state}")
            log = "Robot: " + self.rbt_id + ". State: " + str(self.main_state) + ". Sub State: "
            match self.main_state:
                case State.SELECT:
                    pass
                case State.OBSTACLE:
                    log = log + str(self.obstacle_state)
                    #self.get_logger().info(f"Obstacle State: {self.obstacle_state}")
                case State.ROBOT_VISION:
                    log = log + str(self.robot_vision_state)
                    #self.get_logger().info(f"Vision State: {self.robot_vision_state}")
                case State.ROBOT_DISTANCE:
                    log = log + str(self.robot_dist_state)
                    #self.get_logger().info(f"Distance State: {self.robot_dist_state}")
                case State.ROLL:
                    pass
                case _:
                    pass
            self.get_logger().info(f"{log}")
            #self.get_logger().info(f"new {self.pitch, self.roll}")
            #self.get_logger().info(f"Covariance: {self.cov_matrix}")
        else:
            self.count += 1



        #On each loop, checks if the robot has crashed, is about to crash, or if it is lost
        #In order of priority to ensure the most immediate threats are dealt with first
        if self.crashed:
            #Designed to stop the robot if it does crash. 
            #To reduce the risk of flipping and reduce the localisation error
            #self.get_logger().info(f"Roll Alert")
            self.enter_evasion(State.ROLL)
        elif self.rbt_dist_collision:
            #Activates if the robot detects another robot is too close
            self.enter_evasion(State.ROBOT_DISTANCE)
        elif self.rbt_vision_collision:
            #activates if the robot sees another robot is too close ahead
            self.enter_evasion(State.ROBOT_VISION)
        elif self.scan_triggered[SCAN_FRONT]:
            #activates if the robot is about to drive into a wall or obstacle
            self.enter_evasion(State.OBSTACLE)


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

            case State.OBSTACLE: 
                #This is the state the robot will enter if the lidar detects an obstacle is near
                match self.obstacle_state:
                    case Obstacle_State.SELECT:
                        #In this substate, the robot determines the best way to avoid the obstacle
                        #If it is safe to turn left, and left is the safe direction, then the robot enters the turn_left substate
                        #if it is safe to turn right, and right is the safer direction, then the robot enters the right_turn substate
                        #If neither direction is safe, then the robot enters teh reverse substate
                        msg = Twist()
                        self.cmd_publisher.publish(msg)
                        self.initial_yaw = self.yaw
                        self.previous_pose = self.pose

                        if not self.scan_triggered[SCAN_LEFT] and self.safer_direction == LEFT: #Turn left
                            self.obstacle_state = Obstacle_State.TURN_LEFT
                        elif not self.scan_triggered[SCAN_RIGHT] and self.safer_direction == RIGHT:
                            self.obstacle_state = Obstacle_State.TURN_RIGHT
                        else:
                            self.obstacle_state = Obstacle_State.REVERSE

                    #100 degrees is used for both left and right turns, to avoid the robot getting stuck going in a circle
                    case Obstacle_State.TURN_LEFT:
                        #The robot turns 100 degrees to the left, then goes to the drive substate
                        if self.turn(100, LEFT):
                            self.obstacle_state = Obstacle_State.DRIVE
                    case Obstacle_State.TURN_RIGHT:
                        #The robot turns 100 degrees to the right, then goes to the drive substate
                        if self.turn(100, RIGHT):
                            self.obstacle_state = Obstacle_State.DRIVE
                    case Obstacle_State.REVERSE:
                        #The robot turns 180 degrees, then goes to the drive substate
                        if self.reverse():
                            self.obstacle_state = Obstacle_State.DRIVE
                    case Obstacle_State.DRIVE:
                        #After the turns are complete, the robot enters the drive substate
                        #Here, it drives forward by 0.5 meters, then leaves the obstacle avoidance state
                        if self.drive(0.5):
                            self.exit_evasion()
                            self.obstacle_state = Obstacle_State.SELECT
                    case _:
                        pass






            case State.ROBOT_VISION: 
                #This is the avoidance state for if the robot sees a robot that is too near with its camera 
                match self.robot_vision_state:
                    case Robot_Vision_State.SELECT:
                        #In this substate, the robot determines how best to avoid the robot threat
                        #If the danger is to the right and it is safe to turn left, then the robot enters the turn_left substate
                        #If the danger is to the left and it is safe to turn right, then the robot enters the turn_right substate
                        #If it is safe for the robot it reverse, then the robot enters the reverse substate
                        #And if no other options are safe, the robot enters the stay substate
                        msg = Twist()
                        self.cmd_publisher.publish(msg)
                        self.initial_yaw = self.yaw
                        self.previous_pose = self.pose

                        if self.rbt_vision_threat.x < 0 and not self.scan_triggered[SCAN_LEFT]: #Turn left
                            self.robot_vision_state = Robot_Vision_State.TURN_LEFT
                        elif self.rbt_vision_threat.x >= 0 and not self.scan_triggered[SCAN_RIGHT]:
                            self.robot_vision_state = Robot_Vision_State.TURN_RIGHT
                        elif not self.scan_triggered[SCAN_BACK]:
                            self.robot_vision_state = Robot_Vision_State.REVERSE
                        else:
                            self.start_time = self.get_clock().now() 
                            self.robot_vision_state = Robot_Vision_State.STAY
                            
                    case Robot_Vision_State.TURN_LEFT:
                        #The robot turns 100 degrees to the left, then enteres the drive substate
                        if self.turn(100, LEFT):
                            self.robot_vision_state = Robot_Vision_State.DRIVE

                    case Robot_Vision_State.TURN_RIGHT:
                        #The robot turns 100 degrees to the right, then enteres the drive substate
                        if self.turn(100, RIGHT):
                            self.robot_vision_state = Robot_Vision_State.DRIVE

                    case Robot_Vision_State.REVERSE:
                        #The robot turns 180 degrees, then enteres the drive substate
                        if self.reverse():
                            self.robot_vision_state = Robot_Vision_State.DRIVE

                    case Robot_Vision_State.STAY:
                        #The robot remains still for 3 seconds, then leaves the avoidance state
                        msg = Twist()
                        self.cmd_publisher.publish(msg)

                        if (self.get_clock().now() - self.start_time) > Duration(seconds = 3):
                            self.exit_evasion()
                            self.robot_vision_state = Robot_Vision_State.SELECT

                    case Robot_Vision_State.DRIVE:
                        #The robot drives forward 0.5 meters, then leaves the avoidance state
                        if self.drive(0.5):
                            self.exit_evasion()
                            self.robot_vision_state = Robot_Vision_State.SELECT
                    case _:
                        pass





                
            case State.ROBOT_DISTANCE: 
                #This is the avoidance state for if the robot detects a robot is too close through the coordinate system
                match self.robot_dist_state:
                    case Robot_Dist_State.PROCESS:
                        self.initial_yaw = self.yaw
                        #In this substate, the robot calculates how to turn directly away from the threat, then goes to the robot turn substate
                        self.escape_angle = self.get_angle(math.degrees(self.yaw - (self.target_yaw)) + 180) #gets the target angle
                        if self.escape_angle > 180:
                            self.escape_angle = abs(self.escape_angle - 360)
                            self.escape_direction = RIGHT
                        else:
                            self.escape_direction = LEFT
                        self.robot_dist_state = Robot_Dist_State.TURN

                    case Robot_Dist_State.TURN:
                        #The robot makes a turn based on inputted parameters, then it goes to the check substate once the turn is complete
                        if self.turn(self.escape_angle, self.escape_direction):
                            self.robot_dist_state = Robot_Dist_State.CHECK

                    case Robot_Dist_State.CHECK:
                        #In this substate, the robot checks if it can safely escape
                        #If it is safe to drive forward, then the robot goes to the drive substate
                        #If it is not safe to drive forward, then the robot checks left and right
                        #If it is safe to turn left, and left is the safer direction, then the robot sets parameters for a left turn and enters the turn substate
                        #If it is safe to turn right, and right is the safer direction, then the robot sets parameters for a right turn and enters the turn substate
                        #If neither left, right, or forward is safe. Then the robot enters the stay substate
                        if not self.scan_triggered[SCAN_FRONT]:
                            self.robot_dist_state = Robot_Dist_State.DRIVE
                            self.previous_pose = self.pose
                        elif self.safer_direction == LEFT and not self.scan_triggered[SCAN_LEFT]:
                            self.escape_angle = 100
                            self.escape_direction = LEFT
                            self.robot_dist_state = Robot_Dist_State.TURN
                        elif self.safer_direction == RIGHT and not self.scan_triggered[SCAN_RIGHT]:
                            self.escape_angle = 100
                            self.escape_direction = RIGHT
                            self.robot_dist_state = Robot_Dist_State.TURN
                        else:
                            self.robot_dist_state = Robot_Dist_State.STAY
                            self.command_state = Command_State.WAITING

                    case Robot_Dist_State.DRIVE:
                        #The robot drives forward 0.5 meters, then exits the avoidance state
                        if self.drive(0.5):
                            self.exit_evasion()
                            self.robot_dist_state = Robot_Dist_State.PROCESS

                    case Robot_Dist_State.STAY:
                        #The robot stays still for 3 seconds, then exits the avoidance state
                        if self.command_state == Command_State.WAITING:
                            self.start_time = self.get_clock().now()
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.command_state = Command_State.ACTING
                        elif self.command_state == Command_State.ACTING:
                            if (self.get_clock().now() - self.start_time) > Duration(seconds = 3):
                                self.exit_evasion()
                                self.robot_dist_state = Robot_Dist_State.PROCESS
                        
                    case _:
                        pass






            case State.ROLL:
                #The robot enters the roll state if it detects movement in the pitch or roll directions
                #In the the roll state, the robot stops all movement and waits 3 seconds
                #Once the 3 seconds are finished, the robot leaves the roll state.
                if self.command_state == Command_State.WAITING:
                    self.start_time = self.get_clock().now()
                    msg = Twist()
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.cmd_publisher.publish(msg)
                    self.command_state = Command_State.ACTING
                else:
                    if (self.get_clock().now() - self.start_time) > Duration(seconds = 3):
                        self.exit_evasion()
                
                

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
