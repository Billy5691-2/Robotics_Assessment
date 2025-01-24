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

from assessment_interfaces.msg import HomeZone, RobotList, ItemHolders, ItemHolder, Item, Robot
from solution_interfaces.msg import RobotPosition, RobotAssignment, RobotAssignments, TrackedItem, CrashCheck, CollisionWarnings, CollisionWarning



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

ROBOT_VISION_SIZE = 0.11 #The size a robot has to be in the camera to trigger the vision avoidance state

NULL = 0 #Used when assigning each robot's target colour. 
#Null is the default value for diameter, representing the robot not being able to see any items of that colour

BALL_OVERRIDE_DIAMETER = 35 #If an item of the correct type is visible at this size, certain stages of collection and delivery can be skipped

class State(Enum):
    SELECT = 0#3 states for normal robot control
    BALL= 1
    HOME = 2

    #5 states that override normal robot control, for crash avoidance and localisation
    LOCALISE = 3
    OBSTACLE = 4
    ROBOT_VISION = 5
    ROBOT_DISTANCE = 6
    ROLL = 7

class Command_State(Enum):
    #States used to determine whether the robot is waiting for a command, or has recieved a command and is acting on it
    #This is basically just a True or False state, but more readable
    WAITING = 0
    ACTING = 1

class Ball_State(Enum):
    #The states the robot can be in when actively searching for a ball
    SELECT = 0
    SEARCH_START = 1
    SPIN = 2
    AIM = 3
    DRIVE = 4
    HOME_PREP = 5

    #The 2 random states are only used when the normal steps can't find an item of the correct colour
    RANDOM_TURN = 6
    RANDOM_DRIVE = 7

class Home_State(Enum):
    #The states the robot can be in when actively navigating back to the home_zone
    SELECT = 0
    NAVIGATE = 1
    ITEM_SWAP = 2

    #These 5 states are only used when NAV2 can't be trusted to guide the robot home
    SPIN = 3
    AIM = 4
    DRIVE = 5
    RANDOM_TURN = 6
    RANDOM_DRIVE = 7

class Localise_State(Enum):
    #The two states of localisation
    DRIVE = 0
    TURN = 1

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
    #The robot controller contains most of the functionality of the robots,
    #including how it reacts to items and home, and how it acts to avoid obstacles

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

        #Initial yaw is used to determine how far the robot has turned during a maneuver
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.rbt_positions = {'robot1': [-3.5,2], 'robot2': [-3.5,0], 'robot3': [-3.5,-2]}#self.robot_positions contains the known positions of all the robots in the map
        #based on 3 robot spawn positions, will cause issues with 1 and 2 robot systems


        #Robots use the namespace assigned during their generation to determine their id
        name = self.get_namespace()
        self.rbt_id = name.replace('/', '')

        
        #Publishers:
                
        #The robot publishes to positions every loop, to update its coordinates in the world
        self.rbt_position_publisher = self.create_publisher(RobotPosition, 'position', 10)

        #The cmd publisher publishes movements commands to the cmd_vel topic to make the robot move
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        #Subscribers
        #odom contains Odometry information, included the robot's pose relative to where it started, and its yaw
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        #Scan subcribes to the Scan topic get the robot's lidar data
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        #Robot spotter subscribes to the robot's robots topic, which provides information on what robots the camera can see, and their size.
        self.robot_spotter_subscriber = self.create_subscription(
            RobotList, 'robots', self.rbt_vision_callback, 10)

        #Home zone subscribes to the robot's home_zone topic, which provides information on whether the robot can see the homezone, and where it is in the camera.
        self.home_zone_subscriber = self.create_subscription(
            HomeZone, 'home_zone', self.home_zone_callback, 10)

        #target_item provides the information on the best item in the robot's camera
        self.item_tracker_subscriber = self.create_subscription(
            TrackedItem, 'target_item', self.item_tracker_callback, 10)

        #Item Holder subscribes to the root topic item_holders, which lists what items each robot is holding.
        self.item_holder_subscriber = self.create_subscription( 
            ItemHolders, '/item_holders', self.holding_item_callback, 10)
        
        #Amcl_pose subscribes to the robot's amcl_pose topic, which tracks the robot's position according to nav2, and the accuracy of that position
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.rbt_amcl_pose_callback, 10)

        #crash_detection updates whether the robot is currently experiencing a collision
        self.crash_detection_subscriber = self.create_subscription(
            CrashCheck, 'crash_detection', self.crash_detection_callback, 10)

        #/robot_collisions warns if the robot is near another robot, and the yaw required to move away from the nearest robot if needed
        self.robot_collisions_subscriber = self.create_subscription(
            CollisionWarnings, '/robot_collisions', self.collision_warning_callback, 10)
        
        #robot targets subscribes to the root topic robot_assignments, which publishes the item assignments of each robot once it has calculated them
        self.robot_targets_subscriber = self.create_subscription(
            RobotAssignments, '/robot_assignments', self.robot_target_callback, 10)
        

        #Variable Initiation
        
        self.main_state = State.SELECT #The main state the robot is in
        self.command_state = Command_State.WAITING #Tracks whether the robot has recieved commands already, or is ready to recieve new commands
        self.spin_count = 0#Used to track the number of times the robot has spun, used to allow it to perform 360 degree spins without spinning forever
        self.start_time = self.get_clock().now() #The time when an action was started

        self.localise_state = Localise_State.DRIVE #The localise substate is used only when the robot is trying to localise
        self.location_confident = False #This variable tracks whether the robot is confident in its position, used to trigger localisation state.

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

        self.ball_state = Ball_State.SELECT #The ball substate is used when the robot is searching for a ball
        self.item_selected = True #Used to trigger appropriate actions when the robot can see a valid ball in the camera
        self.target_item = Item() #Contains information about the target ball, such as its position in the camera frame
        self.ball_search_time = self.get_clock().now()#Used to track how long the robot has been searching for a specific ball that it can't see anymore
        self.reached_search_start = False #Used to track whether the robot has reached its starting point for its search for balls or not, to avoid infinite loops
        self.aim_timer = self.get_clock().now()#Tracks how long the robot has been trying to aim at a target, to avoid infinite loops of aiming between 2 equally valid targets
        self.home_yaw = 0

        self.search_pose = PoseStamped() #Contains the coordinates of where the robot should start its search for balls from, if it can't see any valid targets already
        self.search_pose.header.frame_id = 'map'
        self.search_pose.header.stamp = self.get_clock().now().to_msg()
        self.search_pose.pose.position.x = x + 3.75 #Bases the search start position off of the robot's spawn position
        self.search_pose.pose.position.y = y

        (self.search_pose.pose.orientation.x,
        self.search_pose.pose.orientation.y,
        self.search_pose.pose.orientation.z,
        self.search_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')

        self.home_state = Home_State.SELECT #The home substate is used when the robot is searching for home, after picking up a ball
        self.home_pose = initial_pose #Home pose remembers where the robot spawned, as it always spawns in the homezone
        self.home_pose_accurate = True #Tracks whether the robot thinks the home pose is still accurate, or if errors mean it is no longer of use so it should to revert to simplier navigation
        self.held_item = ItemHolder() #Tracks what item the robot is holding
        self.prev_held_item = ItemHolder() #Tracks what item the robot was holding, used when the robot accidentally swaps items
        self.nav_count = 0 #Tracks how many time the robot has tried to navigate but swapped items, avoids an edge case of the robot never navigating because it is constantly swapping items
        self.home_timer = self.get_clock().now() #Tracks how long the robot has been trying to get home without nav2

        self.pose = Pose()#The robot's current position
        self.previous_pose = Pose()#The robot's position when this variable was last updated, used to track movements

        #Random variables are assigned, then kept through multiple loops to avoid constantly rerolling the values
        self.random_walk_dist = 0.5#How far the robot should move forward
        self.random_turn_angle = 90#How far the robot should turn
        self.random_turn_direction = LEFT #Which direction the robot should turn

        #Variables for colour assignments
        self.item_assigned = False #A boolean which makes the robot act differently to its usual state when waiting to be assigned a target colour
        self.target_colour = "RED" #A default value that will be replaced with the actual colour the robot will target

        self.crashed = False #A flag for if the robot is experiencing the effects of a collsion, i.e. rolling or pitching

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
        #Updates the crashed flag. If a crash is detected,
        # #all movement and commands are immediately stopped to try and preserve accuracy of the robots position
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

    def home_zone_callback(self, msg): 
        #The callback function for the home_zone topic, updates the self.home_zone variable when called
        self.home_zone = msg

    def item_tracker_callback(self,msg):
        #Updates information on the best visible item, as well whether a valid item is visible
        self.target_item = msg.item
        self.item_selected = msg.visible

    def robot_target_callback(self, msg):
        #This callback function gets the assignments of each robot from robot_targets,
        #it then identifies the data relevant to the robot and updates its target colour, as well as signalling it has been assigned a target
        for eachRobot in msg.data:
            if eachRobot.robot_id == self.rbt_id:
                self.target_colour = eachRobot.colour
                self.item_assigned = True
               
    def holding_item_callback(self, msg):
        #The callback function for /item_holders, filters information for other robots then updates held_item and prev_held_item
        if len(msg.data) > 0:
            for eachItem in msg.data:
                if eachItem.robot_id == self.rbt_id:
                    if eachItem.item_colour != self.held_item.item_colour:
                        self.prev_held_item = self.held_item
                    self.held_item = eachItem
        else:
            self.held_item = ItemHolder()

    def rbt_amcl_pose_callback(self, msg):
        #The callback function for amcl_pose, used to update the robot's confidence in its location
        #Lower covariance values mean more confidence, 1 has been chosen as the cut-off point
        #covariance matrix is 6x6 matrix, represented as 36 item long list
        #values on the diagonal of matrix represent accuracy of each value
        #x(0,0), y(1,1), z(2,2), pitch(3,3), roll(4,4), yaw(5,5)
        #smaller value means more confidence
        covariance_matrix = msg.pose.covariance
        confidence_x = covariance_matrix[0]
        confidence_y = covariance_matrix[7]
        if min([confidence_x, confidence_y]) > 1:
            self.location_confident = False
        else:
            self.location_confident = True

    def rbt_vision_callback(self, msg):
        #The callback function for robots, tracks the robots that the camera can see, triggers rbt_vision_collision if a robot is seen too close, and saves the information on that robot
        self.rbt_vision_collision = False 
        for eachItem in msg.data:
            if eachItem.size > ROBOT_VISION_SIZE:
                self.rbt_vision_collision = True
                self.rbt_vision_threat = eachItem

    def collision_warning_callback(self, msg):
        #Recieves data on if the robot is at risk of colliding with another robot, and the yaw required to move away if needed
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
        #Sets multiple values to a non-moving, uncommanded state so evasion can begin from a blank slate.
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
        #publishers the robot's position to /robot_tracker so the information can be passed to the other robots
        self.rbt_position_update()

        #A debugging mode to disable robot 1, allows for easier teleop control
        debug = True
        if debug and self.rbt_id == 'robot1':
            return
        
        if not self.item_assigned: 
            #This stops the robot from progressing into the statemachine until it has recieved an item assignment
            return

        #Debugging information
        if self.count >= 10 or self.main_state != self.prev_main_state:
            self.count = 0
            self.prev_main_state = self.main_state
            #self.get_logger().info(f"State: {self.main_state}")
            log = "State: " + str(self.main_state) + ". Sub State: "
            match self.main_state:
                case State.SELECT:
                    pass
                case State.BALL:
                    log = log + str(self.ball_state)
                    #self.get_logger().info(f"Ball State: {self.ball_state}")
                case State.HOME:
                    log = log + str(self.home_state)
                    #self.get_logger().info(f"Home State: {self.home_state}")
                case State.LOCALISE:
                    log = log + str(self.localise_state)
                    #self.get_logger().info(f"Local State: {self.localise_state}")
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
        elif not self.location_confident: 
            #activates if the robot is no longer confident in its position in the world
            self.enter_evasion(State.LOCALISE)






        match self.main_state:
            case State.SELECT: 
                #Select state is the default state for the robot
                #It determine what control state the robot should be in considering its current parameters
                #The robot reaches this state when a robot reaches home, reaches a ball, or completes evasion maneuvers
                #The robot can either be put into the home state, if it is holding an item of the right colour
                #Or the robot can be put into the ball state if it is not holding the correct item, or no item.
                
                if self.held_item.item_colour == self.target_colour:
                    self.main_state = State.HOME
                    self.home_state = Home_State.SELECT
                    self.command_state = Command_State.WAITING
                    self.nav_count = 0
                    self.reached_search_start = False
                else:
                    self.main_state = State.BALL
                    self.ball_state = Ball_State.SELECT
                    self.command_state = Command_State.WAITING
                    self.ball_search_time = self.get_clock().now()




            
            case State.BALL:
                #State.Ball is for when the robot is searching for an item.
                match self.ball_state:
                    case Ball_State.SELECT:
                        #Ball_State.SELECT determines which substate of Ball the robot should be in, from its current parameters.
                        self.command_state = Command_State.WAITING
                        if self.held_item.item_colour == self.target_colour: 
                            #If the robot has accidentally picked up its target item, it leaves the ball state
                            self.main_state = State.SELECT

                        if not self.reached_search_start and self.target_item.diameter < BALL_OVERRIDE_DIAMETER: 
                            #If the robot can't see an item that is close enough (big enough), and it has never reached the search pose since entering the ball state,
                            #then the robot will enter the SEARCH_START substate where it will navigate to a predetermined pose
                            self.ball_state = Ball_State.SEARCH_START

                        elif self.item_selected:
                            #If the robot can see an item of the target colour, it will begin the process of collecting the item by aiming at it
                            self.ball_state = Ball_State.AIM
                            self.aim_timer = self.get_clock().now()

                        elif (self.get_clock().now() - self.ball_search_time) > Duration(seconds = 10):
                            #If the robot has reached the search state, spun around, and still not found an item, then it will begin randomly searching
                            self.ball_state = Ball_State.RANDOM_DRIVE
                        else:
                            #The robot will try spinning to look for an item if all other parameters aren't met. 
                            self.ball_state = Ball_State.SPIN

                    case Ball_State.SEARCH_START:
                        #In this sub state, the robot sets a waypoint at a predetermined point, then navigates to it with nav2
                        #Once it arrives, it marks that it has arrived at the point so that the robot does not get stuck in a loop
                        #If it collects a ball on its way to the waypoint, it cancels the navigation so it can head home
                        if self.command_state == Command_State.WAITING:
                            self.search_pose.header.stamp = self.get_clock().now().to_msg()
                            self.navigator.goToPose(self.search_pose)
                            self.command_state = Command_State.ACTING
                        else:
                            if self.navigator.isTaskComplete():
                                self.ball_state = Ball_State.SELECT
                                self.command_state = Command_State.WAITING
                                self.ball_search_time = self.get_clock().now()
                                self.reached_search_start = True
                            elif self.held_item.item_colour == self.target_colour:
                                self.navigator.cancelTask()
                                self.ball_state = Ball_State.HOME_PREP
                                self.command_state = Command_State.WAITING

                                
                        
                    case Ball_State.SPIN:
                        #In this sub state, the robot attempts to complete a full 360 turn to look for an item.
                        #If it sees one while turning, it stops spinning and goes to the aim sub state
                        #If it completes a full spin without seeing an item, then it goes to the random drive sub state
                        if self.command_state == Command_State.WAITING:
                            self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                            self.spin_count = 0
                            self.command_state = Command_State.ACTING
                        else:
                            if self.item_selected:
                                self.navigator.cancelTask()
                                self.ball_state = Ball_State.AIM
                                self.aim_timer = self.get_clock().now()
                                self.command_state = Command_State.WAITING
                            elif self.navigator.isTaskComplete():
                                if self.spin_count > 0:
                                    self.ball_state = Ball_State.RANDOM_DRIVE
                                    self.command_state = Command_State.WAITING
                                else:
                                    self.spin_count += 1
                                    self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)

                    case Ball_State.AIM:
                        #In this state, the robot spends a small amount of time time turning to aim at the largest item of the correct colour in view
                        #If it spends too long looking, or gets accurate enough, then it goes to the drive state
                        #If it loses sight of all possible targets while aiming, then it goes to the random drive state to avoid infinite loops
                        #If it somehow picked up the correct item while in this state, the robot leaves the ball state and goes back to state select
                        msg = Twist()
                        msg.angular.z = ANGULAR_VELOCITY * ((5 * self.target_item.x) / 320)
                        self.cmd_publisher.publish(msg)
                        if ((self.target_item.x < 15 and self.target_item.x > -15) 
                            or (self.get_clock().now() - self.aim_timer) > Duration(seconds = 1)):
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.ball_state = Ball_State.DRIVE
                        elif not self.item_selected:
                            self.ball_state = Ball_State.RANDOM_DRIVE
                        if (self.held_item.item_colour == self.target_colour):
                            self.ball_state = Ball_State.HOME_PREP

                    case Ball_State.DRIVE:
                        #In this sub state, the robot drives towards a targetted item
                        #It turns towards the item as it drives
                        #If it succeeds at picking up an item of the right colour, it leaves the ball state
                        #If it loses sight of all items, it continues driving forward for 1.5 seconds, then returns to the ball select state if it didn't pick up the item
                        #The timer is included as when an item is too close, it is no longer identified by the camera.
                        if self.item_selected:
                                self.ball_search_time = self.get_clock().now()
                        msg = Twist()
                        msg.linear.x = LINEAR_VELOCITY
                        msg.angular.z = ANGULAR_VELOCITY * (self.target_item.x /320)
                        self.cmd_publisher.publish(msg)
                        if (self.held_item.item_colour == self.target_colour):
                            self.ball_state = Ball_State.HOME_PREP
                        elif (self.get_clock().now() - self.ball_search_time) > Duration(seconds = 1.5):
                            self.ball_state = Ball_State.SELECT

                    case Ball_State.HOME_PREP:
                        #After picking up an item, the robot will turn to face the position it started from
                        #This is slightly less efficient than just letting nav2 takeover,
                        #but it reduces the risk of a bug occuring when nav2 has to make a near 180degree turn
                        if self.command_state == Command_State.WAITING:
                            self.home_yaw = self.get_angle(math.degrees(
                                self.yaw - math.atan2((0 - self.pose.position.y), (0 - self.pose.position.x))))
                            if self.home_yaw > 180:
                                self.home_yaw = abs(self.home_yaw - 360)
                                self.random_turn_direction = LEFT
                            else:
                                self.random_turn_direction = RIGHT
                            self.command_state = Command_State.ACTING
                        else:
                            msg = Twist()
                            msg.angular.z = ANGULAR_VELOCITY * self.random_turn_direction
                            self.cmd_publisher.publish(msg)
                            yaw_difference = angles.normalize_angle(self.yaw - self.initial_yaw)
                            if math.fabs(yaw_difference) >= math.radians(self.home_yaw - 20):
                                self.main_state = State.SELECT
                                
                    case Ball_State.RANDOM_TURN:
                        #Part of random drive, the robot randomly turns in a direction
                        #If, while turning, it spots a valid item, it goes to the aim substate
                        #Otherwise, once the turn is complete, it goes to the drive substate
                        if self.command_state == Command_State.WAITING:
                            self.random_turn_angle = random.uniform(70, 150)
                            self.random_turn_direction = random.choice([LEFT, RIGHT])
                            self.initial_yaw = self.yaw
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.turn(self.random_turn_angle, self.random_turn_direction):
                                self.ball_state = Ball_State.RANDOM_DRIVE
                                self.command_state = Command_State.WAITING
                            elif self.item_selected:
                                self.ball_state = Ball_State.AIM
                                self.command_state = Command_State.WAITING

                    case Ball_State.RANDOM_DRIVE:
                        #The robot drives a random amount forward, then goes to the random turn state.
                        #If, while driving, it spots an item or picks up an item of the right colour, it goes to the aim state
                        if self.command_state == Command_State.WAITING:
                            self.random_walk_dist = (random.randint(3, 12) / 6)
                            self.previous_pose = self.pose
                            self.command_state = Command_State.ACTING                        
                        else: 
                            if self.item_selected or self.held_item.item_colour == self.target_colour:
                                self.home_state = Ball_State.AIM
                                self.command_state = Command_State.WAITING
                            elif self.drive(self.random_walk_dist):
                                self.ball_state = Ball_State.RANDOM_TURN
                                self.command_state = Command_State.WAITING
                    case _: 
                        pass





            case State.HOME: 
                match self.home_state:
                    case Home_State.SELECT:
                        #The home select substate determines how the robot should act in its journey home
                        self.nav_count += 1
                        if self.nav_count > 3:
                            #If the robot has tried to naviagte home 3 times and been interrupted 3 times, it does some random driving
                            #It will then try navigating again
                            #This is to avoid an infinite loop of the robot being unable to navigate from its current position, while also swapping items constantly.                      
                            self.home_state = Home_State.RANDOM_TURN
                        elif self.home_pose_accurate:
                            #If the robot has no reason to believe the home pose is innacurate, then it will go to the navigate pose and use nav2 to get home
                            self.home_pose.header.stamp = self.get_clock().now().to_msg()
                            self.navigator.goToPose(self.home_pose)
                            self.start_time = self.get_clock().now()
                            self.home_state = Home_State.NAVIGATE
                        elif (self.get_clock().now() - self.home_timer) > Duration(seconds = 10):
                            #If the robot can't use nav2 to get home, and it has spent more than 15 seconds looking for home, it will randomly drive around
                            self.home_state = Home_State.RANDOM_DRIVE
                            self.home_timer = self.get_clock().now()
                        else:
                            #If nav2 can't get the robot home, then the robot will spin to look for the homezone
                            self.home_state = Home_State.SPIN
                            self.home_timer = self.get_clock().now()
                            
                        
                    case Home_State.NAVIGATE:
                        #self.get_logger().info(f"{self.held_item, self.prev_held_item}")
                        #In this substate, the robot uses nav2 to navigate home
                        #If the robot accidentally swaps items while returning home, it will enter the item_swap substate
                        if (self.held_item.item_colour != self.target_colour 
                            and self.prev_held_item.item_colour == self.target_colour and self.held_item.holding_item):
                            self.navigator.cancelTask()
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.home_state = Home_State.ITEM_SWAP
                            self.start_time = self.get_clock().now()
                            return
                        
                        if not self.navigator.isTaskComplete():
                            feedback = self.navigator.getFeedback()
                            
                            if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 20): #default 45
                                self.get_logger().info(f"Navigation took too long... cancelling")
                                self.navigator.cancelTask()

                            if (not self.held_item.holding_item and self.target_item.diameter > BALL_OVERRIDE_DIAMETER):
                                self.navigator.cancelTask()
                                self.main_state = State.SELECT
                        
                        else:
                            #When the robot has finished navigating, there 3 possible results
                            #If the navigation was cancelled for taking too long, or failed, then the robot assumes the home pose is innaccurate and resorts to other methods of getting home
                            #If the navigation succeeded but the robot is still holding an item, then the home pose is no longer accurate so the robot resorts to other methods of getting home
                            #If the robot successfully gets home and delivers its item, then the robot goes back to the main select state.
                            result = self.navigator.getResult()
                            match result:
                                case TaskResult.SUCCEEDED:
                                    if self.held_item.item_colour == self.target_colour:
                                        self.home_pose_accurate = False
                                        self.home_state = Home_State.SPIN
                                    else:
                                        self.main_state = State.SELECT
                                        self.nav_count = 0

                                case TaskResult.CANCELED:
                                    self.home_pose_accurate = False
                                    self.home_state = Home_State.SPIN

                                case TaskResult.FAILED:
                                    self.home_pose_accurate = False
                                    self.home_state = Home_State.SELECT

                                case _:
                                    pass

                    case Home_State.SPIN:
                        #This substate is used when nav2 is no longer trustworthy to get the robot home.
                        #The robot spins 360 degrees until it can see the homezone, then it goes to the aim substate
                        #If it doesn't see the homezone, then it goes to random drive substate
                        if self.command_state == Command_State.WAITING:
                            self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                            self.spin_count = 0
                            self.command_state = Command_State.ACTING
                        else:
                            if self.navigator.isTaskComplete():
                                if self.spin_count > 0:
                                    self.home_state = Home_State.RANDOM_DRIVE
                                    self.command_state = Command_State.WAITING
                                else:
                                    self.spin_count += 1
                                    self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                            elif self.home_zone.visible:
                                self.navigator.cancelTask()
                                self.command_state = Command_State.WAITING
                                self.home_state = Home_State.AIM
                                self.aim_timer = self.get_clock().now()
                                msg = Twist()
                                self.cmd_publisher.publish(msg)


                    case Home_State.AIM:
                        #In this substate, the robot aims itself at the homezone,
                        #Once it is accurate enough, or it has spent too long aiming, the robot goes to the drive substate
                        #If it loses sight of the homezone, the robot goes into the random drive substate to avoid infinite loops        
                        msg = Twist()
                        msg.angular.z = ANGULAR_VELOCITY * (self.home_zone.x / 320)
                        self.cmd_publisher.publish(msg)
                        if ((self.home_zone.x < 5 and self.home_zone.x > -5) 
                            or (self.get_clock().now() - self.aim_timer) > Duration(seconds = 1)):
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.home_state = Home_State.DRIVE
                        if not self.home_zone.visible:
                            self.home_state = Home_State.RANDOM_DRIVE

                    case Home_State.DRIVE:
                        #Once the homezone has been spotted and aimed at, the robot begins driving towards it 
                        #If, while driving, it swaps items with another colour, it enter the item_swap substate
                        #If the robot is no longer holding an item, it assumes it reached the homezone and goes to the main select state.
                        if (self.held_item.item_colour != self.target_colour 
                            and self.prev_held_item.item_colour == self.target_colour and self.held_item.holding_item):
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.home_state = Home_State.ITEM_SWAP
                            self.start_time = self.get_clock().now()
                            return
                        
                        msg = Twist()
                        msg.linear.x = LINEAR_VELOCITY
                        msg.angular.z = ANGULAR_VELOCITY * (self.home_zone.x / 320)
                        self.cmd_publisher.publish(msg)
                        if not self.held_item.holding_item:
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.main_state = State.SELECT
                        elif not self.home_zone.visible:
                            self.home_state = Home_State.SPIN
                            self.command_state = Command_State.WAITING

                    case Home_State.ITEM_SWAP:
                        #If the robot enters the item swap substate, it remains still for 5 seconds, or until it has swapped back to the correct item
                        #It then goes back to the ball select substate
                        msg = Twist()
                        self.cmd_publisher.publish(msg)
                        if ((self.get_clock().now() - self.start_time) > Duration(seconds = 5)
                            or self.held_item.item_colour == self.target_colour):
                            self.main_state = State.SELECT
                            self.command_state = Command_State.WAITING

                    case Home_State.RANDOM_TURN:
                        #Part of the random drive system
                        #The robot makes a random turn, then goes to the random drive sub state once the turn is complete
                        #If it swaps items while turning, the robot goes to the item swap substate
                        if (self.held_item.item_colour != self.target_colour 
                            and self.prev_held_item.item_colour == self.target_colour and self.held_item.holding_item):
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.home_state = Home_State.ITEM_SWAP
                            self.start_time = self.get_clock().now()
                            return
                        
                        if self.command_state == Command_State.WAITING:
                            self.random_turn_angle = random.uniform(70, 150)
                            self.random_turn_direction = random.choice([LEFT, RIGHT])
                            self.initial_yaw = self.yaw
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.turn(self.random_turn_angle, self.random_turn_direction):
                                self.home_state = Home_State.RANDOM_DRIVE
                                self.command_state = Command_State.WAITING
                            

                    case Home_State.RANDOM_DRIVE:
                        #The robot drives forward a random distance
                        #If, while driving, it stops and enters the item swap substate
                        #If the robot can see home, and there have not been 3 failed navigation attempts, then it goes to the aim substate
                        #If the robot has had 3 failed navigation attempts, then once the drive is complete, it goes back to the home select substate
                        #Otherwise, the robot goes to the random turn substate
                        if (self.held_item.item_colour != self.target_colour 
                            and self.prev_held_item.item_colour == self.target_colour):
                            msg = Twist()
                            self.cmd_publisher.publish(msg)
                            self.home_state = Home_State.ITEM_SWAP
                            self.start_time = self.get_clock().now()
                            return
                        
                        if self.command_state == Command_State.WAITING:
                            self.random_walk_dist = (random.randint(3, 10) / 10)
                            self.previous_pose = self.pose
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.home_zone.visible and not self.nav_count > 3: 
                                self.home_state = Home_State.AIM
                                self.aim_timer = self.get_clock().now()
                                self.command_state = Command_State.WAITING
                            elif self.drive(self.random_walk_dist):
                                if self.nav_count > 3:
                                    self.nav_count = 0
                                    self.home_state = Home_State.SELECT
                                    self.command_state = Command_State.WAITING
                                else:
                                    self.home_state = Home_State.RANDOM_TURN
                                    self.command_state = Command_State.WAITING





            case State.LOCALISE: 
                #In this avoidance state, the robot will randomly drive around until it is confident in its position
                #It will only leave when it is confident in its position, or it is interrupted by a more important avoidance state
                match self.localise_state:
                    case Localise_State.DRIVE:
                        if self.command_state == Command_State.WAITING:
                            self.random_walk_dist = (random.randint(3, 10) / 10)
                            self.previous_pose = self.pose
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.location_confident:
                                self.exit_evasion()
                            elif self.drive(self.random_walk_dist):
                                self.localise_state = Localise_State.TURN


                    case Localise_State.TURN:
                        if self.command_state == Command_State.WAITING:
                            self.random_turn_angle = random.uniform(30, 90)
                            self.random_turn_direction = random.choice([LEFT, RIGHT])
                            self.initial_yaw = self.yaw
                            self.command_state = Command_State.ACTING
                        else: 
                            if self.location_confident:
                                self.exit_evasion()
                            elif self.turn(self.random_turn_angle, self.random_turn_direction):
                                self.localise_state = Localise_State.DRIVE
                    case _:
                        pass






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
                        #The robot saves its current yaw, and calculates which direction and how far it has to turn to face away from the threat
                        #It then goes to the TURN substate to make the required turn
                        self.initial_yaw = self.yaw
                        self.escape_angle = self.get_angle(math.degrees(self.yaw - self.target_yaw) + 180) #gets the target angle
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
