import rclpy
import sys
import argparse
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
import random
from enum import Enum

from solution_interfaces.msg import RobotAssignment, RobotAssignments, ItemSize, RobotTargets

NULL=0 #Null is used to determine if a robot can't see any of an item 
NUM_ROBOTS = 3 #The number of robots in the scenario
NUM_COLOURS = 3 #The number of possible item colours in the scenario
COLOURS = ["RED", "GREEN", "BLUE"] #A list containing all possible colours in the scenario

class Mode(Enum):
    #These are the 4 possible modes for assignment, descriptions included further down
    CYCLICAL = 0
    BEST = 1
    RANDOM = 2
    SINGLE = 3

class RobotAssigner(Node):
    #RobotAssigner assigns the target colour of each robot, based on what the robots can see and what algorithm is selected

    def __init__(self, args):
        super().__init__('robot_targets')

        #To change assignment mode, change these values
        self.mode = Mode.CYCLICAL #The algorithm used for assigning colours to robots
        self.repeat_colours = False #Whether colours can be repeated, overridden in some modes
        self.single_colour = "BLUE" #When using single colour mode, this determines what colour to target

        """
        #Failed attempt at adding launch arguments to make experiments easier
        #If it had worked, it would be possible to change mode, repeat colours, and single target colour from the launch command
        
        parser = argparse.ArgumentParser()
        group = parser.add_argument_group()
        group.add_argument('--mode', type=str, metavar='MODE', help='Assignment mode')
        group.add_argument('--colour', type=str, metavar='COLOUR', help='Target colour')
        group.add_argument('--repeat', type=bool, metavar='REPEAT', help='Repeat colours')
        self.args = parser.parse_args(args[1:])

        self.get_logger().info(f"{self.args.mode, self.args.colour, self.args.repeat}")

        mode = self.args.mode
        mode = mode.upper()

        colour = self.args.colour
        colour = colour.upper()

        repeat = self.args.repeat

        match mode:
            case "CYCLICAL":
                self.mode = Mode.CYCLICAL
            case "BEST":
                self.mode = Mode.BEST
            case "SINGLE":
                self.mode = Mode.SINGLE
            case "RANDOM":
                self.mode = Mode.RANDOM
            case _:
                self.mode = Mode.CYCLICAL
                self.get_logger().info("Error in inputted mode, defaulting to cyclical")

        match colour:
            case "RED":
                self.single_colour = "RED"
            case "GREEN":
                self.single_colour = "GREEN"
            case "BLUE":
                self.single_colour = "BLUE"
            case _:
                self.single_colour = "RED"
                self.get_logger().info("Error in inputted colour, defaulting to Red")
        
        match repeat:
            case True:
                self.repeat_colours = True
            case False:
                self.repeat_colours = False
            case _:
                self.repeat_colours = False
                self.get_logger().info("Error in inputted repeat, defaulting to False")

        self.get_logger().info(f"Mode: {self.mode}, Colour: {self.single_colour}, Repeat: {self.repeat_colours}")
        """


        #/robot_assignments is where the list of assignments is published, once the chosen assignment algorithm is complete
        self.target_publisher = self.create_publisher(RobotAssignments, '/robot_assignments', 10)

        #A subscription is created to the targets_list node of every robot in the scenario, based on NUM_ROBOTS
        self.robot_subscriptions = []
        for i in range(NUM_ROBOTS):
            namespace = 'robot' + str(i + 1) + '/targets_list'
            self.robot_subscriptions.append(self.create_subscription(RobotTargets, namespace, self.rbt_target_callback, 10))


        self.colours = COLOURS #self.colours is a list of every colour in the scenario
        self.assigned_colours = [] #self.assigned_colours is a list of every colour that has been assigned so far

        #If in the BEST mode, repeat colours is required
        #While not possible in the assessment, there is a check for if the number of robots is greater than number of colours,
        #as repeat colours is required in that scenario to allow all robots to get an assignment
        if NUM_ROBOTS > NUM_COLOURS or self.mode == Mode.BEST:
            self.repeat_colours = True

        #self.robot_assignments will contain a list of all robots and their assigned colours
        self.robot_assignments = RobotAssignments() 

        self.robot_targets = [] #Robot targets will contain a list of robots, and the closest item of each colour they can see
        self.unassigned_robots = [] #Unassigned robots contains a list of all robots that have not been assigned a colour yet
        for i in range(NUM_ROBOTS):
            id = 'robot' + str(i+1)
            self.unassigned_robots.append(id)
            rbt_target = RobotTargets()
            rbt_target.robot_id = id
            rbt_target.ready = False
            for i in range(NUM_COLOURS):
                item = ItemSize()
                item.colour = ''
                item.size = NULL
                rbt_target.data.append(item)

            self.robot_targets.append(rbt_target)

        self.complete = False #self.complete marks when the assignment has successfully run, to stop it being reran


        timer_period = 2  # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def rbt_target_callback(self, msg):
        #This callback function takes the informaton on what the robots can see with their camera and saves it
        #The information included is what items can see and how big they appear
        for i in range(NUM_ROBOTS):
            if self.robot_targets[i].robot_id == msg.robot_id:
                self.robot_targets[i] = msg

    def assign_robot(self, robot, colour):
        #assign_robot creates the assignment message of a robot and appends it to the robot_assignments list
        #the robot is also removed from the unassigned_robots list so it can't be given a new assignment
        self.unassigned_robots.remove(robot)
        assignment = RobotAssignment()
        assignment.robot_id = robot
        assignment.colour = colour
        self.robot_assignments.data.append(assignment)


    def cyclical_colour_assignment(self, colour):
        #The assignment algorithm used when in CYCLICAL mode

        #A dictionary is created containing a robot ID and the provided diameter of the colour being checked
        robot_item_count = {}
        for eachRobot in self.robot_targets:
            if eachRobot.robot_id in self.unassigned_robots:
                for eachColour in eachRobot.data:
                    if eachColour.colour == colour:
                        robot_item_count[eachRobot.robot_id] = eachColour.size

        #Each value in the dictionary is compared to find which robot is closest to an item of the correct colour
        max = NULL
        for eachRobot in robot_item_count:
            if robot_item_count[eachRobot] > max:
                robot = eachRobot
                max = robot_item_count[eachRobot]

        #If no robots can see an item of the right colour, this if statement will activate, stopping assignments being made
        if max == NULL:
            return False

        #If a robot is chosen, the colour and robot are sent to assign_robot to be saved as an assignment
        self.assign_robot(robot, colour)
        return True



    def best_colour_assignment(self, robot):
        #This is the assignment algorithm for BEST

        #The diameter of every colour the robot can see if compared,
        #the largest diameter is then used to determine which colour the robot will be assigned
        max = NULL
        colour = ""
        for eachRobot in self.robot_targets:
            if robot == eachRobot.robot_id:
                for eachColour in eachRobot.data:
                    if eachColour.size > max:
                        max = eachColour.size
                        colour = eachColour.colour

        #If the robot can see no items, it is not given an assignment here
        if max == NULL:
            return
        #Otherwise, the robot and chosen colour are given to assign_robots to be saved.
        self.assign_robot(robot, colour)


        
        



    def timer_callback(self):
        if self.complete:
            self.target_publisher.publish(self.robot_assignments)
            #This if statement traps the node in an infinite loop once it is has successfully published assignments
            #This stops the assignments being recalculated as the robots move
            return

        #When a robot publishes the information from its camera, it includes a boolean to confirm it is ready to recieve its assignment
        #This counter is used to count how many of the robots have confirmed they are ready
        #The readiness check is to ensure the assignment process doesn't start before all items are spawned and the robots are ready
        counter = 0
        for eachRobot in self.robot_targets:
            if eachRobot.ready:
                counter += 1
        
        if counter != NUM_ROBOTS:
            #When every robot has checked in, the assignment process can progress
            return
        
        attempts = 0 #Attempts records how many assignment attempts have been made, so the program doesn't get stuck
        #If attempts exceeds a set threshhold, usually NUM_ROBOTS, then the RANDOM mode is used for the remaining robots
        while len(self.unassigned_robots) > 0:
            #The program is trapped in a while loop until every robot is assigned an item


            match self.mode:
                case Mode.CYCLICAL:
                    #The CYCLICAL mode cycles through each available colour, assigning the colour to the best robot available on that pass
                    #If there are more robots than colours, then multiple passes will be required
                    #It is also based on the order of the COLOUR's list, so changing that list can effect how colours are assigned
                    #It is currently set to cycle from closest to furthest (Red, then Green, then Blue)
                    for i in range(len(self.colours)):
                        if self.cyclical_colour_assignment(self.colours[i]):
                            self.assigned_colours.append(self.colours[i])

                    #If repeat colours is set to false, then colours are removed from the list of available list after assignment
                    if not self.repeat_colours:
                        for eachColour in self.assigned_colours:
                            if eachColour in self.colours:
                                self.colours.remove(eachColour)

                    #If too many attempts fail, then the program swaps to RANDOM assignment
                    #This should only occur if the remaining robots can't see any of the remaining colours
                    if attempts > (NUM_ROBOTS + 1):
                        self.mode = Mode.RANDOM

                    attempts += 1


                case Mode.BEST:
                    #In the BEST mode, robots are assigned the best colour available to them, based on what they can see at the start
                    #Best in this case, means closest.
                    #Multiple robots can target one colour, if they both see that colour item as the closest option
                    for eachRobot in self.unassigned_robots:
                        self.best_colour_assignment(eachRobot)
                    
                    if attempts > (NUM_ROBOTS + 1):
                        self.mode = Mode.RANDOM
                    
                    attempts += 1


                case Mode.RANDOM:
                    #The RANDOM mode simply assigns a random colour to each robot with no logic
                    #If repeat colours are allowed, it can assign multiple robots the same colour
                    for eachRobot in self.unassigned_robots:
                        colour = random.choice(self.colours)
                        if not self.repeat_colours:
                            self.colours.remove(colour)
                        self.assign_robot(eachRobot, colour)


                case Mode.SINGLE:
                    #The SINGLE mode assigns one colour to all robots
                    for eachRobot in self.unassigned_robots:
                        self.assign_robot(eachRobot, self.single_colour)




        #Once all assignments are decided, they can be published and the algorithm can be marked as complete
        self.target_publisher.publish(self.robot_assignments)
        self.complete = True


        #A disabled log statement for logging what assignments each robot has been given
        log = ""
        for i in range(NUM_ROBOTS):
            log = log + self.robot_assignments.data[i].robot_id + ': ' + self.robot_assignments.data[i].colour + " | "
        #self.get_logger().info(f"{log}")




def main(args=sys.argv):
    rclpy.init(args=args, signal_handler_options = SignalHandlerOptions.NO)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    robot_targets = RobotAssigner(args_without_ros)


    try:
        rclpy.spin(robot_targets)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        robot_targets.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()