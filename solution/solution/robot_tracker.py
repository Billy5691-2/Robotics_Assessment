import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from solution_interfaces.msg import RobotPosition, RobotPositions

NUM_ROBOTS = 3 #The number of robots in the scenario

class RobotTracker(Node):
    #RobotTracker consolidates the coordinates from all the robots
    #It's information is used by robot_collision_warning to determine which robots are too close together
    #As coordinates are provided by the robots, they can be incorrect if a robot loses track of its position in the world

    def __init__(self):
        super().__init__('robot_tracker')

        #/robot_tracker is where the list of all the robot's coordinates is published
        self.tracker_publisher = self.create_publisher(RobotPositions, '/robot_tracker', 10)

        #A subscription is created to every robot, to recieve their coordinates in the world
        self.robot_subscriptions = []
        for i in range(NUM_ROBOTS):
            namespace = 'robot' + str(i + 1) + '/position'
            self.robot_subscriptions.append(self.create_subscription(RobotPosition, namespace, self.rbt_callback, 10))

        #robot_positions contains a list of every robot and their position in the world
        self.robot_positions = RobotPositions()
        for i in range(NUM_ROBOTS):
            rbt_position = RobotPosition()
            rbt_position.robot_id = ('robot' + str(i+1))
            rbt_position.x = float(-3.5)
            rbt_position.y = float(2 - (2 * i)) 
            #Works for 3 robots, will have wrong positions for other amounts
            #Not a major issue as robots will update their position anyway
            #If points in the list are too close, they can cause issues at the beginning of the simulation
            self.robot_positions.data.append(rbt_position)

        #debug variables
        self.count = 0

        #Robot positions are published every 0.1 seconds
        timer_period = 0.1  # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def rbt_callback(self, msg):
        #When a robot publishes its position, this function is called.
        #The function matches the incoming position to the approriate robot,
        #then the position of that robot is updated in the rbt_position list
        for i in range(NUM_ROBOTS):
            if msg.robot_id == self.robot_positions.data[i].robot_id:
                self.robot_positions.data[i] = msg

    def timer_callback(self):
        #The full list is published so the robots can identify the positions of their fellow robots
        self.tracker_publisher.publish(self.robot_positions)


        #debug information on the positions of the robots
        if self.count > 20:
            #This log statement only works for 3 robots
            #log = "Rbt 1 x: " + str(round(self.robot_positions.data[0].x, 4)) + ". Rbt 2 x: " + str(round(self.robot_positions.data[1].x, 4)) + ". Rbt 3 x: " + str(round(self.robot_positions.data[2].x, 4))
            #log = log + ". \n Rbt 1 y: " + str(round(self.robot_positions.data[0].y, 4)) + ". Rbt 2 y: " + str(round(self.robot_positions.data[1].y, 4)) + ". Rbt 3 y: " + str(round(self.robot_positions.data[2].y, 4))
            #self.get_logger().info(f'Robot Positions: \n {log}')
            self.count = 0
        else:
            self.count += 1


def main(args=None):
    rclpy.init(args=args)
    robot_tracker = RobotTracker()
    try:
        rclpy.spin(robot_tracker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        robot_tracker.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()