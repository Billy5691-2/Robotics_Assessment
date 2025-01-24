import rclpy
import sys
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from solution_interfaces.msg import RobotPosition, RobotPositions, CollisionWarning, CollisionWarnings

NUM_ROBOTS = 3 #The number of robots in the simulation
ROBOT_DISTANCE_WARNING = 0.9 #The distance at which robots will react to each other

class RobotCollisions(Node):
    #RobotCollisions checks if robots are at risk of crashing into each other, based on their coordinates,
    #And provides a warning and a yaw to move away from the threat, if a danger is identified

    def __init__(self):
        super().__init__('robot_collisions')

        #/robot_tracker provides a list of all the robots and their most recent coordinates.
        self.tracker_subscription = self.create_subscription(
            RobotPositions, '/robot_tracker', self.tracker_callback, 10)

        #/robot_collisions publishes a list containing the collision warnings for all robots, labelled with the robot ID
        self.crash_warning_publisher = self.create_publisher(
            CollisionWarnings, '/robot_collisions', 10)
        
    def tracker_callback(self, msg):
        #Called when /robot_tracker updates the coordinates for all the robots.
        #This callback calculates the distances between all the robots
        #If the distance between 2 robots is below the threshold,
        #the angle 180 degrees away from the nearest robot is provided to each robot in danger, so it may react accordingly

        #An empty list with no warnings is created for each robot
        robot_warnings = CollisionWarnings()
        for i in range(NUM_ROBOTS):
            warning = CollisionWarning()
            warning.robot_id = 'robot' + str(i + 1)
            warning.threat = False
            warning.angle = 0.0
            robot_warnings.data.append(warning)

        #Robots at risk are then calculated, any robots that are at risk have their warning flagged as true
        #and are given the yaw required to drive to safety
        robot_list = msg.data
        for i in range(NUM_ROBOTS):
            closest_distance = 1000
            x_one = robot_list[i].x
            y_one = robot_list[i].y
            for j in range(NUM_ROBOTS):
                if i != j:
                    x_two = robot_list[j].x
                    y_two = robot_list[j].y
                    x_diff = x_two - x_one
                    y_diff = y_two - y_one
                    distance = math.sqrt(x_diff**2 + y_diff**2)
                    if distance < ROBOT_DISTANCE_WARNING and distance < closest_distance:
                        self.get_logger().info(f"i: {i}, x {x_one:.3f}, y: {y_one:.3f}")
                        self.get_logger().info(f"j: {j}, x {x_two:.3f}, y: {y_two:.3f}")
                        robot_warnings.data[i].angle = math.atan2(y_diff, x_diff)
                        robot_warnings.data[i].threat = True
                        closest_distance = distance 
        self.crash_warning_publisher.publish(robot_warnings)
        


        

        



def main(args=None):
    rclpy.init(args=args)

    node = RobotCollisions()


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