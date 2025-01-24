import rclpy
import sys
import time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import random

from assessment_interfaces.msg import ItemList, Item

from solution_interfaces.msg import ItemSize, RobotTargets, RobotAssignment, RobotAssignments, TrackedItem 

NULL=0 #A default for the size of an item, meaning an item can't be seen

class ItemTracker(Node):
    #ItemTracker uses the information from the 'items' topic to identify the best target for the robot, and relay it to the main controller
    #It also provides a list of the closest item of each colour to 'robot_targets' so it can correctly assign targets to each robot,
    #And uses the information by that node to determine which items are valid targets for this robot

    def __init__(self):
        super().__init__('item_tracker')

        #Gets the namespace of the node, to get the robot's ID
        name = self.get_namespace()
        self.rbt_id = name.replace('/', '')


        time.sleep(5)#A delay to make sure all items have spawned in

        #Items provides a list containing all the items the camera can identify, their diameter in pixels, and their colour
        self.item_spotter_subscriber = self.create_subscription(
            ItemList, 'items', self.item_spotter_callback, 10)

        #/robot_assignments contains a list, which associates a robot ID with a colour, so a robot knows which colour to target
        self.robot_targets_subscriber = self.create_subscription(
            RobotAssignments, '/robot_assignments', self.robot_target_callback, 10)
        
        #The rbt_target publisher is used to send what items the robot can see when it spawns, and how far away they are
        #This information is used by robot_targets to determine what colour item each robot should target
        self.rbt_target_publisher = self.create_publisher(RobotTargets, 'targets_list', 10)

        #target_item publishes the closest item the camera can see, of the correct colour,
        #so the robot controller can act on the information included
        self.rbt_item_publisher = self.create_publisher(TrackedItem, 'target_item', 10)

        #target_colour is the colour of the items the robot should be trying to pick up
        self.target_colour = "RED"

        #item_assigned flags whether the robot has been assigned a colour,
        #or whether it should keep publishing the items it can see to robot_targets
        self.item_assigned = False
        

    
    def item_spotter_callback(self, msg):
        #The callback function for the items topic, updates the best target item when called, and whether a valid target is visible
        target_item = Item()
        item_selected = TrackedItem()
        item_selected.visible = False
        for eachItem in msg.data:
            #Filters for target colour, whether the ball is being carried by a robot, and tries to pick the largest item in the camera.
            #Items above y = 6 are also discarded, as they are being carried by other robots
            if eachItem.colour == self.target_colour and eachItem.diameter > target_item.diameter and eachItem.y < 6:
                target_item = eachItem
                item_selected.visible = True
                item_selected.item = target_item
        
        self.rbt_item_publisher.publish(item_selected)

        #One used when the robot has not been assigned a target item colour
        #Provides a list to the robot_targets server, containing the diameter of the closest item of each colour
        #Values are defaulted to NULL, representing no item seen
        #Once the robot has been assigned a target item, this part of the function will not be called
        if not self.item_assigned:
            robot_targets = RobotTargets()
            robot_targets.robot_id = self.rbt_id
            robot_targets.ready = True
            item_counts = {"RED": NULL, "BLUE": NULL, "GREEN": NULL}
            for eachItem in msg.data:
                if item_counts[eachItem.colour] < eachItem.diameter:
                    item_counts[eachItem.colour] = eachItem.diameter
                
            for eachColour in item_counts:
                colour_count = ItemSize()
                colour_count.colour = eachColour
                colour_count.size = item_counts[eachColour]
                robot_targets.data.append(colour_count)
            #self.get_logger().info(f"{robot_targets}")
            self.rbt_target_publisher.publish(robot_targets)


    def robot_target_callback(self, msg):
        #This callback function gets the assignments of each robot from robot_targets,
        #The robots ID is matched to an ID in the list, and the associated target colour is saved
        for eachRobot in msg.data:
            if eachRobot.robot_id == self.rbt_id:
                self.target_colour = eachRobot.colour
                self.item_assigned = True
                #self.get_logger().info(f"Item Assigned: {self.target_colour}")


def main(args=None):
    rclpy.init(args=args)

    robot_targets = ItemTracker()


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