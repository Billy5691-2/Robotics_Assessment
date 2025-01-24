import rclpy
import sys
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf_transformations import euler_from_quaternion


from solution_interfaces.msg import CrashCheck
from sensor_msgs.msg import Imu



class CrashDetection(Node):
    #CrashDetection looks at the IMU of a robot to see if it has crashed, based on its roll and pitch

    def __init__(self):
        super().__init__('robot_crash_detection')

        #The IMU topic contains information from the robot's IMU, which provides information on the robot's position and it's acceleration
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)

        #The crash_detection node publishes true if the robot is rolling or pitching, signalling a crash
        #Otherwise, it publishes false
        self.rbt_crash_detection_publisher = self.create_publisher(CrashCheck, 'crash_detection', 10)

        #Crash_counter is for data logging only
        self.rbt_crash_counter_publisher = self.create_publisher(CrashCheck, 'crash_counter', 10)
        #self.prev_crash stops crashes being counted twice
        self.prev_crash = False


    def imu_callback(self, msg):
        #When the IMU publishes data, this callback checks to see if any roll or pitch has been detected
        #Any movement over 3 degrees in either axis is taken as a sign of a crash, as the assessment world is flat
        crash = CrashCheck()
        crash.crashed = False

        data = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([data.x,
                                                    data.y,
                                                    data.z,
                                                    data.w])

        if abs(math.degrees(pitch)) > 3 or abs(math.degrees(roll)) > 3:
            crash.crashed = True


            #The two if statements below are for counting the number of crashes
            #They have no effect on the functionality of the crash detection for robots
            #This is an imperfect method of counting crashes as a crash can be counted multiple times in some cases
            #The severity of crashes is also not recorded
            if not self.prev_crash:
                self.rbt_crash_counter_publisher.publish(crash)
                self.prev_crash = True
        if not crash.crashed:
            self.prev_crash = False

        #A true or false is published to the crash_detection topic
        self.rbt_crash_detection_publisher.publish(crash)



def main(args=None):
    rclpy.init(args=args)

    crash_node = CrashDetection()


    try:
        rclpy.spin(crash_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        crash_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()