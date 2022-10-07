#!/usr/bin/env python
#My_Map.png Maze End:
# x = 15
# 9 < y < 10
# z: 0.0
import math
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class RobotController:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/robot0/laser_0", LaserScan, self.laser_callback)
        self.laser_msg = LaserScan()
        self.rate = rospy.Rate(10)
        self.no_of_rays = 667
        self.x_vec_sum = 0
        self.y_vec_sum = 0
        self.front_laser_index = 333
    def laser_callback(self, msg):
        self.laser_msg = msg
    def get_laser_range(self, pos):
        if self.laser_msg.ranges[pos] == float('inf'):
            return self.laser_msg.range_max
        else:
            return self.laser_msg.ranges[pos]
    def get_laser_angle(self, pos):
        return pos * self.laser_msg.angle_increment + self.laser_msg.angle_min
    def get_front_laser(self):
        return self.get_laser_range(self.front_laser_index)
    def get_vec_sums(self):
        for i in range(len(self.laser_msg.ranges)):
            theta = self.get_laser_angle(i)
            x_vec = self.get_laser_range(i) * math.cos(float(theta)) / self.laser_msg.range_max
            y_vec = self.get_laser_range(i) * math.sin(float(theta)) / self.laser_msg.range_max
            self.x_vec_sum += x_vec
            self.y_vec_sum += y_vec
        self.x_vec_sum = self.x_vec_sum / len(self.laser_msg.ranges)
        self.y_vec_sum = self.y_vec_sum / len(self.laser_msg.ranges)
    def safe_forward(self):
        self.laser_msg.angle_increment
        self.get_vec_sums()
        final_angle = math.atan2(self.y_vec_sum,self.x_vec_sum)
        cmdTwist = Twist()
        cmdTwist.linear.x = self.get_front_laser() / self.laser_msg.range_max / 5
        cmdTwist.angular.z = final_angle
        self.vel_pub.publish(cmdTwist)
    def main_loop(self):
        while not rospy.is_shutdown():
            self.safe_forward()
def main(args=None):
    rospy.init_node("Controller",anonymous=True)
    robot_controller = RobotController()
    time.sleep(1) #Sleep to allow time to initialize. Otherwise subscriber might recieve an empty message
    robot_controller.main_loop()

if __name__ == '__main__':
    main()