#!/usr/bin/env python

import rospy
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped

class Movement():

    def __init__(self):
        rospy.init_node("speed_controller")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0

        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)

        self.twist = Twist()

        self.r = rospy.Rate(4)

        xgoal = input("Enter goal for x: ")
        ygoal = input("Enter goal for y: ")

        self.goal = Point()
        self.goal.x = xgoal
        self.goal.y = ygoal

        self.move()

    def newOdom(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.q_rotation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) =euler_from_quaternion([self.q_rotation.x, self.q_rotation.y, self.q_rotation.z, self.q_rotation.w])

    def move(self):
        
        while not rospy.is_shutdown():

            self.dist_x = self.goal.x - self.x
            self.dist_y = self.goal.y - self.y

            self.measured_Angle = atan2(self.dist_y,self.dist_x)

            if(abs(self.measured_Angle - self.theta) > 0.1):
                self.twist.linear.x = 0.0
                self.twist.angular.z= 0.2
            
            else:
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.0
            
            self.publisher.publish(self.twist)
            self.r.sleep()

if __name__ == "__main__":
    Movement()




























                







































