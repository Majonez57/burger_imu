#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class SimpleMover():
    def __init__(self):
        rospy.init_node('turtlebot3_move', anonymous=True)
        self.rate = rospy.Rate(10)
        self.movetopic = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.message = Twist()

    """
    Main Loop checks user sensor inputs, then publishes an appropriate twist command
    """
    def main(self):
        user_stop = False
        while not user_stop:
            self.movetopic.publish(self.message)
            self.rate.sleep()


    """
    The following functions are for testing only!
    """

    def move_forward(self, linear_speed=0.2, duration=1):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.movetopic.publish(move_cmd)
            self.rate.sleep()
        self.stop_robot()

    def move_backward(self, linear_speed=0.2, duration=1):
        move_cmd = Twist()
        move_cmd.linear.x = -linear_speed
        move_cmd.angular.z = 0.0
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.movetopic.publish(move_cmd)
            self.rate.sleep()
        self.stop_robot()

    def turn_left(self, angular_speed=0.2, duration=1):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = angular_speed
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.movetopic.publish(move_cmd)
            self.rate.sleep()
        self.stop_robot()

    def turn_right(self, angular_speed=0.2, duration=1):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = -angular_speed
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.movetopic.publish(move_cmd)
            self.rate.sleep()
        self.stop_robot()

    def stop_robot(self):
        move_cmd = Twist()
        self.movetopic.publish(move_cmd)

if __name__ == '__main__':
    simple_mover = SimpleMover()
    simple_mover.move_forward()
