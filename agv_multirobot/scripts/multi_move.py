#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Twist


vel1 = Twist()
vel2 = Twist()
vel3 = Twist()


def main():
    
    pub_cmdvel1 = rospy.Publisher('/AGV_OTA1/cmd_vel', Twist, queue_size=10)
    pub_cmdvel2 = rospy.Publisher('/AGV_OTA2/cmd_vel', Twist, queue_size=10)
    pub_cmdvel3 = rospy.Publisher('/AGV_OTA3/cmd_vel', Twist, queue_size=10)
    
    while not rospy.is_shutdown():

        vel1.linear.x = 0.5
        vel1.angular.z = 0.3
        vel2.linear.x = 0.5
        vel2.angular.z = -0.3
        vel3.linear.x = 0.5

        pub_cmdvel1.publish(vel1)
        pub_cmdvel2.publish(vel2)
        pub_cmdvel3.publish(vel3)
            
        r.sleep()


if __name__ == '__main__':
    
    rospy.init_node('multi_move_node')
    
    r = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():
        main()
        r.sleep()
        