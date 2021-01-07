#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

data = Image()
odom = Odometry()
vel = Twist()


def camera_callback(data):

    pub_camera = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    pub_camera.publish(data)

def odom_callback(odom):
    
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
    pub_odom.publish(odom)

def cmd_vel_callback(vel):
    
    pub_cmdvel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_cmdvel.publish(vel)

def main():
    

    sub_camera = rospy.Subscriber('/camera/image_raw', Image, camera_callback)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    sub_cmdvel = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('converter_node')
    
    r = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():
        main()
        r.sleep()
        