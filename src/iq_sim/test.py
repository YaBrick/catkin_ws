import rospy
from std_msgs.msg import String

def hello_world_pub():
    rospy.init_node("hello_world_pub_node")
    pub = rospy.Publisher("hello_world", String, queue_size=10)

    i = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        pub.publish(f"Hello World text {i}")
        i += 1   
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_world_pub()
    except rospy.ROSInterruptException:
        pass
