import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# Angles in radians
def joint_publisher(joint_1_angle, joint_2_angle, joint_3_angle, joint_4_angle, joint_5_angle):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    duration = 2
    start_time = time.time()
    while time.time() - start_time < duration: 
        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions = [joint_1_angle, joint_2_angle, joint_3_angle, joint_4_angle, joint_5_angle]    
        point.time_from_start = rospy.Duration(1)
        state.points.append(point)
        pub.publish(state)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass