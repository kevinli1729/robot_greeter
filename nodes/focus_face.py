#!/usr/bin/env python
import rospy
import controlit_goal_generators
from std_msgs.msg import Float64MultiArray

def focus_face():
    pub = rospy.Publisher("/dreamer_controller/neck/goal_position", Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    mygenerator = SineWaveGenerator.SineWaveGenerator(
        rosTopic = "/dreamer_controller/neck/eye_goal_position", amplitude = 0.59, offset = 0, initGoal = 0, numDoFs = 4, jointIndex = 2, period = 20, updateFreq = 10)
    mygenerator.start()
    #rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq

if __name__ == '__main__':
    try:
        focus_face()
    except rospy.ROSInterruptException:
        pass