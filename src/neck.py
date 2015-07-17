#!/usr/bin/env python
# license removed for brevity
import rospy
import controlit_goal_generators
import SineWaveGenerator
from std_msgs.msg import Float64MultiArray

def controller():
    #pub = rospy.Publisher("/dreamer_controller/neck/goal_position", Float64MultiArray, queue_size=10)
    rospy.init_node('neck', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    mygenerator = SineWaveGenerator.SineWaveGenerator(
        rosTopic = "/dreamer_controller/neck/goal_position", amplitude = 1.4, offset = 0, initGoal = 0, numDoFs = 4, jointIndex = 1, period = 20, updateFreq = 10)
    mygenerator.start()
    #rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass