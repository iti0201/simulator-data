import rospy
import geometry_msgs.msg
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
import math


class GazMedMotor:

    def __init__(self):
        self.terminated = False
        self.stopped = False
        self.duty_cycle_sp = None
        self.clear = True
        self.motor_counter = 0
        sub = rospy.Subscriber('lego_robot/sonar_joint_states', JointState, self.callback)
        sub2 = rospy.Subscriber('gazebo/model_states', ModelStates, self.modelcallback)
        self.current = 0
        self.parent_rot_speed = 0

    def callback(self, data):
        currentPos = data.position[0]
        if currentPos > 3.14:
            currentPos -= 6.28
        if currentPos < -3.14:
            currentPos += 6.28
        self.current = currentPos

    def modelcallback(self, data):
        index = data.name.index('lego_robot')
        self.parent_rot_speed = data.twist[index].angular.z

    def talker_abs_pos(self, position_sp, speed):
        self.motor_counter += 1
        current_counter = self.motor_counter

        pub = rospy.Publisher('lego_robot/sonar_wheel_pose', Pose, queue_size=10)

        publishFrequency = 25
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0
        pos_radians = -math.radians(position_sp)
        if pos_radians > 2.356:
            pos_radians = 2.356
        elif pos_radians < -2.356:
            pos_radians = -2.356
        
        while not rospy.is_shutdown() and not self.stopped:
            if self.current == 0:
                continue
            else:
                
                if math.fabs(self.current - pos_radians) < 0.05:
                    break
            
            if self.motor_counter != current_counter:
                return
            # print(counter, self.motor_counter)
            pose = Pose()
            position = geometry_msgs.msg.Vector3()
            # rotation = geometry_msgs.msg.Quaternion()
            position.x = math.radians(speed)
            # tf.transformations.quaternion_from_euler(0, 0, kwargs['position_sp'])
            position.y = self.parent_rot_speed
            
            counter += 1
            pose.position = position
            orientation = geometry_msgs.msg.Quaternion()
            orientation.z = pos_radians
            pose.orientation = orientation
            # rospy.loginfo(pose)
            pub.publish(pose)
            rate.sleep()

        pose = Pose()
        position = geometry_msgs.msg.Vector3()
        position.x = 0
        position.y = self.parent_rot_speed
        pose.position = position
        orientation = geometry_msgs.msg.Quaternion()
        orientation.z = pos_radians
        pose.orientation = orientation
        # rospy.loginfo(pose)
        pub.publish(pose)

    def activate_thread(self):

        self.stopped = False

    def stop_thread(self):
        # print('STOPPING')
        self.stopped = True
        self.clear = True

    def set_duty_cycle_sp(self, duty_cycle_sp):
        self.duty_cycle_sp = duty_cycle_sp
