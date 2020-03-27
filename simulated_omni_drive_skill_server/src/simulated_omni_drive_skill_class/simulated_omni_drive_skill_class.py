import rospy
import actionlib
from simulated_omni_drive_skill_msgs.msg import SimulatedOmniDriveSkillAction, SimulatedOmniDriveSkillResult, SimulatedOmniDriveSkillFeedback
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from numpy import abs, sign, arctan2, arctan, sqrt
from tf.transformations import euler_from_quaternion
from math import atan2, cos, sin

# Define angle and position error thresholds
MAX_ETF = 0.005
MAX_EDF = 0.005

# Simulates omni-drive publishing a Twist message to
# a /cmd_vel topic
class SimulatedOmniDriveSkill(object):

    def __init__(self, action_name='SimulatedOmniDriveSkill'):

        # Standard skill generation initialization
        self.action_name = action_name
        self.simulated_omni_drive_skill_server = actionlib.SimpleActionServer(self.action_name, SimulatedOmniDriveSkillAction, self.execute_skill, False)
        self.simulated_omni_drive_skill_server.start()
        self.percentage = 0
        self.outcomes = ["succeeded", "aborted", "preempted"]

        # Waits for /gazebo/get_model_state to be ready and creates a ServiceProxy
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Publisher for the robot speed
        self.pub_vel = rospy.Publisher('friday/cmd_vel', Twist, queue_size=10)

        # ROS rate (10 Hz)
        self.rate = rospy.Rate(10)

    # Calculates the error between the robot angle and the angle between
    # the final position and current position
    def theta_error(self, goal, current_pos, robot_angle):

        theta_f = atan2(goal.x - current_pos.pose.position.x, goal.y - current_pos.pose.position.y)
        return theta_f + robot_angle

    # Calculates the error between the current robot angle and
    # the desired robot angle
    def theta_error_final(self, robot_angle, goal):

        return robot_angle - goal.ori

    # Calculates the error between the robot current position
    # and the desired final position
    def pos_error(self, goal, current_pos):

        return sqrt((goal.x - current_pos.pose.position.x)*(goal.x - current_pos.pose.position.x) + (goal.y - current_pos.pose.position.y)*(goal.y - current_pos.pose.position.y))

    # Moves the robot to the desired pose based on the angle
    def gotoxytheta_omni(self, vel, goal, current_pos):

        # Initizalizes useful variables
        finished = False

        # When 0, neither position or orientation are correct
        # When 1, one of them is correct and the other not
        # When 2, both are correct and skill is finished
        cont = 0

        # Converts robot position from quaternions to euler angles
        quat = [current_pos.pose.orientation.x, current_pos.pose.orientation.y, current_pos.pose.orientation.z, current_pos.pose.orientation.w]
        robot_angle = euler_from_quaternion(quat)[2]

        # Calculates angle and position errors
        theta_error = self.theta_error(goal, current_pos, robot_angle)
        theta_error_final = self.theta_error_final(robot_angle, goal)
        pos_error = self.pos_error(goal, current_pos)

        # Check final angle error and if it's higher than threshold
        # converge rotation to error zero, else stop rotating
        if abs(theta_error_final) > MAX_ETF:
            vel.angular.z = goal.speed * -sign(theta_error_final)
        else:
            vel.angular.z = 0
            cont+=1

        # Check position error and if it's higher than threshold
        # calculated desired linear speed, else stop 
        if pos_error > MAX_EDF:
            vel.linear.x = goal.speed * sin(theta_error)
            vel.linear.y = goal.speed * cos(theta_error)
        else:
            vel.linear.x = 0
            vel.linear.y = 0
            cont+=1

        # If both position and orientation are correct set finished flag to true
        if cont == 2:
            finished = True

        return vel, finished

    def execute_skill(self, goal):
        '''
        The execution of the skill should be coded here.
        In order to save you time, the methods check_preemption(), feedback(), success() and aborted() should be used.
        The check_preemption() method should be called periodically.
        The variable self.percentage should be updated when there is an evolution in the execution of the skill.
        feedback() method should be called when there is an evolution in the execution of the skill.
        '''

        # Initialize finished flag and Twist message
        vel = Twist()
        finished = False

        # Execute until flag is true
        while not finished:

            # Get current pose from Gazebo
            current_pos = self.get_model_state(model_name="friday")

            # Execute main method and publish calculated vel
            vel, finished = self.gotoxytheta_omni(vel, goal, current_pos)
            self.pub_vel.publish(vel)
        
        rospy.sleep(0.5) # wait a short time to guarantee the UR10 is stable
        self.success()

    def feedback(self, status=None):
        feedback = SimulatedOmniDriveSkillFeedback()
        feedback.percentage = self.percentage
        feedback.skillStatus = status if status else 'SimulatedOmniDriveSkill Executing'
        self.simulated_omni_drive_skill_server.publish_feedback(feedback)
        self.log_info(feedback)

    def success(self, status=None, outcome='succeeded'):
        result_status = status if status else 'SimulatedOmniDriveSkill executed successfully'
        result = self.result_constructor(percentage=100, status=result_status, outcome=outcome)
        self.simulated_omni_drive_skill_server.set_succeeded(result, result.skillStatus)

    def aborted(self, status=None, outcome='aborted'):
        result_status = status if status else 'SimulatedOmniDriveSkill aborted'
        result = self.result_constructor(status=result_status, outcome=outcome)
        self.simulated_omni_drive_skill_server.set_aborted(result, result.skillStatus)

    def check_preemption(self):
        if self.simulated_omni_drive_skill_server.is_preempt_requested():
            result = self.result_constructor(status='SimulatedOmniDriveSkill Preempted', outcome='preempted')
            self.simulated_omni_drive_skill_server.set_preempted(result, result.skillStatus)
            return

    def result_constructor(self, status, percentage=None, outcome=None):
        result = SimulatedOmniDriveSkillResult()
        result.percentage = percentage if percentage else self.percentage
        result.skillStatus = status
        result.outcome = outcome
        self.log_info(result)
        return result

    @staticmethod
    def log_info(status):
        info = '[SimulatedOmniDriveSkill] Percentage: ' + str(status.percentage) + '%. Status: ' + status.skillStatus
        rospy.loginfo(info)
        return info
