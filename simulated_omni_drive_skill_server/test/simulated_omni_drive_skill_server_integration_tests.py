#!/usr/bin/env python

"""Tests for the SimulatedOmniDrive Skill Class"""

from unittest import TestCase
from mock import patch, MagicMock
import rospy
import actionlib
import time

from simulated_omni_drive_skill_msgs.msg import SimulatedOmniDriveSkillAction, SimulatedOmniDriveSkillResult, SimulatedOmniDriveSkillFeedback, SimulatedOmniDriveSkillGoal
from simulated_omni_drive_skill_server.simulated_omni_drive_skill_class import SimulatedOmniDriveSkill


class IntegrationTestSimulatedOmniDriveSkillServer(TestCase):
    """Test class for the SimulatedOmniDrive Skill Class"""

    @classmethod
    def setUpClass(cls):

        rospy.init_node('simulated_omni_drive_skill_class_test')

    def test_connection_to_server(self):

        client = actionlib.SimpleActionClient('SimulatedOmniDriveSkill', SimulatedOmniDriveSkillAction)
        result = client.wait_for_server()

        self.assertTrue(result)

    def test_execute_skill(self):

        goal = None

        client = actionlib.SimpleActionClient('SimulatedOmniDriveSkill', SimulatedOmniDriveSkillAction)
        client.wait_for_server()

        client.send_goal(goal)
        result = client.wait_for_result(rospy.Duration.from_sec(20.0))

        self.assertTrue(result)

if __name__ == '__main__':
    import rostest

    rostest.rosrun('simulated_omni_drive_skill_server', 'simulated_omni_drive_skill_server_integration_tests', IntegrationTestSimulatedOmniDriveSkillServer)