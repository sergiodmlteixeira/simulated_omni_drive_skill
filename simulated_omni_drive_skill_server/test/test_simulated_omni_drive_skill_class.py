#!/usr/bin/env python

"""Tests for the SimulatedOmniDriveSkill Class"""

from unittest import TestCase
from mock import patch, MagicMock
import rospy
import actionlib
import time

from simulated_omni_drive_skill_msgs.msg import SimulatedOmniDriveSkillAction, SimulatedOmniDriveSkillResult, SimulatedOmniDriveSkillFeedback, SimulatedOmniDriveSkillActionGoal, SimulatedOmniDriveSkillGoal
from simulated_omni_drive_skill_server.simulated_omni_drive_skill_class import SimulatedOmniDriveSkill

ACTION_NAME = 'SimulatedOmniDriveSkill'

class TestSimulatedOmniDriveSkillClass(TestCase):
    """Test class for the SimulatedOmniDrive Class"""

    @classmethod
    def setUpClass(cls):

        rospy.init_node('simulated_omni_drive_skill_class_test')

    def setUp(self):

	rospy.wait_for_service = MagicMock()
        actionlib.SimpleActionServer.start = MagicMock()
        self.test_instance = self.create_instance()

    def tearDown(self):

        self.test_instance.simulated_omni_drive_skill_server.__del__()

    def create_instance(self, action_name=ACTION_NAME):

        instance = SimulatedOmniDriveSkill(action_name=action_name)

        return instance

    def test_instance_creation(self):
        """Tests the creation of a class instance."""

        self.assertIsInstance(self.test_instance, SimulatedOmniDriveSkill)

    def test_instance_creation_action_name(self):

        self.assertEqual(self.test_instance.action_name, ACTION_NAME)

    def test_instance_creation_action_server(self):

        self.assertIsInstance(self.test_instance.simulated_omni_drive_skill_server, actionlib.SimpleActionServer)
        self.assertEqual(self.test_instance.simulated_omni_drive_skill_server.execute_callback, self.test_instance.execute_skill)

    def test_instance_creation_percentage(self):

        self.assertEqual(self.test_instance.percentage, 0)

    def test_instance_creation_outcomes(self):

        self.assertIsInstance(self.test_instance.outcomes, list)
        self.assertEqual(self.test_instance.outcomes, ["succeeded", "aborted", "preempted"])

    @patch('simulated_omni_drive_skill_server.simulated_omni_drive_skill_class.SimulatedOmniDriveSkill.log_info')
    @patch('actionlib.SimpleActionServer.publish_feedback')
    def test_feedback_not_empty_status(self, mock1, mock2):

        test_percentage = 42
        self.test_instance.percentage = test_percentage
        test_status = 'Test status'
        expected_feedback = SimulatedOmniDriveSkillFeedback(percentage=test_percentage, skillStatus=test_status)

        self.test_instance.feedback(test_status)

        mock1.assert_called_with(expected_feedback)
        mock2.assert_called_with(expected_feedback)

    @patch('actionlib.SimpleActionServer.set_succeeded')
    def test_success_not_empty_status_outcome(self, mock):

        test_percentage = 100
        self.test_instance.percentage = test_percentage
        test_status = 'Test Status'
        test_outcome = 'Test Outcome'
        expected_result = SimulatedOmniDriveSkillResult(percentage=test_percentage, skillStatus=test_status, outcome=test_outcome)

        self.test_instance.success(test_status, test_outcome)

        mock.assert_called_with(expected_result, test_status)

    @patch('actionlib.SimpleActionServer.set_succeeded')
    def test_success_empty_status_outcome(self, mock):

        test_percentage = 100
        self.test_instance.percentage = test_percentage
        expected_status = 'SimulatedOmniDriveSkill executed successfully'
        expected_outcome = 'succeeded'
        expected_result = SimulatedOmniDriveSkillResult(percentage=test_percentage, skillStatus=expected_status, outcome=expected_outcome)

        self.test_instance.success()

        mock.assert_called_with(expected_result, expected_status)

    @patch('actionlib.SimpleActionServer.set_aborted')
    def test_aborted_not_empty_status_outcome(self, mock):

        test_percentage = 42
        self.test_instance.percentage = test_percentage
        test_status = 'Test Status'
        test_outcome = 'Test Outcome'
        expected_result = SimulatedOmniDriveSkillResult(percentage=test_percentage, skillStatus=test_status, outcome=test_outcome)

        self.test_instance.aborted(test_status, test_outcome)

        mock.assert_called_with(expected_result, test_status)

    @patch('actionlib.SimpleActionServer.set_aborted')
    def test_aborted_empty_status_outcome(self, mock):

        test_percentage = 42
        self.test_instance.percentage = test_percentage
        expected_status = 'SimulatedOmniDriveSkill aborted'
        expected_outcome = 'aborted'
        expected_result = SimulatedOmniDriveSkillResult(percentage=test_percentage, skillStatus=expected_status, outcome=expected_outcome)

        self.test_instance.aborted()

        mock.assert_called_with(expected_result, expected_status)

    def test_check_preemption_not_preempted(self):

        actionlib.SimpleActionServer.is_preempt_requested = MagicMock(return_value=False)

        result = self.test_instance.check_preemption()

        self.assertFalse(result)

    @patch('actionlib.SimpleActionServer.set_preempted')
    def test_check_preemption_preempted(self, mock):

        actionlib.SimpleActionServer.is_preempt_requested = MagicMock(return_value=True)
        expected_status = 'SimulatedOmniDriveSkill Preempted'
        expected_outcome = 'preempted'
        expected_result = SimulatedOmniDriveSkillResult(skillStatus=expected_status, outcome=expected_outcome)

        result = self.test_instance.check_preemption()

        mock.assert_called_with(expected_result, expected_status)
        self.assertTrue(result)

    def test_result_constructor_no_percentage_no_outcome(self):

        expected_percentage = 42
        self.test_instance.percentage = expected_percentage
        expected_status = 'Test Status'
        expected_outcome = None

        result = self.test_instance.result_constructor(expected_status)

        self.assertEqual(result.percentage, expected_percentage)
        self.assertEqual(result.skillStatus, expected_status)
        self.assertEqual(result.outcome, expected_outcome)

    def test_result_constructor_with_percentage_with_outcome(self):

        expected_percentage = 42
        expected_status = 'Test Status'
        expected_outcome = 'Test Outcome'
        expected_result = SimulatedOmniDriveSkillResult(percentage=expected_percentage, skillStatus=expected_status, outcome=expected_outcome)

        result = self.test_instance.result_constructor(expected_status, percentage=expected_percentage, outcome=expected_outcome)

        self.assertEqual(result, expected_result)

    def test_log_info_valid(self):

        test_percentage = 42
        test_status = 'Test Status'
        test_outcome = 'Test Outcome'
        test_result = SimulatedOmniDriveSkillResult(percentage=test_percentage, skillStatus=test_status, outcome=test_outcome)

        expected_result = '[SimulatedOmniDriveSkill] Percentage: ' + str(test_percentage) + '%. Status: ' + test_status

        result = self.test_instance.log_info(test_result)

        self.assertEqual(result, expected_result)

if __name__ == '__main__':
    
    import rosunit
    rosunit.unitrun('simulated_omni_drive_skill_server', 'test_simulated_omni_drive_skill_class', TestSimulatedOmniDriveSkillClass)
