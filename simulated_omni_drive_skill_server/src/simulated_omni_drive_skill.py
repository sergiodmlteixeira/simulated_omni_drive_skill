#!/usr/bin/env python

import rospy
import traceback
from simulated_omni_drive_skill_class import SimulatedOmniDriveSkill


if __name__ == "__main__":

    rospy.init_node('simulated_omni_drive_skill')

    try:
        actionName = rospy.get_param('~action_name')
    except Exception as e:
        raise KeyError('Unable to access ROS parameter server for ' + str(actionName))

    try:
        SimulatedOmniDriveSkill = SimulatedOmniDriveSkill(actionName)
        rospy.spin()

    except Exception as e:
        rospy.logerr('[SimulatedOmniDriveSkill] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())
        quit()
