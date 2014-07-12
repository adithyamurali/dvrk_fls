#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import time
import smach
import rospy

from base_classes import *

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

class MasterClass:
    def __init__(self):
        rospy.init_node('Master_SM_FLS',anonymous=False)
        self.state_machine = smach.StateMachine(outcomes=['SUCCESS', 'FAILURE'])
        self.davinciArm = RavenArm(raven_constants.Arm.Right)
        self.setup_state_machine()

    def setup_state_machine(self):
        smach.loginfo("Initializing state machine")
        rospy.sleep(1)

        with self.state_machine:
            smach.StateMachine.add('START',
                Start(self.davinciArm),
                transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA'})

            smach.StateMachine.add('MOVE_TO_RETRACTION_STAGING_AREA',
                MoveToRetractionStagingArea(self.davinciArm),
                transitions={'success':'MOVE_TO_GRASP_POINT', 'failure': 'HOME_POSITION'})

            smach.StateMachine.add('HOME_POSITION',
                HomePosition(self.davinciArm),
                transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA', 'failure': 'HOME_POSITION'})

            smach.StateMachine.add('MOVE_TO_GRASP_POINT',
                MoveToGraspPoint(self.davinciArm),
                transitions={'success':'GRASP_BLOCK'})

            smach.StateMachine.add('GRASP_BLOCK',
                GraspBlock(self.davinciArm),
                transitions={'success':'RETURN_TO_RETRACTION_STAGING_AREA_WITH_BLOCK', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK'})

            smach.StateMachine.add('CHECK_GRASP',
                CheckGrasp(self.davinciArm),
                transitions={'success':'MOVE_TO_DROP_OFF_STAGING_AREA', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK'})

            smach.StateMachine.add('RELEASE_GRIPPERS_NO_BLOCK',
                ReleaseGrippersNoBlock(self.davinciArm),
                transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK'})

            smach.StateMachine.add('RETURN_TO_RETRACTION_STAGING_AREA_WITH_BLOCK',
                ReturnToRetractionStagingAreaWithBlock(self.davinciArm),
                transitions={'success':'CHECK_GRASP', 'failure': 'ABORT'})

            smach.StateMachine.add('MOVE_TO_DROP_OFF_STAGING_AREA',
                MoveToDropOffStagingArea(self.davinciArm),
                transitions={'success':'MOVE_TO_DROP_OFF_POINT', 'failure': 'ABORT'})

            smach.StateMachine.add('MOVE_TO_DROP_OFF_POINT',
                MoveToDropOffPoint(self.davinciArm),
                transitions={'success':'RELEASE_GRIPPERS', 'failure': 'ABORT'})

            smach.StateMachine.add('RELEASE_GRIPPERS',
                ReleaseGripper(self.davinciArm),
                transitions={'success':'CHECK_DROP_OFF', 'failure': 'RELEASE_GRIPPERS'})

            smach.StateMachine.add('CHECK_DROP_OFF',
                CheckDropOff(self.davinciArm),
                transitions={'success':'SUCCESS', 'failure': 'ABORT'})

            smach.StateMachine.add('ABORT', Abort(self.davinciArm), transitions={'failure': 'FAILURE'})

    def run(self):
        self.davinciArm.start()
        rospy.sleep(2)
        
        rate = rospy.Rate(1)
        try:
            self.state_machine.execute()
        except Exception, e:
            print e

        while not rospy.is_shutdown():
            if not self.state_machine.is_running():
                print "State Machine stopped"
                rospy.sleep(0.5)
                rospy.signal_shutdown('state machines finished')
                break
            rate.sleep()

        self.davinciArm.stop()

def main():
    smach.loginfo("Starting main...")
    master = MasterClass()
    master.run()


if __name__ == '__main__':
    main()



