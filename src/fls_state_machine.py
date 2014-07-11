#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import time
import smach

from base_classes import *

def setup_state_machine():
    smach.loginfo("Initializing state machine")
    state_machine = smach.StateMachine(outcomes=['SUCCESS', 'FAILURE'])
    with state_machine:
        smach.StateMachine.add('START',
            Start(),
            transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA'})

        smach.StateMachine.add('MOVE_TO_RETRACTION_STAGING_AREA',
            MoveToRetractionStartingArea(),
            transitions={'success':'MOVE_TO_GRASP_POINT', 'failure': 'HOME_POSITION'})

        smach.StateMachine.add('HOME_POSITION',
            HomePosition(),
            transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA', 'failure': 'HOME_POSITION'})

        smach.StateMachine.add('MOVE_TO_GRASP_POINT',
            MoveToGraspPoint(),
            transitions={'success':'GRASP_BLOCK', 'failure': 'MOVE_TO_RETRACTION_STAGING_AREA'})

        smach.StateMachine.add('GRASP_BLOCK',
            GraspBlock(),
            transitions={'success':'RETURN_TO_RETRACTION_STAGING_AREA_WITH_BLOCK', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK')

        smach.StateMachine.add('CHECK_GRASP',
            CheckGrasp(),
            transitions={'success':'MOVE_TO_DROP_OFF_STAGING_AREA', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK'})

        smach.StateMachine.add('RELEASE_GRIPPERS_NO_BLOCK',
            ReleaseGrippersNoBlock(),
            transitions={'success':'MOVE_TO_RETRACTION_STAGING_AREA', 'failure': 'RELEASE_GRIPPERS_NO_BLOCK'})

        smach.StateMachine.add('RETURN_TO_RETRACTION_STAGING_AREA_WITH_BLOCK',
            ReturnToRetractionStagingAreaWithBlock(),
            transitions={'success':'CHECK_GRASP', 'failure': 'ABORT'})

        smach.StateMachine.add('MOVE_TO_DROP_OFF_STAGING_AREA',
            MoveToDropOffStagingArea(),
            transitions={'success':'MOVE_TO_DROP_OFF_POINT', 'failure': 'ABORT'})

        smach.StateMachine.add('MOVE_TO_DROP_OFF_POINT',
            MoveToDropOffPoint(),
            transitions={'success':'RELEASE_GRIPPERS', 'failure': 'ABORT'})

        smach.StateMachine.add('RELEASE_GRIPPERS',
            ReleaseGripper(),
            transitions={'success':'CHECK_DROP_OFF', 'failure': 'RELEASE_GRIPPERS'})

        smach.StateMachine.add('CHECK_DROP_OFF',
            CheckDropOff(),
            transitions={'success':'SUCCESS', 'failure': 'ABORT'})

    state_machine.execute()

def main():
    smach.loginfo("Starting main...")
    setup_state_machine()


if __name__ == '__main__':
    main()



