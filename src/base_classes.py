#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time
import tfx
import smach
import rospy

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

SLEEP_TIME = 0

class StateTestClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """

    def __init__(self):
        self.counter = -1
        self.outcomes = None
        singlePose =tfx.pose([0.02068052528579386, 0.07571039991719071, -0.13466460554427698],
            (-0.7419649188498894, 0.6226963561227075, 0.012382684241208437, 0.2481611903553049))
        #Need to find new singlePose and hardcode it in
        self.dropOffStagingPose = singlePose
        self.dropOffPose = singlePose
        self.dropPose = singlePose

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

class Start(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Start"
        return 'success'

class MoveToRetractionStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm        
    
    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"

        return 'success'

class HomePosition(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: HomePosition"
        return 'success'

class MoveToGraspPoint(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: MoveToGraspPoint"
        return 'success'

class GraspBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: GraspBlock"
        return "success"        

class CheckGrasp(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: CheckGrasp"
        return 'success'

class ReleaseGrippersNoBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: ReleaseGrippersNoBlock"
        return 'success'

class ReturnToRetractionStagingAreaWithBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReturnToRetractionStagingAreaWithBlock"
        return 'success'

class MoveToDropOffStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"
        return 'success'

class MoveToDropOffPoint(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffPoint"
        return 'success'

class ReleaseGripper(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReleaseGripper"
        return 'success'

class CheckDropOff(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: CheckDropOff"
        return 'success'

class Abort(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Abort"
        return 'success'